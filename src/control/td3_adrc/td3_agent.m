classdef td3_agent < handle
%TD3_AGENT Twin Delayed Deep Deterministic Policy Gradient agent
%   Complete TD3 implementation for continuous action spaces
%
%   Key features:
%   - Twin Q-networks (mitigate overestimation)
%   - Delayed policy updates (training stability)
%   - Target policy smoothing (variance reduction)
%
%   Reference: Fujimoto et al. (2018) "Addressing Function Approximation
%              Error in Actor-Critic Methods"

    properties
        % Networks
        actor
        actor_target
        critic
        critic_target

        % Replay buffer
        buffer

        % Hyperparameters
        gamma           % Discount factor
        tau             % Soft update coefficient
        policy_delay    % Update policy every N critic updates
        target_noise    % Noise for target policy smoothing
        noise_clip      % Clipping for target noise
        exploration_noise

        % Dimensions
        state_dim
        action_dim
        action_bounds

        % Training state
        total_steps
        update_count

        % Logging
        training_log
    end

    methods
        function obj = td3_agent(state_dim, action_dim, action_bounds, params)
            %TD3_AGENT Construct TD3 agent
            %   agent = td3_agent(state_dim, action_dim, action_bounds)
            %   agent = td3_agent(state_dim, action_dim, action_bounds, params)
            %
            %   Inputs:
            %       state_dim     - Dimension of state space
            %       action_dim    - Dimension of action space
            %       action_bounds - Action bounds [low high] (action_dim x 2)
            %       params        - TD3 parameters from td3_params()

            if nargin < 4
                params = td3_params();
            end

            obj.state_dim = state_dim;
            obj.action_dim = action_dim;
            obj.action_bounds = action_bounds;

            % Hyperparameters
            obj.gamma = params.gamma;
            obj.tau = params.tau;
            obj.policy_delay = params.policy_delay;
            obj.target_noise = params.target_noise;
            obj.noise_clip = params.target_noise_clip;
            obj.exploration_noise = params.exploration_noise;

            % Initialize networks
            hidden_dims = params.actor.hidden_layers;
            obj.actor = actor_network(state_dim, action_dim, hidden_dims, action_bounds);
            obj.actor_target = actor_network(state_dim, action_dim, hidden_dims, action_bounds);
            obj.actor_target.copy_from(obj.actor, 1);  % Hard copy
            obj.actor.learning_rate = params.actor.lr;

            hidden_dims = params.critic.hidden_layers;
            obj.critic = critic_network(state_dim, action_dim, hidden_dims);
            obj.critic_target = critic_network(state_dim, action_dim, hidden_dims);
            obj.critic_target.copy_from(obj.critic, 1);  % Hard copy
            obj.critic.learning_rate = params.critic.lr;

            % Initialize replay buffer
            obj.buffer = replay_buffer(params.buffer.capacity, state_dim, action_dim);

            % Training state
            obj.total_steps = 0;
            obj.update_count = 0;

            % Logging
            obj.training_log.critic_loss = [];
            obj.training_log.actor_loss = [];
            obj.training_log.episode_rewards = [];
        end

        function action = select_action(obj, state, explore)
            %SELECT_ACTION Select action for given state
            %   action = agent.select_action(state) deterministic
            %   action = agent.select_action(state, true) with exploration
            %
            %   Inputs:
            %       state   - Current state (state_dim x 1 or 1 x state_dim)
            %       explore - Add exploration noise (default: false)
            %
            %   Outputs:
            %       action  - Selected action (action_dim x 1)

            if nargin < 3
                explore = false;
            end

            if explore
                action = obj.actor.select_action(state, obj.exploration_noise);
            else
                action = obj.actor.select_action(state, 0);
            end
        end

        function store_transition(obj, state, action, reward, next_state, done)
            %STORE_TRANSITION Store experience in replay buffer
            obj.buffer.add(state, action, reward, next_state, done);
            obj.total_steps = obj.total_steps + 1;
        end

        function [critic_loss, actor_loss] = train_step(obj, batch_size)
            %TRAIN_STEP Perform one training update
            %   [critic_loss, actor_loss] = agent.train_step(batch_size)
            %
            %   Inputs:
            %       batch_size - Number of samples to train on
            %
            %   Outputs:
            %       critic_loss - MSE loss from critic update
            %       actor_loss  - Policy loss (mean Q-value)

            if nargin < 2
                batch_size = 256;
            end

            actor_loss = 0;

            % Sample from replay buffer
            [states, actions, rewards, next_states, dones] = ...
                obj.buffer.sample(batch_size);

            %% Compute target Q-value
            % Get target actions from target policy with noise
            next_actions = obj.actor_target.forward(next_states);

            % Target policy smoothing: add clipped noise
            noise = obj.target_noise * randn(size(next_actions));
            noise = max(-obj.noise_clip, min(obj.noise_clip, noise));
            next_actions = next_actions + noise;

            % Clip to action bounds
            action_low = obj.action_bounds(:, 1)';
            action_high = obj.action_bounds(:, 2)';
            next_actions = max(action_low, min(action_high, next_actions));

            % Twin Q-values from target critics
            [Q1_target, Q2_target] = obj.critic_target.forward(next_states, next_actions);

            % Use minimum to reduce overestimation
            Q_target = min(Q1_target, Q2_target);

            % Bellman backup
            targets = rewards + obj.gamma * (1 - dones) .* Q_target;

            %% Update critics
            [grads1, grads2, critic_loss] = obj.critic.compute_gradients(states, actions, targets);
            obj.critic.update(grads1, grads2);

            obj.update_count = obj.update_count + 1;

            %% Delayed policy update
            if mod(obj.update_count, obj.policy_delay) == 0
                % Update actor
                [actor_grads, actor_loss] = obj.actor.compute_gradients(states, obj.critic);
                obj.actor.update(actor_grads);

                % Soft update target networks
                obj.actor_target.copy_from(obj.actor, obj.tau);
                obj.critic_target.copy_from(obj.critic, obj.tau);
            end

            % Log
            obj.training_log.critic_loss(end+1) = critic_loss;
            if actor_loss ~= 0
                obj.training_log.actor_loss(end+1) = actor_loss;
            end
        end

        function [episode_reward, steps] = train_episode(obj, env, max_steps, batch_size)
            %TRAIN_EPISODE Train for one episode
            %   [reward, steps] = agent.train_episode(env, max_steps, batch_size)
            %
            %   Inputs:
            %       env        - Environment with reset() and step() methods
            %       max_steps  - Maximum steps per episode
            %       batch_size - Batch size for training

            if nargin < 3
                max_steps = 1000;
            end
            if nargin < 4
                batch_size = 256;
            end

            state = env.reset();
            episode_reward = 0;
            steps = 0;

            for t = 1:max_steps
                % Select action with exploration
                action = obj.select_action(state, true);

                % Take step in environment
                [next_state, reward, done, ~] = env.step(action);

                % Store transition
                obj.store_transition(state, action, reward, next_state, done);

                episode_reward = episode_reward + reward;
                steps = steps + 1;
                state = next_state;

                % Train if buffer has enough samples
                if obj.buffer.is_ready_for_training(batch_size)
                    obj.train_step(batch_size);
                end

                if done
                    break;
                end
            end

            obj.training_log.episode_rewards(end+1) = episode_reward;
        end

        function rewards = evaluate(obj, env, num_episodes, max_steps)
            %EVALUATE Evaluate agent without exploration
            %   rewards = agent.evaluate(env, num_episodes, max_steps)
            %
            %   Returns episode rewards for evaluation

            if nargin < 3
                num_episodes = 10;
            end
            if nargin < 4
                max_steps = 1000;
            end

            rewards = zeros(num_episodes, 1);

            for ep = 1:num_episodes
                state = env.reset();
                episode_reward = 0;

                for t = 1:max_steps
                    action = obj.select_action(state, false);  % No exploration
                    [next_state, reward, done, ~] = env.step(action);
                    episode_reward = episode_reward + reward;
                    state = next_state;

                    if done
                        break;
                    end
                end

                rewards(ep) = episode_reward;
            end
        end

        function save(obj, filepath)
            %SAVE Save agent to file
            %   agent.save('path/to/agent.mat')

            agent_data.gamma = obj.gamma;
            agent_data.tau = obj.tau;
            agent_data.policy_delay = obj.policy_delay;
            agent_data.target_noise = obj.target_noise;
            agent_data.noise_clip = obj.noise_clip;
            agent_data.exploration_noise = obj.exploration_noise;
            agent_data.state_dim = obj.state_dim;
            agent_data.action_dim = obj.action_dim;
            agent_data.action_bounds = obj.action_bounds;
            agent_data.total_steps = obj.total_steps;
            agent_data.update_count = obj.update_count;
            agent_data.training_log = obj.training_log;

            % Save to temp files then package
            [fpath, fname, ~] = fileparts(filepath);
            if isempty(fpath)
                fpath = '.';
            end

            actor_file = fullfile(fpath, [fname '_actor.mat']);
            critic_file = fullfile(fpath, [fname '_critic.mat']);

            obj.actor.save(actor_file);
            obj.critic.save(critic_file);

            save(filepath, 'agent_data', 'actor_file', 'critic_file');
        end

        function load(obj, filepath)
            %LOAD Load agent from file
            data = load(filepath);

            obj.gamma = data.agent_data.gamma;
            obj.tau = data.agent_data.tau;
            obj.policy_delay = data.agent_data.policy_delay;
            obj.target_noise = data.agent_data.target_noise;
            obj.noise_clip = data.agent_data.noise_clip;
            obj.exploration_noise = data.agent_data.exploration_noise;
            obj.total_steps = data.agent_data.total_steps;
            obj.update_count = data.agent_data.update_count;
            obj.training_log = data.agent_data.training_log;

            obj.actor.load(data.actor_file);
            obj.actor_target.copy_from(obj.actor, 1);
            obj.critic.load(data.critic_file);
            obj.critic_target.copy_from(obj.critic, 1);
        end

        function plot_training(obj)
            %PLOT_TRAINING Plot training progress

            figure('Name', 'TD3 Training Progress');

            subplot(3, 1, 1);
            plot(obj.training_log.episode_rewards, 'b-');
            xlabel('Episode');
            ylabel('Episode Reward');
            title('Training Rewards');
            grid on;

            subplot(3, 1, 2);
            plot(obj.training_log.critic_loss, 'r-');
            xlabel('Update Step');
            ylabel('Critic Loss');
            title('Critic Loss');
            grid on;

            subplot(3, 1, 3);
            plot(obj.training_log.actor_loss, 'g-');
            xlabel('Policy Update');
            ylabel('Actor Loss');
            title('Actor Loss (neg Q-value)');
            grid on;
        end
    end

end
