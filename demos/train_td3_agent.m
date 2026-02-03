%% train_td3_agent.m - Train TD3 agent for ADRC parameter tuning
% Trains the TD3 agent to optimize ADRC controller bandwidth parameters
%
% Usage:
%   trained_agent = train_td3_agent(200);  % Train for 200 episodes
%
% Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"

function trained_agent = train_td3_agent(num_episodes, verbose)
    if nargin < 1 || isempty(num_episodes)
        num_episodes = 200;
    end
    if nargin < 2
        verbose = true;
    end

    %% Setup
    setup_paths;
    rng(42);  % For reproducibility

    fprintf('==========================================\n');
    fprintf('    TD3-ADRC Agent Training\n');
    fprintf('==========================================\n');
    fprintf('Episodes: %d\n', num_episodes);
    fprintf('Starting training...\n\n');

    %% Create environment
    env = AttitudeControlEnv();

    %% Create TD3 agent
    params = td3_params();
    state_dim = 6;
    action_dim = 3;
    action_bounds = [5, 30; 5, 30; 3, 15];  % Bandwidth bounds

    agent = td3_agent(state_dim, action_dim, action_bounds, params);
    agent.exploration_noise = 0.15;

    %% Training parameters
    max_steps_per_episode = 500;
    batch_size = 128;
    warmup_episodes = 10;
    eval_interval = 20;

    %% Training loop
    episode_rewards = zeros(num_episodes, 1);
    best_reward = -inf;
    training_start = tic;

    for ep = 1:num_episodes
        % Reset environment
        state = env.reset();
        episode_reward = 0;
        steps = 0;

        for step = 1:max_steps_per_episode
            % Select action
            if ep <= warmup_episodes
                % Random exploration during warmup
                action = action_bounds(:,1) + rand(3,1) .* (action_bounds(:,2) - action_bounds(:,1));
            else
                % Use policy with exploration noise
                action = agent.select_action(state, true);
            end

            % Take step in environment
            [next_state, reward, done] = env.step(action);

            % Store transition
            agent.store_transition(state, action, reward, next_state, done);

            episode_reward = episode_reward + reward;
            steps = steps + 1;
            state = next_state;

            % Train agent
            if agent.buffer.get_size() >= batch_size && ep > warmup_episodes
                agent.train_step(batch_size);
            end

            if done
                break;
            end
        end

        episode_rewards(ep) = episode_reward;

        % Logging
        if verbose && (mod(ep, 10) == 0 || ep == 1)
            avg_reward = mean(episode_rewards(max(1,ep-20):ep));
            fprintf('Episode %3d/%d | Reward: %7.2f | Avg(20): %7.2f | Steps: %3d | Buffer: %d\n', ...
                ep, num_episodes, episode_reward, avg_reward, steps, agent.buffer.get_size());
        end

        % Track best
        if episode_reward > best_reward && ep > warmup_episodes
            best_reward = episode_reward;
        end

        % Periodic evaluation
        if mod(ep, eval_interval) == 0 && ep > warmup_episodes
            eval_reward = evaluate_agent(agent, env, 5);
            if verbose
                fprintf('  -- Evaluation (5 ep): %.2f +/- %.2f\n', ...
                    mean(eval_reward), std(eval_reward));
            end
        end
    end

    training_time = toc(training_start);

    %% Training summary
    fprintf('\n==========================================\n');
    fprintf('Training Complete!\n');
    fprintf('==========================================\n');
    fprintf('Total time: %.1f seconds\n', training_time);
    fprintf('Final avg reward (last 20): %.2f\n', mean(episode_rewards(max(1,end-19):end)));
    fprintf('Best episode reward: %.2f\n', best_reward);

    %% Save trained agent
    save_path = fullfile('results', 'td3_agent_trained.mat');
    agent.save(save_path);
    fprintf('Agent saved to: %s\n', save_path);

    %% Plot training progress
    fig = figure('Name', 'TD3 Training Progress', 'Position', [100 100 800 400]);

    subplot(1,2,1);
    plot(episode_rewards, 'b-', 'LineWidth', 0.5);
    hold on;
    if num_episodes > 20
        ma = movmean(episode_rewards, 20);
        plot(ma, 'r-', 'LineWidth', 2);
        legend('Episode Reward', '20-Episode MA', 'Location', 'southeast');
    end
    hold off;
    xlabel('Episode');
    ylabel('Reward');
    title('Training Rewards');
    grid on;

    subplot(1,2,2);
    if ~isempty(agent.training_log.critic_loss)
        semilogy(agent.training_log.critic_loss, 'r-', 'LineWidth', 0.5);
        xlabel('Update Step');
        ylabel('Critic Loss');
        title('Critic Loss');
        grid on;
    end

    saveas(fig, fullfile('results', 'figures', 'training_progress.png'));
    fprintf('Training plot saved.\n');

    trained_agent = agent;
end


function rewards = evaluate_agent(agent, env, num_episodes)
    %EVALUATE_AGENT Evaluate agent without exploration noise
    rewards = zeros(num_episodes, 1);

    for ep = 1:num_episodes
        state = env.reset();
        episode_reward = 0;

        for step = 1:500
            action = agent.select_action(state, false);
            [next_state, reward, done] = env.step(action);
            episode_reward = episode_reward + reward;
            state = next_state;
            if done
                break;
            end
        end
        rewards(ep) = episode_reward;
    end
end
