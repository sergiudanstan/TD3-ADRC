function td3_adrc_mcp_tools()
%TD3_ADRC_MCP_TOOLS MCP server tools for TD3-ADRC controller
%   Provides interactive access to TD3-ADRC functionality via MCP
%
%   Available commands:
%       td3_adrc_mcp_tools('status')      - Get controller status
%       td3_adrc_mcp_tools('train', N)    - Train for N episodes
%       td3_adrc_mcp_tools('eval')        - Evaluate current policy
%       td3_adrc_mcp_tools('demo', name)  - Run a demo
%       td3_adrc_mcp_tools('compare')     - Run controller comparison
%       td3_adrc_mcp_tools('export')      - Export for Blender

    fprintf('TD3-ADRC MCP Tools\n');
    fprintf('==================\n\n');
    fprintf('Available functions:\n');
    fprintf('  status()       - Get controller and agent status\n');
    fprintf('  train(N)       - Train TD3 agent for N episodes\n');
    fprintf('  evaluate()     - Evaluate trained agent\n');
    fprintf('  run_demo(name) - Run demo (wind/trajectory/comparison)\n');
    fprintf('  compare()      - Compare all controllers\n');
    fprintf('  export(data)   - Export data for Blender\n');
    fprintf('  help()         - Show this help\n\n');

end


function info = status()
%STATUS Get current controller and agent status
%   info = status()

    info = struct();

    % Check if TD3 agent exists
    if evalin('base', 'exist(''td3_agent'', ''var'')')
        agent = evalin('base', 'td3_agent');
        info.agent_exists = true;
        info.total_steps = agent.total_steps;
        info.update_count = agent.update_count;
        info.buffer_size = agent.buffer.get_size();

        if ~isempty(agent.training_log.episode_rewards)
            info.last_reward = agent.training_log.episode_rewards(end);
            info.mean_reward = mean(agent.training_log.episode_rewards(max(1,end-100):end));
        end
    else
        info.agent_exists = false;
        info.message = 'No TD3 agent in workspace. Run train() first.';
    end

    % Check for controller
    if evalin('base', 'exist(''controller'', ''var'')')
        info.controller_exists = true;
    else
        info.controller_exists = false;
    end

    disp(info);

end


function agent = train(num_episodes, save_interval)
%TRAIN Train TD3-ADRC agent
%   agent = train(num_episodes)
%   agent = train(num_episodes, save_interval)
%
%   Inputs:
%       num_episodes  - Number of training episodes (default: 500)
%       save_interval - Save model every N episodes (default: 50)

    if nargin < 1 || isempty(num_episodes)
        num_episodes = 500;
    end
    if nargin < 2 || isempty(save_interval)
        save_interval = 50;
    end

    fprintf('Starting TD3-ADRC Training\n');
    fprintf('==========================\n');
    fprintf('Episodes: %d\n', num_episodes);
    fprintf('Save interval: %d\n\n', save_interval);

    % Setup paths
    setup_paths;

    % Create environment and agent
    env = QuadrotorEnv();
    params = td3_params();

    state_dim = params.state.dim;
    action_dim = params.action.dim;
    action_bounds = params.action.bounds;

    agent = td3_agent(state_dim, action_dim, action_bounds, params);

    % Training loop
    for ep = 1:num_episodes
        [episode_reward, steps] = agent.train_episode(env, 1000, 256);

        if mod(ep, 10) == 0
            fprintf('Episode %d/%d | Reward: %.2f | Steps: %d | Buffer: %d\n', ...
                ep, num_episodes, episode_reward, steps, agent.buffer.get_size());
        end

        if mod(ep, save_interval) == 0
            filename = sprintf('results/td3_agent_ep%d.mat', ep);
            agent.save(filename);
            fprintf('  Saved checkpoint: %s\n', filename);
        end
    end

    % Final save
    agent.save('results/td3_agent_final.mat');
    fprintf('\nTraining complete. Final model saved.\n');

    % Store in base workspace
    assignin('base', 'td3_agent', agent);

end


function results = evaluate(agent, num_episodes)
%EVALUATE Evaluate trained TD3-ADRC agent
%   results = evaluate() - Uses agent from workspace
%   results = evaluate(agent) - Uses provided agent
%   results = evaluate(agent, num_episodes) - Specify number of episodes

    if nargin < 1 || isempty(agent)
        if evalin('base', 'exist(''td3_agent'', ''var'')')
            agent = evalin('base', 'td3_agent');
        else
            error('No agent provided and none in workspace');
        end
    end
    if nargin < 2 || isempty(num_episodes)
        num_episodes = 10;
    end

    fprintf('Evaluating TD3-ADRC Agent\n');
    fprintf('=========================\n');

    env = QuadrotorEnv();
    rewards = agent.evaluate(env, num_episodes, 1000);

    results.mean_reward = mean(rewards);
    results.std_reward = std(rewards);
    results.max_reward = max(rewards);
    results.min_reward = min(rewards);
    results.all_rewards = rewards;

    fprintf('Mean reward: %.2f +/- %.2f\n', results.mean_reward, results.std_reward);
    fprintf('Max reward: %.2f\n', results.max_reward);
    fprintf('Min reward: %.2f\n', results.min_reward);

end


function run_demo(demo_name)
%RUN_DEMO Run a demonstration
%   run_demo('wind')        - Wind disturbance rejection
%   run_demo('trajectory')  - Aggressive maneuver tracking
%   run_demo('comparison')  - Full controller comparison

    if nargin < 1 || isempty(demo_name)
        demo_name = 'comparison';
    end

    fprintf('Running demo: %s\n', demo_name);
    fprintf('===================\n\n');

    switch lower(demo_name)
        case 'wind'
            run('demo_wind_disturbance.m');

        case 'trajectory'
            run('demo_trajectory_tracking.m');

        case 'comparison'
            run('demo_comparison.m');

        otherwise
            fprintf('Unknown demo: %s\n', demo_name);
            fprintf('Available demos: wind, trajectory, comparison\n');
    end

end


function compare()
%COMPARE Run full controller comparison
    run_demo('comparison');
end


function export(flight_data, filename)
%EXPORT Export flight data for Blender visualization
%   export(flight_data, filename)

    if nargin < 2 || isempty(filename)
        filename = 'results/animations/flight_data.json';
    end

    export_to_blender(flight_data, filename);
    fprintf('Exported to: %s\n', filename);

end


%% Simple Quadrotor Environment for Training
classdef QuadrotorEnv < handle
    properties
        state
        t
        Ts
        params
        ref
        disturbance_params
    end

    methods
        function obj = QuadrotorEnv()
            obj.Ts = 0.01;
            obj.params = quadrotor_params();
            obj.disturbance_params.amplitude = [0.02; 0.02; 0.01];
            obj.reset();
        end

        function state = reset(obj)
            obj.t = 0;
            % Random initial attitude
            obj.state = [
                0.1 * randn(3, 1);   % Small random attitude
                zeros(3, 1)           % Zero angular rate
            ];
            % Random reference
            obj.ref = deg2rad([
                10 * (2*rand - 1);
                10 * (2*rand - 1);
                15 * (2*rand - 1)
            ]);

            state = obj.get_obs();
        end

        function [next_state, reward, done, info] = step(obj, action)
            % Apply action (ADRC bandwidth parameters)
            wc = action;  % [wc_roll, wc_pitch, wc_yaw]

            % Simple dynamics simulation
            euler = obj.state(1:3);
            omega = obj.state(4:6);

            % Simple PD with adaptive gains
            e = obj.ref - euler;
            kp = wc.^2;
            kd = 2 * wc;
            tau = kp .* e - kd .* omega;

            % Disturbance
            tau_dist = obj.disturbance_params.amplitude .* sin(2*pi*0.5*obj.t);

            % Dynamics
            I = diag([obj.params.Ixx, obj.params.Iyy, obj.params.Izz]);
            omega_dot = I \ (tau + tau_dist);

            % Kinematics (simplified)
            euler_dot = omega;

            % Integrate
            obj.state(1:3) = obj.state(1:3) + obj.Ts * euler_dot;
            obj.state(4:6) = obj.state(4:6) + obj.Ts * omega_dot;
            obj.t = obj.t + obj.Ts;

            % Compute reward
            e_new = obj.ref - obj.state(1:3);
            reward = -10 * norm(e_new) - 0.1 * norm(tau) + 0.1;

            % Check termination
            done = obj.t >= 10 || max(abs(obj.state(1:3))) > deg2rad(60);

            next_state = obj.get_obs();
            info = struct();
        end

        function obs = get_obs(obj)
            e = obj.ref - obj.state(1:3);
            e_int = zeros(3, 1);  % Simplified
            e_dot = -obj.state(4:6);

            obs = [mean(abs(e)); mean(abs(e_int)); mean(abs(e_dot)); obj.state(1:3)];
        end
    end
end
