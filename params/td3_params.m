function params = td3_params()
%TD3_PARAMS Return TD3 reinforcement learning hyperparameters
%   Parameters for Twin Delayed DDPG algorithm
%
%   Returns:
%       params - Structure containing all TD3 hyperparameters
%
%   Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"
%              Fujimoto et al. (2018) "Addressing Function Approximation
%              Error in Actor-Critic Methods"

    %% Network Architecture

    % Actor network (policy)
    params.actor.hidden_layers = [256, 256];    % Hidden layer sizes
    params.actor.activation = 'relu';           % Activation function
    params.actor.output_activation = 'tanh';    % Output bounded to [-1, 1]
    params.actor.lr = 1e-4;                     % Learning rate
    params.actor.l2_reg = 1e-4;                 % L2 regularization

    % Critic networks (Q-functions) - Twin critics
    params.critic.hidden_layers = [256, 256];   % Hidden layer sizes
    params.critic.activation = 'relu';          % Activation function
    params.critic.lr = 1e-3;                    % Learning rate
    params.critic.l2_reg = 1e-4;                % L2 regularization

    %% TD3 Core Hyperparameters

    % Discount factor
    params.gamma = 0.99;

    % Soft target update coefficient
    params.tau = 0.005;

    % Policy update delay (update policy every d gradient steps)
    params.policy_delay = 2;

    % Target policy smoothing
    params.target_noise = 0.2;         % Noise added to target action
    params.target_noise_clip = 0.5;    % Noise clipping range

    % Exploration noise
    params.exploration_noise = 0.1;    % Gaussian noise std for exploration

    %% Replay Buffer

    params.buffer.capacity = 1e6;      % Maximum buffer size
    params.buffer.batch_size = 256;    % Mini-batch size for training
    params.buffer.min_size = 10000;    % Minimum samples before training

    %% Training Parameters

    params.training.num_episodes = 500;        % Total training episodes
    params.training.max_steps = 1000;          % Max steps per episode
    params.training.warmup_steps = 10000;      % Random actions before learning
    params.training.update_freq = 1;           % Gradient updates per step
    params.training.eval_freq = 10;            % Evaluate every N episodes
    params.training.save_freq = 50;            % Save model every N episodes

    %% State Space
    % S_w = [e(t), e_i(t), e_dot(t), phi, theta, psi]'

    params.state.dim = 6;              % State dimension
    params.state.names = {'error', 'error_integral', 'error_derivative', ...
                          'roll', 'pitch', 'yaw'};

    % State normalization bounds
    params.state.bounds = [
        -0.5, 0.5;      % error [rad]
        -1.0, 1.0;      % error_integral [rad*s]
        -2.0, 2.0;      % error_derivative [rad/s]
        -0.8, 0.8;      % roll [rad]
        -0.8, 0.8;      % pitch [rad]
        -3.2, 3.2;      % yaw [rad]
    ];

    %% Action Space
    % a_w = [w_cr(t), w_cp(t), w_cy(t)]' - ADRC parameters for each channel

    params.action.dim = 3;             % Action dimension
    params.action.names = {'wc_roll', 'wc_pitch', 'wc_yaw'};

    % Action bounds (normalized to [-1, 1] internally)
    params.action.bounds = [
        5, 30;          % wc_roll: controller bandwidth [rad/s]
        5, 30;          % wc_pitch: controller bandwidth [rad/s]
        2, 15;          % wc_yaw: controller bandwidth [rad/s]
    ];

    %% Reward Function
    % R_w = gamma1*e(t) + gamma2*e_i(t) + gamma3*e_dot(t) + gamma4*X
    % where X = 1 if |u| <= 1, else 0

    params.reward.gamma1 = -10.0;      % Error penalty weight
    params.reward.gamma2 = -1.0;       % Integral error penalty weight
    params.reward.gamma3 = -0.5;       % Error derivative penalty weight
    params.reward.gamma4 = 1.0;        % Control bound reward
    params.reward.alive_bonus = 0.1;   % Bonus for staying stable

    % Episode termination thresholds
    params.reward.max_error = 1.0;             % Max error before termination [rad]
    params.reward.max_attitude = deg2rad(60);  % Max attitude before termination [rad]

    %% Environment Settings

    params.env.dt = 0.01;              % Environment step time [s]
    params.env.episode_time = 10.0;    % Episode duration [s]

    % Disturbance settings for training
    params.env.disturbance.enable = true;
    params.env.disturbance.wind_max = 0.5;     % Max wind disturbance magnitude
    params.env.disturbance.gust_prob = 0.1;    % Probability of gust per step

    % Reference trajectory settings
    params.env.reference.type = 'mixed';       % 'step', 'sinusoid', 'mixed'
    params.env.reference.amplitude = deg2rad(10);  % Max reference amplitude

    %% Hardware Constraints

    params.hardware.max_motor_speed = 8000;    % [rad/s]
    params.hardware.max_torque = 0.5;          % [N*m]
    params.hardware.battery_voltage = 22.2;    % [V] (6S LiPo)

    %% Random Seeds (for reproducibility)

    params.seed = 42;

end
