classdef td3_adrc_controller < handle
%TD3_ADRC_CONTROLLER Combined TD3 + ADRC controller for quadrotor attitude
%   Uses TD3 to adaptively tune ADRC parameters for optimal performance
%
%   The TD3 agent outputs ADRC controller bandwidth parameters:
%   a = [wc_roll, wc_pitch, wc_yaw]'
%
%   These are used by the NLSEF to adapt the control law in real-time.
%
%   Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"

    properties
        % ADRC components for each axis
        td_roll, td_pitch, td_yaw           % Tracking differentiators
        eso_roll, eso_pitch, eso_yaw        % Extended state observers
        nlsef_roll, nlsef_pitch, nlsef_yaw  % Nonlinear feedback controllers

        % TD3 agent for parameter tuning
        td3_agent
        use_td3         % Enable/disable TD3 adaptation

        % Control state
        error_integral  % Accumulated error for reward
        last_error      % Previous error for derivative

        % Parameters
        params_adrc
        params_td3
        params_quad

        % Sample time
        Ts

        % Logging
        log
    end

    methods
        function obj = td3_adrc_controller(Ts, use_td3)
            %TD3_ADRC_CONTROLLER Construct the combined controller
            %   ctrl = td3_adrc_controller(Ts) with TD3 enabled
            %   ctrl = td3_adrc_controller(Ts, false) ADRC only
            %
            %   Inputs:
            %       Ts     - Sample time [s]
            %       use_td3- Enable TD3 adaptation (default: true)

            if nargin < 1
                Ts = 0.01;
            end
            if nargin < 2
                use_td3 = true;
            end

            obj.Ts = Ts;
            obj.use_td3 = use_td3;

            % Load parameters
            obj.params_adrc = adrc_params();
            obj.params_td3 = td3_params();
            obj.params_quad = quadrotor_params();

            % Initialize ADRC components
            obj.initialize_adrc();

            % Initialize TD3 agent if enabled
            if use_td3
                obj.initialize_td3();
            end

            % Control state
            obj.error_integral = zeros(3, 1);
            obj.last_error = zeros(3, 1);

            % Logging
            obj.log.time = [];
            obj.log.reference = [];
            obj.log.output = [];
            obj.log.error = [];
            obj.log.control = [];
            obj.log.disturbance = [];
            obj.log.td3_action = [];
        end

        function initialize_adrc(obj)
            %INITIALIZE_ADRC Create ADRC components for each axis

            Ts = obj.Ts;
            p = obj.params_adrc;

            % Tracking differentiators
            obj.td_roll = tracking_differentiator(p.td.r, Ts);
            obj.td_pitch = tracking_differentiator(p.td.r, Ts);
            obj.td_yaw = tracking_differentiator(p.td.r, Ts);

            % Extended state observers
            obj.eso_roll = extended_state_observer(p.roll.eso_omega, p.roll.b0, Ts);
            obj.eso_pitch = extended_state_observer(p.pitch.eso_omega, p.pitch.b0, Ts);
            obj.eso_yaw = extended_state_observer(p.yaw.eso_omega, p.yaw.b0, Ts);

            % Nonlinear feedback controllers
            obj.nlsef_roll = nonlinear_feedback(p.roll.nlsef_omega, p.roll.b0, Ts);
            obj.nlsef_pitch = nonlinear_feedback(p.pitch.nlsef_omega, p.pitch.b0, Ts);
            obj.nlsef_yaw = nonlinear_feedback(p.yaw.nlsef_omega, p.yaw.b0, Ts);

            % Enable novel tanh function
            obj.nlsef_roll.enable_tanh();
            obj.nlsef_pitch.enable_tanh();
            obj.nlsef_yaw.enable_tanh();
        end

        function initialize_td3(obj)
            %INITIALIZE_TD3 Create TD3 agent

            p = obj.params_td3;

            state_dim = p.state.dim;
            action_dim = p.action.dim;
            action_bounds = p.action.bounds;

            obj.td3_agent = td3_agent(state_dim, action_dim, action_bounds, p);
        end

        function [tau, info] = compute_control(obj, ref, y, t)
            %COMPUTE_CONTROL Compute attitude control torques
            %   [tau, info] = ctrl.compute_control(ref, y, t)
            %
            %   Inputs:
            %       ref - Reference attitude [phi_d; theta_d; psi_d] [rad]
            %       y   - Measured attitude [phi; theta; psi] [rad]
            %       t   - Current time [s]
            %
            %   Outputs:
            %       tau  - Control torques [tau_phi; tau_theta; tau_psi] [N*m]
            %       info - Structure with internal states

            Ts = obj.Ts;

            %% Update tracking differentiators
            [v1_r, v2_r] = obj.td_roll.update(ref(1), Ts);
            [v1_p, v2_p] = obj.td_pitch.update(ref(2), Ts);
            [v1_y, v2_y] = obj.td_yaw.update(ref(3), Ts);

            %% Update extended state observers
            % Note: control from previous step is needed
            if isempty(obj.log.control)
                u_prev = zeros(3, 1);
            else
                u_prev = obj.log.control(:, end);
            end

            [z1_r, z2_r, z3_r] = obj.eso_roll.update(y(1), u_prev(1), Ts);
            [z1_p, z2_p, z3_p] = obj.eso_pitch.update(y(2), u_prev(2), Ts);
            [z1_y, z2_y, z3_y] = obj.eso_yaw.update(y(3), u_prev(3), Ts);

            %% Compute errors
            e1 = [v1_r - z1_r; v1_p - z1_p; v1_y - z1_y];  % Position error
            e2 = [v2_r - z2_r; v2_p - z2_p; v2_y - z2_y];  % Velocity error
            z3 = [z3_r; z3_p; z3_y];  % Estimated disturbance

            % Update error integral (for TD3 state)
            obj.error_integral = obj.error_integral + e1 * Ts;
            error_derivative = (e1 - obj.last_error) / Ts;
            obj.last_error = e1;

            %% TD3 parameter adaptation
            td3_action = [15; 15; 8];  % Default bandwidths
            if obj.use_td3 && ~isempty(obj.td3_agent)
                % Build state for TD3
                % S_w = [e(t), e_i(t), e_dot(t), phi, theta, psi]'
                td3_state = [mean(abs(e1)); mean(abs(obj.error_integral)); ...
                             mean(abs(error_derivative)); y(1); y(2); y(3)];

                % Handle NaN/Inf in state
                td3_state(~isfinite(td3_state)) = 0;

                % Get action from TD3 (ADRC bandwidths)
                td3_action = obj.td3_agent.select_action(td3_state, false);

                % Validate action (protect against untrained network)
                if any(~isfinite(td3_action))
                    td3_action = [15; 15; 8];  % Use defaults
                end

                % Apply TD3 action to ADRC parameters
                obj.nlsef_roll.set_bandwidth(td3_action(1));
                obj.nlsef_pitch.set_bandwidth(td3_action(2));
                obj.nlsef_yaw.set_bandwidth(td3_action(3));
            end

            %% Compute control using NLSEF
            [tau_r, ~] = obj.nlsef_roll.compute(e1(1), e2(1), z3(1));
            [tau_p, ~] = obj.nlsef_pitch.compute(e1(2), e2(2), z3(2));
            [tau_y, ~] = obj.nlsef_yaw.compute(e1(3), e2(3), z3(3));

            tau = [tau_r; tau_p; tau_y];

            % Protect against NaN/Inf
            tau(~isfinite(tau)) = 0;

            % Saturate control output
            tau_max = [0.5; 0.5; 0.3];
            tau = max(-tau_max, min(tau_max, tau));

            %% Logging
            obj.log.time(end+1) = t;
            obj.log.reference(:, end+1) = ref;
            obj.log.output(:, end+1) = y;
            obj.log.error(:, end+1) = e1;
            obj.log.control(:, end+1) = tau;
            obj.log.disturbance(:, end+1) = z3;
            obj.log.td3_action(:, end+1) = td3_action;

            %% Output info
            info.e1 = e1;
            info.e2 = e2;
            info.z3 = z3;
            info.td3_action = td3_action;
            info.td_output = [v1_r, v1_p, v1_y; v2_r, v2_p, v2_y];
            info.eso_output = [z1_r, z1_p, z1_y; z2_r, z2_p, z2_y; z3_r, z3_p, z3_y];
        end

        function reward = compute_reward(obj, e1, e_int, e_dot, u)
            %COMPUTE_REWARD Compute TD3 reward signal
            %   reward = ctrl.compute_reward(e1, e_int, e_dot, u)
            %
            %   R_w = gamma1*|e| + gamma2*|e_i| + gamma3*|e_dot| + gamma4*X
            %   where X = 1 if |u| <= u_max, else 0

            p = obj.params_td3.reward;

            % Control bound check
            u_max = 1;  % Normalized
            X = double(all(abs(u) <= u_max));

            reward = p.gamma1 * norm(e1) + ...
                     p.gamma2 * norm(e_int) + ...
                     p.gamma3 * norm(e_dot) + ...
                     p.gamma4 * X + ...
                     p.alive_bonus;
        end

        function train_step(obj, state, action, reward, next_state, done)
            %TRAIN_STEP Store experience and train TD3
            %   ctrl.train_step(state, action, reward, next_state, done)

            if obj.use_td3 && ~isempty(obj.td3_agent)
                obj.td3_agent.store_transition(state, action, reward, next_state, done);

                if obj.td3_agent.buffer.is_ready_for_training(256)
                    obj.td3_agent.train_step(256);
                end
            end
        end

        function reset(obj)
            %RESET Reset controller state

            % Reset ADRC components
            obj.td_roll.reset();
            obj.td_pitch.reset();
            obj.td_yaw.reset();

            obj.eso_roll.reset();
            obj.eso_pitch.reset();
            obj.eso_yaw.reset();

            obj.nlsef_roll.reset();
            obj.nlsef_pitch.reset();
            obj.nlsef_yaw.reset();

            % Reset control state
            obj.error_integral = zeros(3, 1);
            obj.last_error = zeros(3, 1);

            % Clear logs
            obj.log.time = [];
            obj.log.reference = [];
            obj.log.output = [];
            obj.log.error = [];
            obj.log.control = [];
            obj.log.disturbance = [];
            obj.log.td3_action = [];
        end

        function set_adrc_only(obj)
            %SET_ADRC_ONLY Disable TD3, use fixed ADRC parameters
            obj.use_td3 = false;
        end

        function set_td3_adrc(obj)
            %SET_TD3_ADRC Enable TD3 parameter adaptation
            obj.use_td3 = true;
        end

        function metrics = compute_performance_metrics(obj)
            %COMPUTE_PERFORMANCE_METRICS Compute IAE, RMSE, etc.
            %   metrics = ctrl.compute_performance_metrics()

            if isempty(obj.log.error)
                metrics = struct();
                return;
            end

            e = obj.log.error;
            t = obj.log.time;
            dt = mean(diff(t));

            % IAE - Integral Absolute Error
            metrics.IAE_roll = sum(abs(e(1, :))) * dt;
            metrics.IAE_pitch = sum(abs(e(2, :))) * dt;
            metrics.IAE_yaw = sum(abs(e(3, :))) * dt;
            metrics.IAE_total = metrics.IAE_roll + metrics.IAE_pitch + metrics.IAE_yaw;

            % RMSE - Root Mean Square Error
            metrics.RMSE_roll = sqrt(mean(e(1, :).^2));
            metrics.RMSE_pitch = sqrt(mean(e(2, :).^2));
            metrics.RMSE_yaw = sqrt(mean(e(3, :).^2));

            % ISE - Integral Square Error
            metrics.ISE_roll = sum(e(1, :).^2) * dt;
            metrics.ISE_pitch = sum(e(2, :).^2) * dt;
            metrics.ISE_yaw = sum(e(3, :).^2) * dt;

            % Maximum error
            metrics.max_error_roll = max(abs(e(1, :)));
            metrics.max_error_pitch = max(abs(e(2, :)));
            metrics.max_error_yaw = max(abs(e(3, :)));

            % Control effort
            u = obj.log.control;
            metrics.control_effort_roll = sum(abs(u(1, :))) * dt;
            metrics.control_effort_pitch = sum(abs(u(2, :))) * dt;
            metrics.control_effort_yaw = sum(abs(u(3, :))) * dt;
        end

        function save_agent(obj, filepath)
            %SAVE_AGENT Save trained TD3 agent
            if obj.use_td3 && ~isempty(obj.td3_agent)
                obj.td3_agent.save(filepath);
            end
        end

        function load_agent(obj, filepath)
            %LOAD_AGENT Load trained TD3 agent
            if obj.use_td3 && ~isempty(obj.td3_agent)
                obj.td3_agent.load(filepath);
            end
        end
    end

end
