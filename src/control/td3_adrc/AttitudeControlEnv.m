classdef AttitudeControlEnv < handle
%ATTITUDECONTROLENV Training environment for TD3-ADRC
%   Simulates quadrotor attitude control for reinforcement learning
%
%   Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"

    properties
        state           % Current state [phi, theta, psi, p, q, r]
        t               % Current time
        Ts              % Sample time
        params          % Quadrotor parameters
        ref             % Reference attitude
        max_time        % Maximum episode time
        disturbance_amp % Disturbance amplitude

        % ADRC components
        td_r, td_p, td_y
        eso_r, eso_p, eso_y
        nlsef_r, nlsef_p, nlsef_y
        last_u
    end

    methods
        function obj = AttitudeControlEnv()
            obj.Ts = 0.01;
            obj.params = quadrotor_params();
            obj.max_time = 5.0;  % 5 second episodes
            obj.disturbance_amp = [0.02; 0.02; 0.01];
            obj.reset();
        end

        function obs = reset(obj)
            obj.t = 0;

            % Random initial attitude (small perturbation)
            obj.state = [
                0.05 * randn(3, 1);  % Small random attitude [rad]
                zeros(3, 1)          % Zero angular rates
            ];

            % Random reference attitude
            obj.ref = deg2rad([
                15 * (2*rand - 1);   % Roll: -15 to 15 deg
                15 * (2*rand - 1);   % Pitch: -15 to 15 deg
                20 * (2*rand - 1);   % Yaw: -20 to 20 deg
            ]);

            % Create fresh ADRC components for this episode
            obj.init_adrc();

            obs = obj.get_observation();
        end

        function init_adrc(obj)
            p = adrc_params();
            Ts = obj.Ts;

            obj.td_r = tracking_differentiator(p.td.r, Ts);
            obj.td_p = tracking_differentiator(p.td.r, Ts);
            obj.td_y = tracking_differentiator(p.td.r, Ts);

            obj.eso_r = extended_state_observer(p.roll.eso_omega, p.roll.b0, Ts);
            obj.eso_p = extended_state_observer(p.pitch.eso_omega, p.pitch.b0, Ts);
            obj.eso_y = extended_state_observer(p.yaw.eso_omega, p.yaw.b0, Ts);

            obj.nlsef_r = nonlinear_feedback(p.roll.nlsef_omega, p.roll.b0, Ts);
            obj.nlsef_p = nonlinear_feedback(p.pitch.nlsef_omega, p.pitch.b0, Ts);
            obj.nlsef_y = nonlinear_feedback(p.yaw.nlsef_omega, p.yaw.b0, Ts);

            obj.last_u = zeros(3, 1);
        end

        function [obs, reward, done] = step(obj, action)
            % Action = [wc_roll, wc_pitch, wc_yaw] - controller bandwidths

            euler = obj.state(1:3);
            omega = obj.state(4:6);

            % Apply action to ADRC (update bandwidths)
            obj.nlsef_r.set_bandwidth(action(1));
            obj.nlsef_p.set_bandwidth(action(2));
            obj.nlsef_y.set_bandwidth(action(3));

            % Compute ADRC control
            tau = obj.compute_control(euler);

            % Add disturbance
            tau_dist = obj.disturbance_amp .* sin(2*pi*0.5*obj.t + rand(3,1)*pi);

            % Simulate dynamics
            [euler_new, omega_new] = obj.simulate_step(euler, omega, tau, tau_dist);

            obj.state = [euler_new; omega_new];
            obj.t = obj.t + obj.Ts;

            % Compute reward
            e = obj.ref - euler_new;
            e_dot = -omega_new;

            % Reward: penalize error, reward smooth control
            reward = -5 * norm(e) - 0.5 * norm(e_dot) - 0.01 * norm(tau) + 0.5;

            % Bonus for small error
            if norm(e) < 0.05
                reward = reward + 1.0;
            end

            % Check termination
            done = false;
            if obj.t >= obj.max_time
                done = true;
            end
            if max(abs(euler_new)) > deg2rad(60)
                done = true;
                reward = reward - 10;  % Penalty for instability
            end

            obs = obj.get_observation();
        end

        function tau = compute_control(obj, euler)
            Ts = obj.Ts;

            % Update tracking differentiators
            [v1_r, v2_r] = obj.td_r.update(obj.ref(1), Ts);
            [v1_p, v2_p] = obj.td_p.update(obj.ref(2), Ts);
            [v1_y, v2_y] = obj.td_y.update(obj.ref(3), Ts);

            % Update ESOs
            [z1_r, z2_r, z3_r] = obj.eso_r.update(euler(1), obj.last_u(1), Ts);
            [z1_p, z2_p, z3_p] = obj.eso_p.update(euler(2), obj.last_u(2), Ts);
            [z1_y, z2_y, z3_y] = obj.eso_y.update(euler(3), obj.last_u(3), Ts);

            % Compute errors
            e1 = [v1_r - z1_r; v1_p - z1_p; v1_y - z1_y];
            e2 = [v2_r - z2_r; v2_p - z2_p; v2_y - z2_y];
            z3 = [z3_r; z3_p; z3_y];

            % Compute control
            [tau_r, ~] = obj.nlsef_r.compute(e1(1), e2(1), z3(1));
            [tau_p, ~] = obj.nlsef_p.compute(e1(2), e2(2), z3(2));
            [tau_y, ~] = obj.nlsef_y.compute(e1(3), e2(3), z3(3));

            tau = [tau_r; tau_p; tau_y];

            % Saturate
            tau_max = [0.5; 0.5; 0.3];
            tau = max(-tau_max, min(tau_max, tau));

            obj.last_u = tau;
        end

        function [euler_new, omega_new] = simulate_step(obj, euler, omega, tau, tau_dist)
            params = obj.params;
            Ts = obj.Ts;

            I = diag([params.Ixx, params.Iyy, params.Izz]);
            kf = [params.kfp; params.kfq; params.kfr];

            % Dynamics
            gyro = cross(omega, I * omega);
            friction = -kf .* omega;
            omega_dot = I \ (tau + friction - gyro + tau_dist);

            % Kinematics
            phi = euler(1); theta = euler(2);
            sphi = sin(phi); cphi = cos(phi);
            cth = cos(theta); tth = tan(theta);

            if abs(cth) < 1e-6
                cth = sign(cth) * 1e-6;
                tth = sin(theta) / cth;
            end

            euler_dot = [
                omega(1) + sphi * tth * omega(2) + cphi * tth * omega(3);
                cphi * omega(2) - sphi * omega(3);
                (sphi/cth) * omega(2) + (cphi/cth) * omega(3)
            ];

            % Integrate
            euler_new = euler + Ts * euler_dot;
            omega_new = omega + Ts * omega_dot;
        end

        function obs = get_observation(obj)
            euler = obj.state(1:3);
            omega = obj.state(4:6);
            e = obj.ref - euler;

            % Observation: [mean_error, integral_proxy, error_derivative, euler]
            obs = [
                mean(abs(e));
                0;  % Simplified
                mean(abs(omega));
                euler
            ];
        end
    end

end
