classdef nonlinear_feedback < handle
%NONLINEAR_FEEDBACK Nonlinear State Error Feedback for ADRC
%   Generates control signal from tracking error and observer states
%
%   Combines TD output (reference) with ESO estimates (states) to generate
%   optimal control that cancels estimated disturbance.
%
%   Control law: u0 = kp*fal(e1, alpha1, delta) + kd*fal(e2, alpha2, delta)
%                u = (u0 - z3) / b0
%
%   Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"

    properties
        % Controller gains
        kp          % Proportional gain
        kd          % Derivative gain
        ki          % Integral gain (optional)

        % Controller bandwidth
        omega_c     % Controller bandwidth [rad/s]

        % Nonlinear function parameters
        alpha1      % Exponent for position error
        alpha2      % Exponent for velocity error
        delta       % Linear zone threshold

        % Control gain
        b0          % Control gain estimate

        % Integral state
        integral_error

        % Use novel tanh function
        use_tanh
        tanh_params % Parameters for tanh function

        % Anti-windup
        integral_limit
        control_limit

        % Sample time
        Ts
    end

    methods
        function obj = nonlinear_feedback(omega_c, b0, Ts)
            %NONLINEAR_FEEDBACK Construct NLSEF controller
            %   nlsef = nonlinear_feedback(omega_c, b0, Ts)
            %
            %   Inputs:
            %       omega_c - Controller bandwidth [rad/s]
            %       b0      - Control gain estimate
            %       Ts      - Sample time [s]

            obj.omega_c = omega_c;
            obj.b0 = b0;
            obj.Ts = Ts;

            % Bandwidth-based gains (critically damped response)
            obj.kp = omega_c^2;
            obj.kd = 2 * omega_c;
            obj.ki = 0;  % Typically not needed with ESO

            % Default nonlinear parameters
            obj.alpha1 = 0.75;
            obj.alpha2 = 1.25;
            obj.delta = 0.1;

            % Integral state
            obj.integral_error = 0;

            % Use novel tanh function (from paper)
            obj.use_tanh = true;
            obj.tanh_params.alpha = 1.0;
            obj.tanh_params.delta = 0.5;

            % Limits
            obj.integral_limit = 10;
            obj.control_limit = 100;
        end

        function [u, u0] = compute(obj, e1, e2, z3, use_disturbance_comp)
            %COMPUTE Compute control signal
            %   [u, u0] = nlsef.compute(e1, e2, z3) computes control with disturbance comp
            %   [u, u0] = nlsef.compute(e1, e2, z3, false) without disturbance comp
            %
            %   Inputs:
            %       e1 - Position/angle error (v1 - z1)
            %       e2 - Velocity error (v2 - z2)
            %       z3 - Estimated total disturbance from ESO
            %       use_disturbance_comp - Enable disturbance compensation (default: true)
            %
            %   Outputs:
            %       u  - Final control signal (after disturbance compensation)
            %       u0 - Raw control signal (before compensation)

            if nargin < 5
                use_disturbance_comp = true;
            end

            % Update integral (if enabled)
            if obj.ki > 0
                obj.integral_error = obj.integral_error + e1 * obj.Ts;
                obj.integral_error = max(-obj.integral_limit, ...
                    min(obj.integral_limit, obj.integral_error));
            end

            % Compute nonlinear feedback
            if obj.use_tanh
                % Use novel parameterized tanh function
                f1 = tanh_function(e1, obj.tanh_params.alpha, obj.tanh_params.delta);
                f2 = tanh_function(e2, obj.tanh_params.alpha, obj.tanh_params.delta);
            else
                % Use traditional fal function
                f1 = obj.fal(e1, obj.alpha1, obj.delta);
                f2 = obj.fal(e2, obj.alpha2, obj.delta);
            end

            % Control law
            u0 = obj.kp * f1 + obj.kd * f2 + obj.ki * obj.integral_error;

            % Disturbance compensation and control gain
            if use_disturbance_comp
                u = (u0 - z3) / obj.b0;
            else
                u = u0 / obj.b0;
            end

            % Apply control limits
            u = max(-obj.control_limit, min(obj.control_limit, u));
        end

        function [u, u0] = compute_pd(obj, e1, e2)
            %COMPUTE_PD Simple PD control (no ESO disturbance)
            %   [u, u0] = nlsef.compute_pd(e1, e2)

            if obj.use_tanh
                f1 = tanh_function(e1, obj.tanh_params.alpha, obj.tanh_params.delta);
                f2 = tanh_function(e2, obj.tanh_params.alpha, obj.tanh_params.delta);
            else
                f1 = obj.fal(e1, obj.alpha1, obj.delta);
                f2 = obj.fal(e2, obj.alpha2, obj.delta);
            end

            u0 = obj.kp * f1 + obj.kd * f2;
            u = u0 / obj.b0;

            u = max(-obj.control_limit, min(obj.control_limit, u));
        end

        function reset(obj)
            %RESET Reset controller state
            obj.integral_error = 0;
        end

        function set_bandwidth(obj, omega_c)
            %SET_BANDWIDTH Update controller bandwidth
            obj.omega_c = omega_c;
            obj.kp = omega_c^2;
            obj.kd = 2 * omega_c;
        end

        function set_gains(obj, kp, kd, ki)
            %SET_GAINS Directly set controller gains
            obj.kp = kp;
            obj.kd = kd;
            if nargin >= 4
                obj.ki = ki;
            end
        end

        function set_tanh_params(obj, alpha, delta)
            %SET_TANH_PARAMS Update tanh function parameters
            %   nlsef.set_tanh_params(alpha, delta)
            %
            %   These are the parameters that TD3 will tune!

            obj.tanh_params.alpha = alpha;
            obj.tanh_params.delta = delta;
        end

        function enable_tanh(obj)
            %ENABLE_TANH Use novel tanh function
            obj.use_tanh = true;
        end

        function disable_tanh(obj)
            %DISABLE_TANH Use traditional fal function
            obj.use_tanh = false;
        end

        function set_b0(obj, b0)
            %SET_B0 Update control gain estimate
            obj.b0 = b0;
        end
    end

    methods (Static)
        function y = fal(e, alpha, delta)
            %FAL Traditional nonlinear function
            %   y = fal(e, alpha, delta)

            if abs(e) <= delta
                y = e / delta^(1 - alpha);
            else
                y = abs(e)^alpha * sign(e);
            end
        end
    end

end
