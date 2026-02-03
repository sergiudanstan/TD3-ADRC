classdef extended_state_observer < handle
%EXTENDED_STATE_OBSERVER Extended State Observer for ADRC
%   Estimates system states and total disturbance as an extended state
%
%   For a second-order system with disturbance:
%       x1_dot = x2
%       x2_dot = f(x1, x2, t) + b*u + d(t)
%            y = x1
%
%   ESO estimates [x1, x2, x3]' where x3 = f + d (total disturbance)
%
%   Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"
%              Gao (2006) "Scaling and Bandwidth-Parameterization Based
%              Controller Tuning"

    properties
        % Order of the observer
        order       % 2 for standard ESO, 3 for extended state

        % Observer gains
        beta1       % Gain for output error
        beta2       % Gain for velocity estimate error
        beta3       % Gain for disturbance estimate error (3rd order)

        % Control gain estimate
        b0          % Estimated control gain

        % State estimates
        z1          % Output estimate (position/angle)
        z2          % Velocity estimate
        z3          % Total disturbance estimate (extended state)

        % Observer bandwidth
        omega_o     % Observer bandwidth [rad/s]

        % Nonlinear ESO parameters
        use_nonlinear   % Use nonlinear observer (fal function)
        alpha1      % Nonlinear gain for z1
        alpha2      % Nonlinear gain for z2
        delta       % Linear zone width

        % Sample time
        Ts
    end

    methods
        function obj = extended_state_observer(omega_o, b0, Ts, order)
            %EXTENDED_STATE_OBSERVER Construct ESO
            %   eso = extended_state_observer(omega_o, b0, Ts) creates 3rd order ESO
            %   eso = extended_state_observer(omega_o, b0, Ts, order) with specified order
            %
            %   Inputs:
            %       omega_o - Observer bandwidth [rad/s]
            %       b0      - Estimated control gain
            %       Ts      - Sample time [s]
            %       order   - Observer order (2 or 3, default: 3)

            if nargin < 4 || isempty(order)
                order = 3;
            end

            obj.order = order;
            obj.omega_o = omega_o;
            obj.b0 = b0;
            obj.Ts = Ts;

            % Bandwidth-parameterized gains (pole placement)
            if order == 2
                % Place both poles at -omega_o
                obj.beta1 = 2 * omega_o;
                obj.beta2 = omega_o^2;
                obj.beta3 = 0;
            else
                % Place all three poles at -omega_o
                obj.beta1 = 3 * omega_o;
                obj.beta2 = 3 * omega_o^2;
                obj.beta3 = omega_o^3;
            end

            % Initialize states
            obj.z1 = 0;
            obj.z2 = 0;
            obj.z3 = 0;

            % Nonlinear ESO settings
            obj.use_nonlinear = false;
            obj.alpha1 = 0.5;
            obj.alpha2 = 0.25;
            obj.delta = 0.01;
        end

        function [z1, z2, z3] = update(obj, y, u, Ts)
            %UPDATE Update observer with new measurement
            %   [z1, z2, z3] = eso.update(y, u) updates with measurement y and control u
            %   [z1, z2, z3] = eso.update(y, u, Ts) with custom time step
            %
            %   Inputs:
            %       y  - Measured output (position/angle)
            %       u  - Control input
            %       Ts - Time step (optional)
            %
            %   Outputs:
            %       z1 - Output estimate
            %       z2 - Velocity estimate
            %       z3 - Total disturbance estimate

            if nargin < 4 || isempty(Ts)
                Ts = obj.Ts;
            end

            % Output estimation error
            e = obj.z1 - y;

            if obj.use_nonlinear
                % Nonlinear ESO with fal function
                g1 = obj.fal(e, obj.alpha1, obj.delta);
                g2 = obj.fal(e, obj.alpha2, obj.delta);

                if obj.order == 3
                    g3 = obj.fal(e, obj.alpha2/2, obj.delta);
                end
            else
                % Linear ESO
                g1 = e;
                g2 = e;
                g3 = e;
            end

            if obj.order == 2
                % Second-order ESO
                z1_dot = obj.z2 - obj.beta1 * g1;
                z2_dot = obj.b0 * u - obj.beta2 * g2;

                obj.z1 = obj.z1 + Ts * z1_dot;
                obj.z2 = obj.z2 + Ts * z2_dot;
                obj.z3 = 0;
            else
                % Third-order ESO (extended state for disturbance)
                z1_dot = obj.z2 - obj.beta1 * g1;
                z2_dot = obj.z3 + obj.b0 * u - obj.beta2 * g2;
                z3_dot = -obj.beta3 * g3;

                obj.z1 = obj.z1 + Ts * z1_dot;
                obj.z2 = obj.z2 + Ts * z2_dot;
                obj.z3 = obj.z3 + Ts * z3_dot;
            end

            z1 = obj.z1;
            z2 = obj.z2;
            z3 = obj.z3;
        end

        function reset(obj, z1, z2, z3)
            %RESET Reset observer states
            %   eso.reset() resets to zero
            %   eso.reset(z1, z2, z3) resets to specified values

            if nargin < 2, z1 = 0; end
            if nargin < 3, z2 = 0; end
            if nargin < 4, z3 = 0; end

            obj.z1 = z1;
            obj.z2 = z2;
            obj.z3 = z3;
        end

        function set_bandwidth(obj, omega_o)
            %SET_BANDWIDTH Update observer bandwidth
            %   eso.set_bandwidth(omega_o) updates gains based on new bandwidth

            obj.omega_o = omega_o;

            if obj.order == 2
                obj.beta1 = 2 * omega_o;
                obj.beta2 = omega_o^2;
            else
                obj.beta1 = 3 * omega_o;
                obj.beta2 = 3 * omega_o^2;
                obj.beta3 = omega_o^3;
            end
        end

        function set_gains(obj, beta1, beta2, beta3)
            %SET_GAINS Directly set observer gains
            %   eso.set_gains(beta1, beta2) for 2nd order
            %   eso.set_gains(beta1, beta2, beta3) for 3rd order

            obj.beta1 = beta1;
            obj.beta2 = beta2;
            if nargin >= 4
                obj.beta3 = beta3;
            end
        end

        function set_b0(obj, b0)
            %SET_B0 Update control gain estimate
            obj.b0 = b0;
        end

        function enable_nonlinear(obj, alpha1, alpha2, delta)
            %ENABLE_NONLINEAR Enable nonlinear ESO
            %   eso.enable_nonlinear() uses default parameters
            %   eso.enable_nonlinear(alpha1, alpha2, delta) with custom params

            obj.use_nonlinear = true;

            if nargin >= 2, obj.alpha1 = alpha1; end
            if nargin >= 3, obj.alpha2 = alpha2; end
            if nargin >= 4, obj.delta = delta; end
        end

        function disable_nonlinear(obj)
            %DISABLE_NONLINEAR Use linear ESO
            obj.use_nonlinear = false;
        end

        function [z1, z2, z3] = get_states(obj)
            %GET_STATES Return current state estimates
            z1 = obj.z1;
            z2 = obj.z2;
            z3 = obj.z3;
        end
    end

    methods (Static)
        function y = fal(e, alpha, delta)
            %FAL Nonlinear function for ESO
            %   y = fal(e, alpha, delta)
            %
            %   fal(e, alpha, delta) = e / delta^(1-alpha),  |e| <= delta
            %                        = |e|^alpha * sign(e),   |e| > delta
            %
            %   Provides high gain for small errors, low gain for large errors

            if abs(e) <= delta
                y = e / delta^(1 - alpha);
            else
                y = abs(e)^alpha * sign(e);
            end
        end
    end

end
