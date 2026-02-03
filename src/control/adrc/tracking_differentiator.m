classdef tracking_differentiator < handle
%TRACKING_DIFFERENTIATOR Nonlinear tracking differentiator for ADRC
%   Generates smooth reference signal and its derivative from noisy input
%
%   The TD uses a discrete-time fhan function to provide:
%   - v1: Smoothed tracking of reference signal
%   - v2: Derivative estimate of reference signal
%
%   Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"
%              Han (2009) "From PID to Active Disturbance Rejection Control"

    properties
        % Parameters
        r       % Tracking speed parameter (larger = faster tracking)
        h       % Filter parameter (sampling time)
        h0      % Numerical precision parameter

        % States
        v1      % Tracked signal
        v2      % Derivative of tracked signal

        % Configuration
        use_fhan    % Use fhan function (true) or linear TD (false)
    end

    methods
        function obj = tracking_differentiator(r, h, v1_init, v2_init)
            %TRACKING_DIFFERENTIATOR Construct tracking differentiator
            %   td = tracking_differentiator(r, h) creates TD with default init
            %   td = tracking_differentiator(r, h, v1_init, v2_init) with init values
            %
            %   Inputs:
            %       r       - Tracking speed parameter (typical: 50-500)
            %       h       - Filter parameter/sampling time
            %       v1_init - Initial tracked signal (optional)
            %       v2_init - Initial derivative (optional)

            if nargin < 1 || isempty(r)
                r = 100;
            end
            if nargin < 2 || isempty(h)
                h = 0.01;
            end
            if nargin < 3 || isempty(v1_init)
                v1_init = 0;
            end
            if nargin < 4 || isempty(v2_init)
                v2_init = 0;
            end

            obj.r = r;
            obj.h = h;
            obj.h0 = h;  % Can be different for numerical precision
            obj.v1 = v1_init;
            obj.v2 = v2_init;
            obj.use_fhan = true;
        end

        function [v1_out, v2_out] = update(obj, v, h)
            %UPDATE Update tracking differentiator with new reference
            %   [v1, v2] = td.update(v) updates with reference v
            %   [v1, v2] = td.update(v, h) updates with custom time step
            %
            %   Inputs:
            %       v - Reference signal to track
            %       h - Time step (optional, uses obj.h if not provided)
            %
            %   Outputs:
            %       v1_out - Smoothed reference signal
            %       v2_out - Derivative estimate

            if nargin < 3 || isempty(h)
                h = obj.h;
            end

            if obj.use_fhan
                % Nonlinear TD using fhan function
                fh = obj.fhan(obj.v1 - v, obj.v2, obj.r, obj.h0);
                obj.v1 = obj.v1 + h * obj.v2;
                obj.v2 = obj.v2 + h * fh;
            else
                % Linear TD (critically damped second-order)
                omega_n = sqrt(obj.r);
                zeta = 1.0;  % Critical damping
                k1 = 2 * zeta * omega_n;
                k2 = omega_n^2;

                v1_dot = obj.v2;
                v2_dot = -k1 * obj.v2 - k2 * (obj.v1 - v);

                obj.v1 = obj.v1 + h * v1_dot;
                obj.v2 = obj.v2 + h * v2_dot;
            end

            v1_out = obj.v1;
            v2_out = obj.v2;
        end

        function reset(obj, v1_init, v2_init)
            %RESET Reset tracking differentiator states
            %   td.reset() resets to zero
            %   td.reset(v1, v2) resets to specified values

            if nargin < 2
                v1_init = 0;
            end
            if nargin < 3
                v2_init = 0;
            end
            obj.v1 = v1_init;
            obj.v2 = v2_init;
        end

        function set_params(obj, r, h)
            %SET_PARAMS Update TD parameters
            %   td.set_params(r, h) updates tracking speed and filter time

            if nargin >= 2 && ~isempty(r)
                obj.r = r;
            end
            if nargin >= 3 && ~isempty(h)
                obj.h = h;
                obj.h0 = h;
            end
        end
    end

    methods (Static)
        function fh = fhan(x1, x2, r, h)
            %FHAN Optimal control function for tracking differentiator
            %   fh = fhan(x1, x2, r, h) computes the optimal control
            %
            %   This is the discrete-time fastest tracking function that
            %   drives the system to origin optimally.
            %
            %   Inputs:
            %       x1 - Position error (v1 - v)
            %       x2 - Velocity (v2)
            %       r  - Tracking speed parameter
            %       h  - Filter parameter

            d = r * h;
            d0 = h * d;
            y = x1 + h * x2;
            a0 = sqrt(d^2 + 8 * r * abs(y));

            if abs(y) <= d0
                a = x2 + y / h;
            else
                a = x2 + 0.5 * (a0 - d) * sign(y);
            end

            if abs(a) <= d
                fh = -r * a / d;
            else
                fh = -r * sign(a);
            end
        end

        function [v1, v2] = filter_signal(v_signal, r, h, v1_init, v2_init)
            %FILTER_SIGNAL Batch filter a signal using TD
            %   [v1, v2] = filter_signal(v, r, h) filters entire signal
            %
            %   Inputs:
            %       v_signal - Input signal vector
            %       r        - Tracking speed parameter
            %       h        - Sampling time
            %       v1_init  - Initial tracked value (optional)
            %       v2_init  - Initial derivative (optional)
            %
            %   Outputs:
            %       v1 - Filtered signal vector
            %       v2 - Derivative estimate vector

            if nargin < 4 || isempty(v1_init)
                v1_init = v_signal(1);
            end
            if nargin < 5 || isempty(v2_init)
                v2_init = 0;
            end

            N = length(v_signal);
            v1 = zeros(N, 1);
            v2 = zeros(N, 1);

            td = tracking_differentiator(r, h, v1_init, v2_init);

            for i = 1:N
                [v1(i), v2(i)] = td.update(v_signal(i), h);
            end
        end
    end

end
