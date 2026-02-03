classdef disturbance_observer < handle
%DISTURBANCE_OBSERVER External disturbance observer for quadrotors
%   Estimates wind and other external disturbances using measurements
%
%   Uses momentum-based approach combined with low-pass filtering
%
%   Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"

    properties
        % Estimated disturbances
        F_dist          % Force disturbance [Fx; Fy; Fz] [N]
        tau_dist        % Torque disturbance [tau_x; tau_y; tau_z] [N*m]

        % Filter states
        vel_filtered    % Filtered velocity
        omega_filtered  % Filtered angular velocity

        % Parameters
        params          % Quadrotor parameters
        tau_f           % Filter time constant
        Ts              % Sample time

        % Previous values for differentiation
        vel_prev
        omega_prev
    end

    methods
        function obj = disturbance_observer(params, tau_f, Ts)
            %DISTURBANCE_OBSERVER Construct disturbance observer
            %   obs = disturbance_observer(params, tau_f, Ts)
            %
            %   Inputs:
            %       params - Quadrotor parameters
            %       tau_f  - Filter time constant [s]
            %       Ts     - Sample time [s]

            obj.params = params;
            obj.tau_f = tau_f;
            obj.Ts = Ts;

            obj.F_dist = zeros(3, 1);
            obj.tau_dist = zeros(3, 1);

            obj.vel_filtered = zeros(3, 1);
            obj.omega_filtered = zeros(3, 1);

            obj.vel_prev = zeros(3, 1);
            obj.omega_prev = zeros(3, 1);
        end

        function [F_est, tau_est] = update(obj, vel, omega, euler, u, Ts)
            %UPDATE Update disturbance estimates
            %   [F_est, tau_est] = obs.update(vel, omega, euler, u, Ts)
            %
            %   Inputs:
            %       vel   - Linear velocity [vx; vy; vz] [m/s]
            %       omega - Angular velocity [p; q; r] [rad/s]
            %       euler - Euler angles [phi; theta; psi] [rad]
            %       u     - Control input [F; tau_x; tau_y; tau_z]
            %       Ts    - Time step (optional)
            %
            %   Outputs:
            %       F_est   - Estimated force disturbance [N]
            %       tau_est - Estimated torque disturbance [N*m]

            if nargin < 6
                Ts = obj.Ts;
            end

            p = obj.params;

            %% Low-pass filter the measurements
            alpha = Ts / (obj.tau_f + Ts);

            obj.vel_filtered = alpha * vel + (1 - alpha) * obj.vel_filtered;
            obj.omega_filtered = alpha * omega + (1 - alpha) * obj.omega_filtered;

            %% Numerical differentiation with filtering
            vel_dot = (obj.vel_filtered - obj.vel_prev) / Ts;
            omega_dot = (obj.omega_filtered - obj.omega_prev) / Ts;

            obj.vel_prev = obj.vel_filtered;
            obj.omega_prev = obj.omega_filtered;

            %% Rotation matrix
            phi = euler(1);
            theta = euler(2);
            psi = euler(3);

            cphi = cos(phi);   sphi = sin(phi);
            cth = cos(theta);  sth = sin(theta);
            cpsi = cos(psi);   spsi = sin(psi);

            R_bi = [
                cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
                cth*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi;
                -sth,      sphi*cth,                    cphi*cth
            ];

            %% Extract control inputs
            F = u(1);          % Total thrust
            tau = u(2:4);      % Body torques

            %% Estimate force disturbance (translational)
            % m * a = -mg*e3 + R*F*e3 + F_drag + F_dist
            % F_dist = m * a + mg*e3 - R*F*e3 - F_drag

            gravity = [0; 0; -p.g];
            thrust_inertial = R_bi * [0; 0; F];

            % Drag model
            F_drag = -[p.kdx; p.kdy; p.kdz] .* vel;

            F_est = p.m * vel_dot - gravity * p.m - thrust_inertial - F_drag;

            %% Estimate torque disturbance (rotational)
            % I * omega_dot = tau - omega x (I*omega) + tau_friction + tau_dist
            % tau_dist = I * omega_dot - tau + omega x (I*omega) - tau_friction

            I = diag([p.Ixx, p.Iyy, p.Izz]);
            gyro = cross(omega, I * omega);
            tau_friction = -[p.kfp; p.kfq; p.kfr] .* omega;

            tau_est = I * omega_dot - tau + gyro - tau_friction;

            %% Low-pass filter the estimates
            obj.F_dist = alpha * F_est + (1 - alpha) * obj.F_dist;
            obj.tau_dist = alpha * tau_est + (1 - alpha) * obj.tau_dist;

            F_est = obj.F_dist;
            tau_est = obj.tau_dist;
        end

        function reset(obj)
            %RESET Reset observer state
            obj.F_dist = zeros(3, 1);
            obj.tau_dist = zeros(3, 1);
            obj.vel_filtered = zeros(3, 1);
            obj.omega_filtered = zeros(3, 1);
            obj.vel_prev = zeros(3, 1);
            obj.omega_prev = zeros(3, 1);
        end

        function [F, tau] = get_estimates(obj)
            %GET_ESTIMATES Return current disturbance estimates
            F = obj.F_dist;
            tau = obj.tau_dist;
        end
    end

end


function D = generate_wind_disturbance(t, type, params)
%GENERATE_WIND_DISTURBANCE Generate wind disturbance profiles for simulation
%   D = generate_wind_disturbance(t, type, params)
%
%   Inputs:
%       t      - Time vector or scalar [s]
%       type   - Disturbance type: 'constant', 'sinusoidal', 'gust', 'turbulence'
%       params - Structure with disturbance parameters
%
%   Outputs:
%       D      - Disturbance vector [Fx; Fy; Fz; tau_x; tau_y; tau_z]

    if nargin < 2
        type = 'sinusoidal';
    end
    if nargin < 3
        params = struct();
    end

    % Default parameters
    if ~isfield(params, 'amplitude')
        params.amplitude = [0.3; 0.2; 0.01; 0.01; 0.01; 0.01];
    end
    if ~isfield(params, 'frequency')
        params.frequency = [1/15; 1/10; 1/10; 0.5; 0.5; 0.3];  % Hz
    end

    N = length(t);
    D = zeros(6, N);

    switch lower(type)
        case 'constant'
            % Constant disturbance
            D = repmat(params.amplitude(:), 1, N);

        case 'sinusoidal'
            % Sinusoidal disturbance (from paper)
            for i = 1:6
                D(i, :) = params.amplitude(i) * sin(2*pi*params.frequency(i)*t);
            end

        case 'gust'
            % Wind gust model
            if ~isfield(params, 'gust_time')
                params.gust_time = 5;  % Start time
            end
            if ~isfield(params, 'gust_duration')
                params.gust_duration = 2;  % Duration
            end

            for i = 1:N
                if t(i) >= params.gust_time && t(i) <= params.gust_time + params.gust_duration
                    % 1-cosine gust shape
                    tau_gust = (t(i) - params.gust_time) / params.gust_duration;
                    gust_shape = 0.5 * (1 - cos(2*pi*tau_gust));
                    D(:, i) = params.amplitude(:) * gust_shape;
                end
            end

        case 'turbulence'
            % Dryden turbulence model (simplified)
            if ~isfield(params, 'intensity')
                params.intensity = 0.5;
            end

            % Generate colored noise
            for i = 1:6
                white_noise = randn(1, N);
                % Simple low-pass filter for colored noise
                tau = 0.5;
                dt = t(2) - t(1);
                alpha = dt / (tau + dt);

                filtered = zeros(1, N);
                for j = 2:N
                    filtered(j) = alpha * white_noise(j) + (1-alpha) * filtered(j-1);
                end

                D(i, :) = params.amplitude(i) * params.intensity * filtered;
            end

        otherwise
            error('Unknown disturbance type: %s', type);
    end

end
