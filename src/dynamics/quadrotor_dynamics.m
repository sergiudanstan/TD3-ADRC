function [x_dot, outputs] = quadrotor_dynamics(t, x, u, params, disturbance)
%QUADROTOR_DYNAMICS Nonlinear quadrotor dynamics model
%   12-state nonlinear model for X-configuration quadrotor UAV
%
%   State vector x = [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]'
%       - Position: x, y, z [m] in inertial frame
%       - Velocity: vx, vy, vz [m/s] in inertial frame
%       - Euler angles: phi (roll), theta (pitch), psi (yaw) [rad]
%       - Angular rates: p, q, r [rad/s] in body frame
%
%   Input vector u = [F, tau_x, tau_y, tau_z]' or [omega1, omega2, omega3, omega4]'
%       - F: total thrust [N]
%       - tau_x, tau_y, tau_z: body torques [N*m]
%
%   Inputs:
%       t          - Current time [s]
%       x          - State vector (12 x 1)
%       u          - Control input (4 x 1)
%       params     - Quadrotor parameters from quadrotor_params()
%       disturbance- External disturbance forces/torques (6 x 1, optional)
%
%   Outputs:
%       x_dot      - State derivative (12 x 1)
%       outputs    - Structure with additional outputs
%
%   Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"

    if nargin < 5
        disturbance = zeros(6, 1);
    end

    %% Extract states
    % Position (inertial frame)
    pos = x(1:3);

    % Velocity (inertial frame)
    vel = x(4:6);
    vx = vel(1);
    vy = vel(2);
    vz = vel(3);

    % Euler angles
    phi = x(7);     % Roll
    theta = x(8);   % Pitch
    psi = x(9);     % Yaw

    % Angular rates (body frame)
    omega_b = x(10:12);
    p = omega_b(1);
    q = omega_b(2);
    r = omega_b(3);

    %% Extract parameters
    m = params.m;
    g = params.g;
    Ixx = params.Ixx;
    Iyy = params.Iyy;
    Izz = params.Izz;

    % Drag coefficients
    kdx = params.kdx;
    kdy = params.kdy;
    kdz = params.kdz;

    % Rotational friction
    kfp = params.kfp;
    kfq = params.kfq;
    kfr = params.kfr;

    %% Process control input
    % Input can be motor speeds or direct force/torques
    if length(u) == 4
        if max(abs(u)) > 100
            % Input is motor speeds [rad/s]
            omega_motors = u;
            omega_sq = omega_motors.^2;
            F = params.kf * sum(omega_sq);
            tau_x = params.B(2,:) * omega_sq / params.kf * params.kf;
            tau_y = params.B(3,:) * omega_sq / params.kf * params.kf;
            tau_z = params.B(4,:) * omega_sq / params.kf * params.kf;

            % Recalculate using mixer
            forces = params.B * omega_sq;
            F = forces(1);
            tau_x = forces(2);
            tau_y = forces(3);
            tau_z = forces(4);
        else
            % Input is [F, tau_x, tau_y, tau_z]
            F = u(1);
            tau_x = u(2);
            tau_y = u(3);
            tau_z = u(4);
        end
    else
        error('Control input must be 4 x 1 vector');
    end

    %% Rotation matrix (body to inertial)
    % ZYX Euler angle convention
    cphi = cos(phi);   sphi = sin(phi);
    cth = cos(theta);  sth = sin(theta);
    cpsi = cos(psi);   spsi = sin(psi);

    R_bi = [
        cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
        cth*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi;
        -sth,      sphi*cth,                    cphi*cth
    ];

    %% Translational dynamics
    % Gravity vector (inertial frame)
    gravity = [0; 0; -g];

    % Thrust vector in body frame, then transform to inertial
    thrust_body = [0; 0; F];
    thrust_inertial = R_bi * thrust_body;

    % Aerodynamic drag (proportional to velocity)
    drag = -[kdx * vx; kdy * vy; kdz * vz];

    % External disturbance forces
    F_dist = disturbance(1:3);

    % Translational acceleration
    acc = gravity + thrust_inertial / m + drag / m + F_dist / m;

    %% Rotational dynamics
    % Body torques
    tau = [tau_x; tau_y; tau_z];

    % Rotational friction torques
    tau_friction = -[kfp * p; kfq * q; kfr * r];

    % External disturbance torques
    tau_dist = disturbance(4:6);

    % Gyroscopic effects
    gyro = cross(omega_b, [Ixx; Iyy; Izz] .* omega_b);

    % Angular acceleration (Euler's equation)
    omega_b_dot = [
        (tau_x + tau_friction(1) + tau_dist(1) - (Izz - Iyy) * q * r) / Ixx;
        (tau_y + tau_friction(2) + tau_dist(2) - (Ixx - Izz) * p * r) / Iyy;
        (tau_z + tau_friction(3) + tau_dist(3) - (Iyy - Ixx) * p * q) / Izz
    ];

    %% Euler angle kinematics
    % Transform body rates to Euler angle rates
    % Singularity at theta = +/- 90 degrees
    if abs(cth) < 1e-6
        cth = sign(cth) * 1e-6;
    end

    euler_dot = [
        p + sphi * (sth/cth) * q + cphi * (sth/cth) * r;
        cphi * q - sphi * r;
        (sphi/cth) * q + (cphi/cth) * r
    ];

    %% Assemble state derivative
    x_dot = zeros(12, 1);
    x_dot(1:3) = vel;           % Position derivative = velocity
    x_dot(4:6) = acc;           % Velocity derivative = acceleration
    x_dot(7:9) = euler_dot;     % Euler angle derivatives
    x_dot(10:12) = omega_b_dot; % Angular rate derivatives

    %% Additional outputs
    if nargout > 1
        outputs.thrust = F;
        outputs.torque = tau;
        outputs.R = R_bi;
        outputs.acc = acc;
        outputs.omega_b_dot = omega_b_dot;
        outputs.drag = drag;
        outputs.disturbance = disturbance;
    end

end
