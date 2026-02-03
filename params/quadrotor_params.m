function params = quadrotor_params()
%QUADROTOR_PARAMS Return quadrotor physical parameters
%   Based on X-configuration quadrotor UAV
%
%   Returns:
%       params - Structure containing all quadrotor parameters
%
%   Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"

    %% Mass and Inertia
    params.m = 1.4;           % Total mass [kg]
    params.g = 9.81;          % Gravitational acceleration [m/s^2]

    % Moments of inertia [kg*m^2]
    params.Ixx = 0.0211;
    params.Iyy = 0.0219;
    params.Izz = 0.0366;
    params.I = diag([params.Ixx, params.Iyy, params.Izz]);

    %% Geometry
    params.L = 0.225;         % Arm length [m]
    params.arm_angle = pi/4;  % X-configuration angle [rad]

    %% Motor and Propeller Parameters
    params.kf = 6.11e-8;      % Thrust coefficient [N/(rad/s)^2]
    params.km = 1.5e-9;       % Moment coefficient [N*m/(rad/s)^2]
    params.kt = params.km / params.kf;  % Torque-to-thrust ratio

    % Motor limits
    params.omega_min = 0;           % Minimum motor speed [rad/s]
    params.omega_max = 8000;        % Maximum motor speed [rad/s]
    params.omega_hover = 5500;      % Hover motor speed [rad/s]

    % Motor time constant
    params.tau_motor = 0.02;        % Motor response time constant [s]

    %% Aerodynamic Parameters
    params.Cd = 0.1;          % Drag coefficient
    params.rho = 1.225;       % Air density [kg/m^3]
    params.A = 0.1;           % Reference area [m^2]

    % Aerodynamic drag coefficients
    params.kdx = 0.01;        % Translational drag x [N/(m/s)]
    params.kdy = 0.01;        % Translational drag y [N/(m/s)]
    params.kdz = 0.02;        % Translational drag z [N/(m/s)]

    % Rotational friction coefficients
    params.kfp = 0.001;       % Friction about x-axis [N*m/(rad/s)]
    params.kfq = 0.001;       % Friction about y-axis [N*m/(rad/s)]
    params.kfr = 0.002;       % Friction about z-axis [N*m/(rad/s)]

    %% Control Allocation Matrix
    % Mixer matrix: [F; tau_x; tau_y; tau_z] = B * [omega1^2; omega2^2; omega3^2; omega4^2]
    % X-configuration: motors at 45, 135, 225, 315 degrees
    L = params.L;
    kf = params.kf;
    km = params.km;
    angle = params.arm_angle;

    % Motor positions (X-config)
    %   1 (CW)  2 (CCW)
    %      \  /
    %       \/
    %       /\
    %      /  \
    %   4 (CCW) 3 (CW)

    params.B = [
        kf,                kf,                kf,                kf;                % Total thrust
        kf*L*sin(angle),  -kf*L*sin(angle),  -kf*L*sin(angle),   kf*L*sin(angle);   % Roll torque
       -kf*L*cos(angle),  -kf*L*cos(angle),   kf*L*cos(angle),   kf*L*cos(angle);   % Pitch torque
       -km,                km,               -km,                km                  % Yaw torque
    ];

    % Inverse mixer for control allocation
    params.B_inv = pinv(params.B);

    %% State Limits
    % Position limits [m]
    params.pos_max = [100; 100; 100];
    params.pos_min = [-100; -100; 0];

    % Velocity limits [m/s]
    params.vel_max = [20; 20; 10];

    % Attitude limits [rad]
    params.phi_max = deg2rad(45);     % Roll limit
    params.theta_max = deg2rad(45);   % Pitch limit
    params.psi_max = deg2rad(180);    % Yaw limit (no limit effectively)

    % Angular rate limits [rad/s]
    params.p_max = deg2rad(200);
    params.q_max = deg2rad(200);
    params.r_max = deg2rad(100);

    %% Simulation Parameters
    params.dt = 0.001;        % Simulation time step [s]
    params.dt_control = 0.01; % Control loop time step [s]

end
