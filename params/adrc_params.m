function params = adrc_params()
%ADRC_PARAMS Return ADRC controller parameters
%   Parameters for Tracking Differentiator, Extended State Observer,
%   and Nonlinear State Error Feedback
%
%   Returns:
%       params - Structure containing all ADRC parameters
%
%   Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"

    %% Tracking Differentiator Parameters
    % TD smooths the reference signal and provides derivative estimate
    params.td.r = 100;        % Tracking speed parameter
    params.td.h = 0.01;       % Filter parameter (sampling time)

    %% Extended State Observer Parameters
    % ESO estimates states and total disturbance

    % Observer bandwidth (affects convergence speed and noise sensitivity)
    params.eso.omega_o = 80;  % Observer bandwidth [rad/s]

    % Second-order ESO gains (pole placement at -omega_o)
    % For 2nd order system: beta1 = 2*omega_o, beta2 = omega_o^2
    params.eso.beta1 = 2 * params.eso.omega_o;
    params.eso.beta2 = params.eso.omega_o^2;

    % Third-order ESO gains (for extended state)
    % Pole placement at -omega_o (triple pole)
    params.eso.beta1_3rd = 3 * params.eso.omega_o;
    params.eso.beta2_3rd = 3 * params.eso.omega_o^2;
    params.eso.beta3_3rd = params.eso.omega_o^3;

    % ESO nonlinear gains (for nonlinear ESO variant)
    params.eso.alpha1 = 0.5;  % Nonlinear gain exponent
    params.eso.alpha2 = 0.25;
    params.eso.delta = 0.01;  % Linear zone width

    %% Nonlinear State Error Feedback Parameters
    % NLSEF generates control signal from error states

    % Controller bandwidth
    params.nlsef.omega_c = 15;  % Controller bandwidth [rad/s]

    % PD-like gains (derived from bandwidth)
    params.nlsef.kp = params.nlsef.omega_c^2;
    params.nlsef.kd = 2 * params.nlsef.omega_c;

    % Nonlinear function parameters
    params.nlsef.alpha1 = 0.75;   % Position error nonlinearity
    params.nlsef.alpha2 = 1.25;   % Velocity error nonlinearity
    params.nlsef.delta = 0.1;     % Linear zone threshold

    %% Novel Parameterized tanh Function Parameters
    % Replaces discontinuous fal function with smooth tanh-based function
    % y = -(delta^(alpha*e) - delta^(-alpha*e)) / (delta^(alpha*e) + delta^(-alpha*e))

    params.tanh.alpha = 1.0;      % Nonlinearity shaping parameter
    params.tanh.delta = 0.5;      % Smoothness parameter (0 < delta < 1)

    % Channel-specific tanh parameters (initial values, TD3 will tune these)
    % Roll channel
    params.tanh.roll.alpha = 1.0;
    params.tanh.roll.delta = 0.5;

    % Pitch channel
    params.tanh.pitch.alpha = 1.0;
    params.tanh.pitch.delta = 0.5;

    % Yaw channel
    params.tanh.yaw.alpha = 0.8;
    params.tanh.yaw.delta = 0.6;

    %% Channel-Specific ADRC Gains
    % Different dynamics for roll, pitch, yaw require different tuning

    % Roll channel (fast dynamics)
    params.roll.eso_omega = 100;
    params.roll.nlsef_omega = 20;
    params.roll.b0 = 1.0;         % Control gain estimate

    % Pitch channel (fast dynamics)
    params.pitch.eso_omega = 100;
    params.pitch.nlsef_omega = 20;
    params.pitch.b0 = 1.0;

    % Yaw channel (slower dynamics)
    params.yaw.eso_omega = 60;
    params.yaw.nlsef_omega = 10;
    params.yaw.b0 = 0.5;

    %% Disturbance Compensation
    params.disturbance.enable = true;
    params.disturbance.filter_tau = 0.05;  % Low-pass filter time constant

    %% Anti-Windup
    params.antiwindup.enable = true;
    params.antiwindup.limit = 10;          % Integrator limit

    %% Sample Time
    params.Ts = 0.01;  % ADRC update rate [s]

end
