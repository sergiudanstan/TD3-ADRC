function [omega_actual, motor_state] = motor_model(omega_cmd, motor_state, params, dt)
%MOTOR_MODEL First-order motor dynamics model
%   Models the motor response as a first-order system with saturation
%
%   Inputs:
%       omega_cmd   - Commanded motor speeds (4 x 1) [rad/s]
%       motor_state - Current motor state structure
%       params      - Quadrotor parameters from quadrotor_params()
%       dt          - Time step [s]
%
%   Outputs:
%       omega_actual - Actual motor speeds (4 x 1) [rad/s]
%       motor_state  - Updated motor state structure
%
%   Motor dynamics: tau * d(omega)/dt + omega = omega_cmd
%
%   Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"

    %% Initialize motor state if needed
    if isempty(motor_state) || ~isfield(motor_state, 'omega')
        motor_state.omega = params.omega_hover * ones(4, 1);
        motor_state.omega_dot = zeros(4, 1);
    end

    %% Extract parameters
    tau = params.tau_motor;           % Motor time constant
    omega_min = params.omega_min;     % Minimum motor speed
    omega_max = params.omega_max;     % Maximum motor speed

    %% Saturate commanded speeds
    omega_cmd = max(omega_min, min(omega_max, omega_cmd));

    %% First-order dynamics
    % tau * d(omega)/dt + omega = omega_cmd
    % Discretized: omega[k+1] = omega[k] + (dt/tau) * (omega_cmd - omega[k])

    alpha = dt / tau;
    if alpha > 1
        alpha = 1;  % Prevent overshoot for large time steps
    end

    omega_prev = motor_state.omega;
    omega_new = omega_prev + alpha * (omega_cmd - omega_prev);

    %% Apply rate limits (optional)
    max_rate = 10000;  % Maximum rate of change [rad/s^2]
    omega_dot = (omega_new - omega_prev) / dt;

    % Limit rate of change
    omega_dot = max(-max_rate, min(max_rate, omega_dot));
    omega_new = omega_prev + omega_dot * dt;

    %% Apply final saturation
    omega_new = max(omega_min, min(omega_max, omega_new));

    %% Update state
    motor_state.omega = omega_new;
    motor_state.omega_dot = omega_dot;
    motor_state.omega_cmd = omega_cmd;

    %% Output
    omega_actual = omega_new;

end


function omega_motors = control_allocation(F, tau, params)
%CONTROL_ALLOCATION Convert desired forces/torques to motor speeds
%   Uses the inverse mixer matrix to compute individual motor speeds
%
%   Inputs:
%       F      - Desired total thrust [N]
%       tau    - Desired torques [tau_x; tau_y; tau_z] [N*m]
%       params - Quadrotor parameters
%
%   Outputs:
%       omega_motors - Motor speeds (4 x 1) [rad/s]

    % Desired wrench
    wrench = [F; tau(1); tau(2); tau(3)];

    % Use inverse mixer to get motor speed squared
    omega_sq = params.B_inv * wrench;

    % Ensure non-negative
    omega_sq = max(0, omega_sq);

    % Take square root to get motor speeds
    omega_motors = sqrt(omega_sq);

    % Saturate to limits
    omega_motors = max(params.omega_min, min(params.omega_max, omega_motors));

end


function [F, tau] = compute_thrust_torque(omega_motors, params)
%COMPUTE_THRUST_TORQUE Compute thrust and torques from motor speeds
%   Uses the mixer matrix to compute total thrust and body torques
%
%   Inputs:
%       omega_motors - Motor speeds (4 x 1) [rad/s]
%       params       - Quadrotor parameters
%
%   Outputs:
%       F   - Total thrust [N]
%       tau - Body torques [tau_x; tau_y; tau_z] [N*m]

    omega_sq = omega_motors.^2;
    wrench = params.B * omega_sq;

    F = wrench(1);
    tau = wrench(2:4);

end
