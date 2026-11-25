%% demo_trajectory_tracking.m - Aggressive Maneuver Tracking Demo
% Demonstrates TD3-ADRC tracking sinusoidal + step attitude commands
%
% Use Case 2: Track aggressive attitude reference signals
%
% Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"

clear; clc; close all;

%% Setup
setup_paths;

% Simulation parameters
Ts = 0.01;          % Sample time [s]
T_sim = 20;         % Total simulation time [s]
t = 0:Ts:T_sim;
N = length(t);

% Load parameters
quad_params = quadrotor_params();

%% Reference Signals (from paper)
% phi_d = 10 deg (step at t=0)
% theta_d = -10 deg (step at t=0)
% psi_d = -15 deg (step at t=0)
% At t=10s: phi_d = 5*sin(2*pi*0.05*(t-10))

phi_d = zeros(1, N);
theta_d = zeros(1, N);
psi_d = zeros(1, N);

for i = 1:N
    ti = t(i);

    if ti < 10
        % Step reference
        phi_d(i) = deg2rad(10);
        theta_d(i) = deg2rad(-10);
        psi_d(i) = deg2rad(-15);
    else
        % Sinusoidal reference for roll, step for pitch/yaw
        phi_d(i) = deg2rad(5 * sin(2*pi*0.05*(ti - 10)));
        theta_d(i) = deg2rad(-10);
        psi_d(i) = deg2rad(-15);
    end
end

ref = [phi_d; theta_d; psi_d];

%% Small disturbance (model uncertainty)
D = zeros(6, N);
for i = 1:N
    ti = t(i);
    D(4, i) = 0.005 * sin(2*pi*0.5 * ti);   % Small roll disturbance
    D(5, i) = 0.005 * cos(2*pi*0.5 * ti);   % Small pitch disturbance
end

%% Initialize Controllers

% 1. PID Controller
pid = struct();
pid.kp = [18; 18; 10];     % Higher gains for aggressive tracking
pid.ki = [3; 3; 2];
pid.kd = [6; 6; 4];
pid.integral = zeros(3, 1);
pid.last_error = zeros(3, 1);

% 2. Standard ADRC
adrc_ctrl = td3_adrc_controller(Ts, false);

% 3. TD3-ADRC
td3_adrc_ctrl = td3_adrc_controller(Ts, true);

%% Simulation Function with Detailed Dynamics

%% Run Simulations
fprintf('Running trajectory tracking simulations...\n');

% PID
fprintf('  Simulating PID...\n');
results_pid = simulate_tracking('PID', [], ref, D, t, quad_params, pid);

% Reset PID
pid.integral = zeros(3, 1);
pid.last_error = zeros(3, 1);

% ADRC
fprintf('  Simulating ADRC...\n');
adrc_ctrl.reset();
results_adrc = simulate_tracking('ADRC', adrc_ctrl, ref, D, t, quad_params, []);

% TD3-ADRC
fprintf('  Simulating TD3-ADRC...\n');
td3_adrc_ctrl.reset();
results_td3 = simulate_tracking('TD3-ADRC', td3_adrc_ctrl, ref, D, t, quad_params, []);

%% Compute Performance Metrics
metrics_pid = compute_tracking_metrics(results_pid);
metrics_adrc = compute_tracking_metrics(results_adrc);
metrics_td3 = compute_tracking_metrics(results_td3);

%% Display Results
fprintf('\n========== Aggressive Maneuver Tracking Results ==========\n\n');

fprintf('Performance Metrics:\n');
fprintf('%-12s  IAE_roll  IAE_pitch  IAE_yaw   Overshoot_roll  Settling_time\n', 'Controller');
fprintf('%-12s  %.2f     %.2f      %.2f      %.1f%%           %.2fs\n', 'PID', ...
    metrics_pid.IAE_roll, metrics_pid.IAE_pitch, metrics_pid.IAE_yaw, ...
    metrics_pid.overshoot_roll, metrics_pid.settling_time_roll);
fprintf('%-12s  %.2f     %.2f      %.2f      %.1f%%           %.2fs\n', 'ADRC', ...
    metrics_adrc.IAE_roll, metrics_adrc.IAE_pitch, metrics_adrc.IAE_yaw, ...
    metrics_adrc.overshoot_roll, metrics_adrc.settling_time_roll);
fprintf('%-12s  %.2f     %.2f      %.2f      %.1f%%           %.2fs\n', 'TD3-ADRC', ...
    metrics_td3.IAE_roll, metrics_td3.IAE_pitch, metrics_td3.IAE_yaw, ...
    metrics_td3.overshoot_roll, metrics_td3.settling_time_roll);

%% Visualization
fprintf('\nGenerating plots...\n');

% Full comparison
fig1 = figure('Name', 'Trajectory Tracking', 'Position', [100 100 1400 800]);

% Roll tracking
subplot(3, 2, 1);
plot(t, rad2deg(ref(1, :)), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
hold on;
plot(t, rad2deg(results_pid.output(1, :)), 'b-', 'LineWidth', 1, 'DisplayName', 'PID');
plot(t, rad2deg(results_adrc.output(1, :)), 'g-', 'LineWidth', 1, 'DisplayName', 'ADRC');
plot(t, rad2deg(results_td3.output(1, :)), 'r-', 'LineWidth', 1.5, 'DisplayName', 'TD3-ADRC');
hold off;
xlabel('Time [s]');
ylabel('Roll [deg]');
title('Roll Tracking');
legend('Location', 'best');
grid on;
xlim([0 T_sim]);

% Pitch tracking
subplot(3, 2, 2);
plot(t, rad2deg(ref(2, :)), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
hold on;
plot(t, rad2deg(results_pid.output(2, :)), 'b-', 'LineWidth', 1);
plot(t, rad2deg(results_adrc.output(2, :)), 'g-', 'LineWidth', 1);
plot(t, rad2deg(results_td3.output(2, :)), 'r-', 'LineWidth', 1.5);
hold off;
xlabel('Time [s]');
ylabel('Pitch [deg]');
title('Pitch Tracking');
grid on;
xlim([0 T_sim]);

% Roll error zoom (step response)
subplot(3, 2, 3);
t_zoom = t <= 5;
plot(t(t_zoom), rad2deg(results_pid.error(1, t_zoom)), 'b-', 'LineWidth', 1, 'DisplayName', 'PID');
hold on;
plot(t(t_zoom), rad2deg(results_adrc.error(1, t_zoom)), 'g-', 'LineWidth', 1, 'DisplayName', 'ADRC');
plot(t(t_zoom), rad2deg(results_td3.error(1, t_zoom)), 'r-', 'LineWidth', 1.5, 'DisplayName', 'TD3-ADRC');
hold off;
xlabel('Time [s]');
ylabel('Roll Error [deg]');
title('Step Response (t=0-5s)');
legend('Location', 'best');
grid on;

% Sinusoidal tracking (t=10-20s)
subplot(3, 2, 4);
t_sin = t >= 10;
plot(t(t_sin), rad2deg(ref(1, t_sin)), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
hold on;
plot(t(t_sin), rad2deg(results_pid.output(1, t_sin)), 'b-', 'LineWidth', 1);
plot(t(t_sin), rad2deg(results_adrc.output(1, t_sin)), 'g-', 'LineWidth', 1);
plot(t(t_sin), rad2deg(results_td3.output(1, t_sin)), 'r-', 'LineWidth', 1.5);
hold off;
xlabel('Time [s]');
ylabel('Roll [deg]');
title('Sinusoidal Tracking (t=10-20s)');
grid on;

% Control effort
subplot(3, 2, 5);
plot(t, results_pid.control(1, :), 'b-', 'LineWidth', 1, 'DisplayName', 'PID');
hold on;
plot(t, results_adrc.control(1, :), 'g-', 'LineWidth', 1, 'DisplayName', 'ADRC');
plot(t, results_td3.control(1, :), 'r-', 'LineWidth', 1, 'DisplayName', 'TD3-ADRC');
hold off;
xlabel('Time [s]');
ylabel('\tau_{roll} [N\cdotm]');
title('Roll Control Effort');
legend('Location', 'best');
grid on;

% Angular rates
subplot(3, 2, 6);
plot(t, rad2deg(results_pid.angular_rate(1, :)), 'b-', 'LineWidth', 1, 'DisplayName', 'PID');
hold on;
plot(t, rad2deg(results_adrc.angular_rate(1, :)), 'g-', 'LineWidth', 1, 'DisplayName', 'ADRC');
plot(t, rad2deg(results_td3.angular_rate(1, :)), 'r-', 'LineWidth', 1, 'DisplayName', 'TD3-ADRC');
hold off;
xlabel('Time [s]');
ylabel('Roll Rate [deg/s]');
title('Roll Angular Rate');
grid on;

sgtitle('Aggressive Maneuver Tracking', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile('results', 'figures', 'trajectory_tracking.png'));

%% Export for Blender
fprintf('\nExporting data for Blender...\n');

results_cell = {results_pid, results_adrc, results_td3};
controller_names = {'PID', 'ADRC', 'TD3-ADRC'};
output_dir = fullfile('results', 'animations', 'trajectory_tracking');
export_comparison_to_blender(results_cell, controller_names, output_dir);

fprintf('\nDemo complete!\n');

%% Helper Functions

%% Local functions
function results = simulate_tracking(ctrl_type, ctrl, ref, D, t, quad_params, pid_params)
    N = length(t);
    Ts = t(2) - t(1);

    % State: [phi, theta, psi, p, q, r]
    x = zeros(6, 1);

    % Logging
    y_hist = zeros(3, N);
    u_hist = zeros(3, N);
    e_hist = zeros(3, N);
    omega_hist = zeros(3, N);

    % Inertia
    I = diag([quad_params.Ixx, quad_params.Iyy, quad_params.Izz]);

    % Friction
    kf = [quad_params.kfp; quad_params.kfq; quad_params.kfr];

    for i = 1:N
        euler = x(1:3);
        omega = x(4:6);
        ref_i = ref(:, i);
        tau_dist = D(4:6, i);

        % Compute control
        switch ctrl_type
            case 'PID'
                e = ref_i - euler;
                pid_params.integral = pid_params.integral + e * Ts;
                % Anti-windup
                pid_params.integral = max(-0.5, min(0.5, pid_params.integral));
                e_dot = (e - pid_params.last_error) / Ts;
                pid_params.last_error = e;

                tau = pid_params.kp .* e + pid_params.ki .* pid_params.integral + ...
                      pid_params.kd .* e_dot;

            case 'ADRC'
                [tau, ~] = ctrl.compute_control(ref_i, euler, t(i));

            case 'TD3-ADRC'
                [tau, ~] = ctrl.compute_control(ref_i, euler, t(i));
        end

        % Saturate control
        tau_max = [0.5; 0.5; 0.3];
        tau = max(-tau_max, min(tau_max, tau));

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
        x(1:3) = x(1:3) + Ts * euler_dot;
        x(4:6) = x(4:6) + Ts * omega_dot;

        % Log
        y_hist(:, i) = euler;
        u_hist(:, i) = tau;
        e_hist(:, i) = ref_i - euler;
        omega_hist(:, i) = omega;
    end

    results.time = t;
    results.reference = ref;
    results.output = y_hist;
    results.control = u_hist;
    results.error = e_hist;
    results.angular_rate = omega_hist;
end

function metrics = compute_tracking_metrics(results)
    e = results.error;
    y = results.output;
    ref = results.reference;
    t = results.time;
    dt = mean(diff(t));

    % IAE
    metrics.IAE_roll = sum(abs(e(1, :))) * dt;
    metrics.IAE_pitch = sum(abs(e(2, :))) * dt;
    metrics.IAE_yaw = sum(abs(e(3, :))) * dt;

    % Overshoot (for step response, first 10 seconds)
    t_step = t <= 10;
    ref_final = ref(1, find(t_step, 1, 'last'));
    y_max = max(abs(y(1, t_step)));
    if abs(ref_final) > 0.01
        metrics.overshoot_roll = (y_max - abs(ref_final)) / abs(ref_final) * 100;
    else
        metrics.overshoot_roll = 0;
    end
    metrics.overshoot_roll = max(0, metrics.overshoot_roll);

    % Settling time (2% criterion)
    settling_band = 0.02 * abs(ref_final);
    if settling_band < 1e-6
        settling_band = deg2rad(0.5);  % 0.5 deg default
    end

    settled_idx = find(abs(e(1, :)) <= settling_band, 1, 'first');
    if ~isempty(settled_idx)
        metrics.settling_time_roll = t(settled_idx);
    else
        metrics.settling_time_roll = t(end);
    end
end
