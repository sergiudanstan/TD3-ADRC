%% demo_wind_disturbance.m - Wind Gust Disturbance Rejection Demo
% Demonstrates TD3-ADRC vs other controllers under wind disturbance
%
% Use Case 1: Quadrotor maintaining hover under time-varying wind
%
% Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"

clear; clc; close all;

%% Setup
setup_paths;

% Simulation parameters
Ts = 0.01;          % Sample time [s]
T_sim = 30;         % Total simulation time [s]
t = 0:Ts:T_sim;
N = length(t);

% Load parameters
quad_params = quadrotor_params();

%% Reference Signal (hover - constant attitude)
phi_d = deg2rad(0);
theta_d = deg2rad(0);
psi_d = deg2rad(0);

ref = repmat([phi_d; theta_d; psi_d], 1, N);

%% Wind Disturbance Model (from paper)
% D = [Fx, Fy, Fz, tau_x, tau_y, tau_z]'
D = zeros(6, N);

for i = 1:N
    ti = t(i);
    % Force disturbances
    D(1, i) = 0.3 * sin(2*pi/15 * ti);      % Fx
    D(2, i) = 0.2 * cos(2*pi/10 * ti);      % Fy
    D(3, i) = 0.01 * sin(2*pi/10 * ti);     % Fz

    % Torque disturbances
    D(4, i) = 0.02 * sin(2*pi/8 * ti);      % tau_roll
    D(5, i) = 0.02 * cos(2*pi/8 * ti);      % tau_pitch
    D(6, i) = 0.01 * sin(2*pi/12 * ti);     % tau_yaw
end

%% Initialize Controllers

% 1. PID Controller
pid = struct();
pid.kp = [15; 15; 8];      % Proportional gains
pid.ki = [2; 2; 1];        % Integral gains
pid.kd = [5; 5; 3];        % Derivative gains
pid.integral = zeros(3, 1);
pid.last_error = zeros(3, 1);

% 2. Standard ADRC (no TD3)
adrc_ctrl = td3_adrc_controller(Ts, false);  % TD3 disabled
adrc_ctrl.set_adrc_only();

% 3. TD3-ADRC (with pre-trained agent or random init)
td3_adrc_ctrl = td3_adrc_controller(Ts, true);

%% Simulation Function
function [results, state] = simulate_attitude(ctrl_type, ctrl, ref, D, t, quad_params, pid_params)
    N = length(t);
    Ts = t(2) - t(1);

    % State: [phi, theta, psi, p, q, r]
    x = zeros(6, 1);

    % Logging
    y_hist = zeros(3, N);
    u_hist = zeros(3, N);
    e_hist = zeros(3, N);

    % Simplified attitude dynamics
    % I * omega_dot = tau - omega x (I*omega) + tau_dist
    I = diag([quad_params.Ixx, quad_params.Iyy, quad_params.Izz]);

    for i = 1:N
        % Current attitude
        euler = x(1:3);
        omega = x(4:6);

        % Reference
        ref_i = ref(:, i);

        % Disturbance
        tau_dist = D(4:6, i);

        % Compute control
        switch ctrl_type
            case 'PID'
                e = ref_i - euler;
                pid_params.integral = pid_params.integral + e * Ts;
                e_dot = (e - pid_params.last_error) / Ts;
                pid_params.last_error = e;

                tau = pid_params.kp .* e + pid_params.ki .* pid_params.integral + ...
                      pid_params.kd .* e_dot;

            case 'ADRC'
                [tau, ~] = ctrl.compute_control(ref_i, euler, t(i));

            case 'TD3-ADRC'
                [tau, ~] = ctrl.compute_control(ref_i, euler, t(i));
        end

        % Attitude dynamics
        gyro = cross(omega, I * omega);
        omega_dot = I \ (tau - gyro + tau_dist);

        % Euler kinematics
        phi = euler(1); theta = euler(2);
        sphi = sin(phi); cphi = cos(phi);
        cth = cos(theta);

        if abs(cth) < 1e-6
            cth = sign(cth) * 1e-6;
        end

        euler_dot = [
            omega(1) + sphi * tan(theta) * omega(2) + cphi * tan(theta) * omega(3);
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
    end

    results.time = t;
    results.reference = ref;
    results.output = y_hist;
    results.control = u_hist;
    results.error = e_hist;
    state = x;
end

%% Run Simulations
fprintf('Running wind disturbance simulations...\n');

% PID
fprintf('  Simulating PID...\n');
[results_pid, ~] = simulate_attitude('PID', [], ref, D, t, quad_params, pid);

% Reset PID state for fresh simulation
pid.integral = zeros(3, 1);
pid.last_error = zeros(3, 1);

% ADRC
fprintf('  Simulating ADRC...\n');
adrc_ctrl.reset();
[results_adrc, ~] = simulate_attitude('ADRC', adrc_ctrl, ref, D, t, quad_params, []);

% TD3-ADRC
fprintf('  Simulating TD3-ADRC...\n');
td3_adrc_ctrl.reset();
[results_td3, ~] = simulate_attitude('TD3-ADRC', td3_adrc_ctrl, ref, D, t, quad_params, []);

%% Compute Performance Metrics
fprintf('\nComputing performance metrics...\n');

metrics_pid = compute_metrics(results_pid);
metrics_adrc = compute_metrics(results_adrc);
metrics_td3 = compute_metrics(results_td3);

%% Display Results
fprintf('\n========== Wind Disturbance Rejection Results ==========\n\n');

fprintf('Integral Absolute Error (IAE):\n');
fprintf('%-12s  Roll     Pitch     Yaw      Total\n', 'Controller');
fprintf('%-12s  %.2f    %.2f    %.2f    %.2f\n', 'PID', ...
    metrics_pid.IAE_roll, metrics_pid.IAE_pitch, metrics_pid.IAE_yaw, metrics_pid.IAE_total);
fprintf('%-12s  %.2f    %.2f    %.2f    %.2f\n', 'ADRC', ...
    metrics_adrc.IAE_roll, metrics_adrc.IAE_pitch, metrics_adrc.IAE_yaw, metrics_adrc.IAE_total);
fprintf('%-12s  %.2f    %.2f    %.2f    %.2f\n', 'TD3-ADRC', ...
    metrics_td3.IAE_roll, metrics_td3.IAE_pitch, metrics_td3.IAE_yaw, metrics_td3.IAE_total);

% Calculate improvement
if metrics_adrc.IAE_total > 0
    improvement = (metrics_adrc.IAE_total - metrics_td3.IAE_total) / metrics_adrc.IAE_total * 100;
    fprintf('\nTD3-ADRC improvement over ADRC: %.2f%%\n', improvement);
end

%% Visualization
fprintf('\nGenerating plots...\n');

% Controller comparison plot
results_cell = {results_pid, results_adrc, results_td3};
controller_names = {'PID', 'ADRC', 'TD3-ADRC'};

fig1 = plot_controller_comparison(results_cell, controller_names, 'Wind Disturbance Rejection');
saveas(fig1, fullfile('results', 'figures', 'wind_disturbance_comparison.png'));

% Disturbance plot
fig2 = figure('Name', 'Wind Disturbance Profile');
subplot(2, 1, 1);
plot(t, D(1:3, :)', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Force [N]');
legend('F_x', 'F_y', 'F_z');
title('Wind Force Disturbance');
grid on;

subplot(2, 1, 2);
plot(t, D(4:6, :)', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Torque [N\cdotm]');
legend('\tau_x', '\tau_y', '\tau_z');
title('Wind Torque Disturbance');
grid on;

saveas(fig2, fullfile('results', 'figures', 'wind_disturbance_profile.png'));

% Performance bar chart
metrics_cell = {metrics_pid, metrics_adrc, metrics_td3};
fig3 = plot_performance_bar(metrics_cell, controller_names);
saveas(fig3, fullfile('results', 'figures', 'wind_disturbance_metrics.png'));

%% Export for Blender
fprintf('\nExporting data for Blender visualization...\n');

% Export all controllers
output_dir = fullfile('results', 'animations', 'wind_disturbance');
export_comparison_to_blender(results_cell, controller_names, output_dir);

fprintf('\nDemo complete!\n');

%% Helper Function
function metrics = compute_metrics(results)
    e = results.error;
    t = results.time;
    dt = mean(diff(t));

    metrics.IAE_roll = sum(abs(e(1, :))) * dt;
    metrics.IAE_pitch = sum(abs(e(2, :))) * dt;
    metrics.IAE_yaw = sum(abs(e(3, :))) * dt;
    metrics.IAE_total = metrics.IAE_roll + metrics.IAE_pitch + metrics.IAE_yaw;

    metrics.RMSE_roll = sqrt(mean(e(1, :).^2));
    metrics.RMSE_pitch = sqrt(mean(e(2, :).^2));
    metrics.RMSE_yaw = sqrt(mean(e(3, :).^2));
end
