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
I = diag([quad_params.Ixx, quad_params.Iyy, quad_params.Izz]);
kf = [quad_params.kfp; quad_params.kfq; quad_params.kfr];

%% Reference Signal (hover - constant attitude)
ref = repmat(deg2rad([0; 0; 0]), 1, N);

%% Wind Disturbance Model (from paper)
D = zeros(6, N);
for i = 1:N
    ti = t(i);
    D(1, i) = 0.3 * sin(2*pi/15 * ti);      % Fx
    D(2, i) = 0.2 * cos(2*pi/10 * ti);      % Fy
    D(3, i) = 0.01 * sin(2*pi/10 * ti);     % Fz
    D(4, i) = 0.02 * sin(2*pi/8 * ti);      % tau_roll
    D(5, i) = 0.02 * cos(2*pi/8 * ti);      % tau_pitch
    D(6, i) = 0.01 * sin(2*pi/12 * ti);     % tau_yaw
end

fprintf('Running wind disturbance simulations...\n');

%% 1. PID Controller
fprintf('  Simulating PID...\n');
pid_kp = [15; 15; 8];
pid_ki = [2; 2; 1];
pid_kd = [5; 5; 3];
pid_int = zeros(3, 1);
pid_last_e = zeros(3, 1);

x = zeros(6, 1);
y_pid = zeros(3, N);
u_pid = zeros(3, N);
e_pid = zeros(3, N);

for i = 1:N
    euler = x(1:3);
    omega = x(4:6);

    e = ref(:, i) - euler;
    pid_int = pid_int + e * Ts;
    pid_int = max(-0.5, min(0.5, pid_int));  % Anti-windup
    e_dot = (e - pid_last_e) / Ts;
    pid_last_e = e;

    tau = pid_kp .* e + pid_ki .* pid_int + pid_kd .* e_dot;
    tau = max(-[0.5;0.5;0.3], min([0.5;0.5;0.3], tau));  % Saturation

    tau_dist = D(4:6, i);
    gyro = cross(omega, I * omega);
    friction = -kf .* omega;
    omega_dot = I \ (tau + friction - gyro + tau_dist);

    phi = euler(1); theta = euler(2);
    sphi = sin(phi); cphi = cos(phi);
    cth = cos(theta);
    if abs(cth) < 1e-6, cth = sign(cth) * 1e-6; end
    tth = sin(theta) / cth;

    euler_dot = [omega(1) + sphi*tth*omega(2) + cphi*tth*omega(3);
                 cphi*omega(2) - sphi*omega(3);
                 (sphi/cth)*omega(2) + (cphi/cth)*omega(3)];

    x(1:3) = x(1:3) + Ts * euler_dot;
    x(4:6) = x(4:6) + Ts * omega_dot;

    y_pid(:, i) = euler;
    u_pid(:, i) = tau;
    e_pid(:, i) = e;
end

%% 2. ADRC Controller
fprintf('  Simulating ADRC...\n');
adrc_ctrl = td3_adrc_controller(Ts, false);
adrc_ctrl.reset();

x = zeros(6, 1);
y_adrc = zeros(3, N);
u_adrc = zeros(3, N);
e_adrc = zeros(3, N);

for i = 1:N
    euler = x(1:3);
    omega = x(4:6);

    [tau, ~] = adrc_ctrl.compute_control(ref(:, i), euler, t(i));

    tau_dist = D(4:6, i);
    gyro = cross(omega, I * omega);
    friction = -kf .* omega;
    omega_dot = I \ (tau + friction - gyro + tau_dist);

    phi = euler(1); theta = euler(2);
    sphi = sin(phi); cphi = cos(phi);
    cth = cos(theta);
    if abs(cth) < 1e-6, cth = sign(cth) * 1e-6; end
    tth = sin(theta) / cth;

    euler_dot = [omega(1) + sphi*tth*omega(2) + cphi*tth*omega(3);
                 cphi*omega(2) - sphi*omega(3);
                 (sphi/cth)*omega(2) + (cphi/cth)*omega(3)];

    x(1:3) = x(1:3) + Ts * euler_dot;
    x(4:6) = x(4:6) + Ts * omega_dot;

    y_adrc(:, i) = euler;
    u_adrc(:, i) = tau;
    e_adrc(:, i) = ref(:, i) - euler;
end

%% 3. TD3-ADRC Controller (load trained agent)
fprintf('  Simulating TD3-ADRC...\n');

td3_ctrl = td3_adrc_controller(Ts, true);

% Load trained weights if available
try
    main_data = load('results/td3_agent_trained.mat');
    actor_file = load(main_data.actor_file);
    actor_weights = actor_file.actor_data;
    td3_ctrl.td3_agent.actor.W1 = actor_weights.W1;
    td3_ctrl.td3_agent.actor.b1 = actor_weights.b1;
    td3_ctrl.td3_agent.actor.W2 = actor_weights.W2;
    td3_ctrl.td3_agent.actor.b2 = actor_weights.b2;
    td3_ctrl.td3_agent.actor.W3 = actor_weights.W3;
    td3_ctrl.td3_agent.actor.b3 = actor_weights.b3;
    fprintf('    (using trained agent)\n');
catch
    fprintf('    (using untrained agent)\n');
end
td3_ctrl.reset();

x = zeros(6, 1);
y_td3 = zeros(3, N);
u_td3 = zeros(3, N);
e_td3 = zeros(3, N);

for i = 1:N
    euler = x(1:3);
    omega = x(4:6);

    [tau, ~] = td3_ctrl.compute_control(ref(:, i), euler, t(i));

    tau_dist = D(4:6, i);
    gyro = cross(omega, I * omega);
    friction = -kf .* omega;
    omega_dot = I \ (tau + friction - gyro + tau_dist);

    phi = euler(1); theta = euler(2);
    sphi = sin(phi); cphi = cos(phi);
    cth = cos(theta);
    if abs(cth) < 1e-6, cth = sign(cth) * 1e-6; end
    tth = sin(theta) / cth;

    euler_dot = [omega(1) + sphi*tth*omega(2) + cphi*tth*omega(3);
                 cphi*omega(2) - sphi*omega(3);
                 (sphi/cth)*omega(2) + (cphi/cth)*omega(3)];

    x(1:3) = x(1:3) + Ts * euler_dot;
    x(4:6) = x(4:6) + Ts * omega_dot;

    y_td3(:, i) = euler;
    u_td3(:, i) = tau;
    e_td3(:, i) = ref(:, i) - euler;
end

%% Compute Performance Metrics
fprintf('\nComputing performance metrics...\n');

dt = Ts;
iae_pid = [sum(abs(e_pid(1,:))), sum(abs(e_pid(2,:))), sum(abs(e_pid(3,:)))] * dt * 180/pi;
iae_adrc = [sum(abs(e_adrc(1,:))), sum(abs(e_adrc(2,:))), sum(abs(e_adrc(3,:)))] * dt * 180/pi;
iae_td3 = [sum(abs(e_td3(1,:))), sum(abs(e_td3(2,:))), sum(abs(e_td3(3,:)))] * dt * 180/pi;

%% Display Results
fprintf('\n========== Wind Disturbance Rejection Results ==========\n\n');
fprintf('Integral Absolute Error (IAE) in deg*s:\n');
fprintf('Controller    Roll     Pitch     Yaw      Total\n');
fprintf('------------------------------------------------------\n');
fprintf('PID          %6.2f   %6.2f   %6.2f   %6.2f\n', iae_pid(1), iae_pid(2), iae_pid(3), sum(iae_pid));
fprintf('ADRC         %6.2f   %6.2f   %6.2f   %6.2f\n', iae_adrc(1), iae_adrc(2), iae_adrc(3), sum(iae_adrc));
fprintf('TD3-ADRC     %6.2f   %6.2f   %6.2f   %6.2f\n', iae_td3(1), iae_td3(2), iae_td3(3), sum(iae_td3));

if sum(iae_adrc) > 0
    imp_td3 = (sum(iae_adrc) - sum(iae_td3)) / sum(iae_adrc) * 100;
    fprintf('\nTD3-ADRC vs ADRC: %+.1f%%\n', imp_td3);
end
if sum(iae_pid) > 0
    imp_adrc = (sum(iae_pid) - sum(iae_adrc)) / sum(iae_pid) * 100;
    fprintf('ADRC vs PID: %+.1f%%\n', imp_adrc);
end

%% Visualization
fprintf('\nGenerating plots...\n');

% Build results structures for plotting
results_pid.time = t; results_pid.reference = ref; results_pid.output = y_pid;
results_pid.control = u_pid; results_pid.error = e_pid;

results_adrc.time = t; results_adrc.reference = ref; results_adrc.output = y_adrc;
results_adrc.control = u_adrc; results_adrc.error = e_adrc;

results_td3.time = t; results_td3.reference = ref; results_td3.output = y_td3;
results_td3.control = u_td3; results_td3.error = e_td3;

results_cell = {results_pid, results_adrc, results_td3};
controller_names = {'PID', 'ADRC', 'TD3-ADRC'};

% Comparison plot
fig1 = figure('Name', 'Wind Disturbance Rejection', 'Position', [100 100 1400 600]);

colors = lines(3);

% Roll
subplot(2, 3, 1);
plot(t, rad2deg(y_pid(1,:)), '-', 'Color', colors(1,:), 'LineWidth', 1.5); hold on;
plot(t, rad2deg(y_adrc(1,:)), '-', 'Color', colors(2,:), 'LineWidth', 1.5);
plot(t, rad2deg(y_td3(1,:)), '-', 'Color', colors(3,:), 'LineWidth', 1.5);
plot(t, rad2deg(ref(1,:)), 'k--', 'LineWidth', 1);
hold off;
xlabel('Time [s]'); ylabel('Roll [deg]');
title('Roll Response');
legend('PID', 'ADRC', 'TD3-ADRC', 'Ref', 'Location', 'best');
grid on;

% Pitch
subplot(2, 3, 2);
plot(t, rad2deg(y_pid(2,:)), '-', 'Color', colors(1,:), 'LineWidth', 1.5); hold on;
plot(t, rad2deg(y_adrc(2,:)), '-', 'Color', colors(2,:), 'LineWidth', 1.5);
plot(t, rad2deg(y_td3(2,:)), '-', 'Color', colors(3,:), 'LineWidth', 1.5);
plot(t, rad2deg(ref(2,:)), 'k--', 'LineWidth', 1);
hold off;
xlabel('Time [s]'); ylabel('Pitch [deg]');
title('Pitch Response');
grid on;

% Yaw
subplot(2, 3, 3);
plot(t, rad2deg(y_pid(3,:)), '-', 'Color', colors(1,:), 'LineWidth', 1.5); hold on;
plot(t, rad2deg(y_adrc(3,:)), '-', 'Color', colors(2,:), 'LineWidth', 1.5);
plot(t, rad2deg(y_td3(3,:)), '-', 'Color', colors(3,:), 'LineWidth', 1.5);
plot(t, rad2deg(ref(3,:)), 'k--', 'LineWidth', 1);
hold off;
xlabel('Time [s]'); ylabel('Yaw [deg]');
title('Yaw Response');
grid on;

% Roll error
subplot(2, 3, 4);
plot(t, rad2deg(abs(e_pid(1,:))), '-', 'Color', colors(1,:), 'LineWidth', 1); hold on;
plot(t, rad2deg(abs(e_adrc(1,:))), '-', 'Color', colors(2,:), 'LineWidth', 1);
plot(t, rad2deg(abs(e_td3(1,:))), '-', 'Color', colors(3,:), 'LineWidth', 1);
hold off;
xlabel('Time [s]'); ylabel('|Error| [deg]');
title('Roll Absolute Error');
grid on;

% Pitch error
subplot(2, 3, 5);
plot(t, rad2deg(abs(e_pid(2,:))), '-', 'Color', colors(1,:), 'LineWidth', 1); hold on;
plot(t, rad2deg(abs(e_adrc(2,:))), '-', 'Color', colors(2,:), 'LineWidth', 1);
plot(t, rad2deg(abs(e_td3(2,:))), '-', 'Color', colors(3,:), 'LineWidth', 1);
hold off;
xlabel('Time [s]'); ylabel('|Error| [deg]');
title('Pitch Absolute Error');
grid on;

% Yaw error
subplot(2, 3, 6);
plot(t, rad2deg(abs(e_pid(3,:))), '-', 'Color', colors(1,:), 'LineWidth', 1); hold on;
plot(t, rad2deg(abs(e_adrc(3,:))), '-', 'Color', colors(2,:), 'LineWidth', 1);
plot(t, rad2deg(abs(e_td3(3,:))), '-', 'Color', colors(3,:), 'LineWidth', 1);
hold off;
xlabel('Time [s]'); ylabel('|Error| [deg]');
title('Yaw Absolute Error');
grid on;

sgtitle('Wind Disturbance Rejection Comparison', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile('results', 'figures', 'wind_disturbance_comparison.png'));

% Disturbance profile plot
fig2 = figure('Name', 'Wind Disturbance Profile');
subplot(2, 1, 1);
plot(t, D(1:3, :)', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Force [N]');
legend('F_x', 'F_y', 'F_z');
title('Wind Force Disturbance');
grid on;

subplot(2, 1, 2);
plot(t, D(4:6, :)', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Torque [N\cdotm]');
legend('\tau_{roll}', '\tau_{pitch}', '\tau_{yaw}');
title('Wind Torque Disturbance');
grid on;

saveas(fig2, fullfile('results', 'figures', 'wind_disturbance_profile.png'));

%% Export for Blender
fprintf('\nExporting data for Blender visualization...\n');
output_dir = fullfile('results', 'animations', 'wind_disturbance');
export_comparison_to_blender(results_cell, controller_names, output_dir);

fprintf('\nDemo complete!\n');
