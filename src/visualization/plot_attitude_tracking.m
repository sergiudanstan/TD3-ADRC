function fig = plot_attitude_tracking(results, title_str)
%PLOT_ATTITUDE_TRACKING Plot attitude tracking performance
%   fig = plot_attitude_tracking(results)
%   fig = plot_attitude_tracking(results, title_str)
%
%   Inputs:
%       results   - Structure with fields:
%                   .time      - Time vector [s]
%                   .reference - Reference angles [3 x N] [rad]
%                   .output    - Measured angles [3 x N] [rad]
%                   .control   - Control torques [3 x N] [N*m]
%                   .error     - Tracking errors [3 x N] [rad]
%       title_str - Plot title (optional)
%
%   Outputs:
%       fig - Figure handle

    if nargin < 2
        title_str = 'Attitude Tracking Performance';
    end

    % Extract data
    t = results.time;
    ref = rad2deg(results.reference);  % Convert to degrees
    y = rad2deg(results.output);
    u = results.control;

    if isfield(results, 'error')
        e = rad2deg(results.error);
    else
        e = ref - y;
    end

    % Create figure
    fig = figure('Name', title_str, 'Position', [100 100 1200 800]);

    % Colors
    colors = lines(4);

    %% Roll tracking
    subplot(3, 3, 1);
    plot(t, ref(1, :), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    hold on;
    plot(t, y(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
    hold off;
    xlabel('Time [s]');
    ylabel('Roll [deg]');
    title('Roll Tracking');
    legend('Location', 'best');
    grid on;

    %% Pitch tracking
    subplot(3, 3, 2);
    plot(t, ref(2, :), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    hold on;
    plot(t, y(2, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
    hold off;
    xlabel('Time [s]');
    ylabel('Pitch [deg]');
    title('Pitch Tracking');
    legend('Location', 'best');
    grid on;

    %% Yaw tracking
    subplot(3, 3, 3);
    plot(t, ref(3, :), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    hold on;
    plot(t, y(3, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
    hold off;
    xlabel('Time [s]');
    ylabel('Yaw [deg]');
    title('Yaw Tracking');
    legend('Location', 'best');
    grid on;

    %% Tracking errors
    subplot(3, 3, 4);
    plot(t, e(1, :), 'r-', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Error [deg]');
    title('Roll Error');
    grid on;

    subplot(3, 3, 5);
    plot(t, e(2, :), 'g-', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Error [deg]');
    title('Pitch Error');
    grid on;

    subplot(3, 3, 6);
    plot(t, e(3, :), 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Error [deg]');
    title('Yaw Error');
    grid on;

    %% Control inputs
    subplot(3, 3, 7);
    plot(t, u(1, :), 'r-', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('\tau_\phi [N\cdotm]');
    title('Roll Torque');
    grid on;

    subplot(3, 3, 8);
    plot(t, u(2, :), 'g-', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('\tau_\theta [N\cdotm]');
    title('Pitch Torque');
    grid on;

    subplot(3, 3, 9);
    plot(t, u(3, :), 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('\tau_\psi [N\cdotm]');
    title('Yaw Torque');
    grid on;

    sgtitle(title_str, 'FontSize', 14, 'FontWeight', 'bold');

end


function fig = plot_controller_comparison(results_cell, controller_names, title_str)
%PLOT_CONTROLLER_COMPARISON Compare multiple controllers
%   fig = plot_controller_comparison(results_cell, controller_names)
%
%   Inputs:
%       results_cell    - Cell array of result structures
%       controller_names- Cell array of controller names
%       title_str       - Plot title (optional)

    if nargin < 3
        title_str = 'Controller Comparison';
    end

    n_ctrl = length(results_cell);
    colors = lines(n_ctrl);

    fig = figure('Name', title_str, 'Position', [100 100 1400 600]);

    %% Roll comparison
    subplot(2, 3, 1);
    for i = 1:n_ctrl
        t = results_cell{i}.time;
        y = rad2deg(results_cell{i}.output(1, :));
        plot(t, y, '-', 'Color', colors(i, :), 'LineWidth', 1.5, ...
            'DisplayName', controller_names{i});
        hold on;
    end
    % Plot reference
    ref = rad2deg(results_cell{1}.reference(1, :));
    plot(t, ref, 'k--', 'LineWidth', 1, 'DisplayName', 'Reference');
    hold off;
    xlabel('Time [s]');
    ylabel('Roll [deg]');
    title('Roll Tracking');
    legend('Location', 'best');
    grid on;

    %% Pitch comparison
    subplot(2, 3, 2);
    for i = 1:n_ctrl
        t = results_cell{i}.time;
        y = rad2deg(results_cell{i}.output(2, :));
        plot(t, y, '-', 'Color', colors(i, :), 'LineWidth', 1.5, ...
            'DisplayName', controller_names{i});
        hold on;
    end
    ref = rad2deg(results_cell{1}.reference(2, :));
    plot(t, ref, 'k--', 'LineWidth', 1, 'DisplayName', 'Reference');
    hold off;
    xlabel('Time [s]');
    ylabel('Pitch [deg]');
    title('Pitch Tracking');
    legend('Location', 'best');
    grid on;

    %% Yaw comparison
    subplot(2, 3, 3);
    for i = 1:n_ctrl
        t = results_cell{i}.time;
        y = rad2deg(results_cell{i}.output(3, :));
        plot(t, y, '-', 'Color', colors(i, :), 'LineWidth', 1.5, ...
            'DisplayName', controller_names{i});
        hold on;
    end
    ref = rad2deg(results_cell{1}.reference(3, :));
    plot(t, ref, 'k--', 'LineWidth', 1, 'DisplayName', 'Reference');
    hold off;
    xlabel('Time [s]');
    ylabel('Yaw [deg]');
    title('Yaw Tracking');
    legend('Location', 'best');
    grid on;

    %% Error comparison (absolute)
    subplot(2, 3, 4);
    for i = 1:n_ctrl
        t = results_cell{i}.time;
        e = rad2deg(abs(results_cell{i}.error(1, :)));
        plot(t, e, '-', 'Color', colors(i, :), 'LineWidth', 1.5, ...
            'DisplayName', controller_names{i});
        hold on;
    end
    hold off;
    xlabel('Time [s]');
    ylabel('|Error| [deg]');
    title('Roll Absolute Error');
    legend('Location', 'best');
    grid on;

    subplot(2, 3, 5);
    for i = 1:n_ctrl
        t = results_cell{i}.time;
        e = rad2deg(abs(results_cell{i}.error(2, :)));
        plot(t, e, '-', 'Color', colors(i, :), 'LineWidth', 1.5);
        hold on;
    end
    hold off;
    xlabel('Time [s]');
    ylabel('|Error| [deg]');
    title('Pitch Absolute Error');
    grid on;

    subplot(2, 3, 6);
    for i = 1:n_ctrl
        t = results_cell{i}.time;
        e = rad2deg(abs(results_cell{i}.error(3, :)));
        plot(t, e, '-', 'Color', colors(i, :), 'LineWidth', 1.5);
        hold on;
    end
    hold off;
    xlabel('Time [s]');
    ylabel('|Error| [deg]');
    title('Yaw Absolute Error');
    grid on;

    sgtitle(title_str, 'FontSize', 14, 'FontWeight', 'bold');

end


function fig = plot_performance_bar(metrics_cell, controller_names)
%PLOT_PERFORMANCE_BAR Bar chart of performance metrics
%   fig = plot_performance_bar(metrics_cell, controller_names)

    n_ctrl = length(metrics_cell);

    % Extract metrics
    IAE_roll = zeros(n_ctrl, 1);
    IAE_pitch = zeros(n_ctrl, 1);
    IAE_yaw = zeros(n_ctrl, 1);
    RMSE_roll = zeros(n_ctrl, 1);
    RMSE_pitch = zeros(n_ctrl, 1);
    RMSE_yaw = zeros(n_ctrl, 1);

    for i = 1:n_ctrl
        m = metrics_cell{i};
        IAE_roll(i) = m.IAE_roll;
        IAE_pitch(i) = m.IAE_pitch;
        IAE_yaw(i) = m.IAE_yaw;
        RMSE_roll(i) = m.RMSE_roll;
        RMSE_pitch(i) = m.RMSE_pitch;
        RMSE_yaw(i) = m.RMSE_yaw;
    end

    fig = figure('Name', 'Performance Metrics', 'Position', [100 100 1000 500]);

    %% IAE Bar Chart
    subplot(1, 2, 1);
    IAE_data = [IAE_roll, IAE_pitch, IAE_yaw];
    bar(IAE_data);
    set(gca, 'XTickLabel', controller_names);
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'northwest');
    ylabel('IAE [deg\cdots]');
    title('Integral Absolute Error');
    grid on;

    %% RMSE Bar Chart
    subplot(1, 2, 2);
    RMSE_data = [RMSE_roll, RMSE_pitch, RMSE_yaw];
    bar(RMSE_data);
    set(gca, 'XTickLabel', controller_names);
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'northwest');
    ylabel('RMSE [deg]');
    title('Root Mean Square Error');
    grid on;

    sgtitle('Controller Performance Comparison', 'FontSize', 14, 'FontWeight', 'bold');

end
