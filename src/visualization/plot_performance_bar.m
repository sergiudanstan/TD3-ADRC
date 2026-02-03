function fig = plot_performance_bar(metrics_cell, controller_names)
%PLOT_PERFORMANCE_BAR Bar chart of performance metrics
%   fig = plot_performance_bar(metrics_cell, controller_names)
%
%   Inputs:
%       metrics_cell     - Cell array of metrics structures
%       controller_names - Cell array of controller names

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
