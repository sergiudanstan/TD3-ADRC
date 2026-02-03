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
