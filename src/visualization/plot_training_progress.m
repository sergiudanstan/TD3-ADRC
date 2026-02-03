function fig = plot_training_progress(training_log, title_str)
%PLOT_TRAINING_PROGRESS Plot TD3 training progress
%   fig = plot_training_progress(training_log)
%   fig = plot_training_progress(training_log, title_str)
%
%   Inputs:
%       training_log - Structure with fields:
%                      .episode_rewards - Rewards per episode
%                      .critic_loss     - Critic loss per update
%                      .actor_loss      - Actor loss per update
%       title_str    - Plot title (optional)
%
%   Outputs:
%       fig - Figure handle

    if nargin < 2
        title_str = 'TD3 Training Progress';
    end

    fig = figure('Name', title_str, 'Position', [100 100 1200 600]);

    %% Episode rewards
    subplot(2, 2, 1);
    rewards = training_log.episode_rewards;
    n_ep = length(rewards);

    plot(1:n_ep, rewards, 'b-', 'LineWidth', 0.5, 'DisplayName', 'Episode Reward');
    hold on;

    % Moving average
    if n_ep > 20
        window = 20;
        ma = movmean(rewards, window);
        plot(1:n_ep, ma, 'r-', 'LineWidth', 2, 'DisplayName', sprintf('%d-Episode MA', window));
    end

    hold off;
    xlabel('Episode');
    ylabel('Episode Reward');
    title('Training Rewards');
    legend('Location', 'southeast');
    grid on;

    %% Critic loss
    subplot(2, 2, 2);
    critic_loss = training_log.critic_loss;
    n_updates = length(critic_loss);

    % Subsample for plotting if too many points
    if n_updates > 10000
        idx = 1:10:n_updates;
        plot(idx, critic_loss(idx), 'r-', 'LineWidth', 0.5);
    else
        plot(1:n_updates, critic_loss, 'r-', 'LineWidth', 0.5);
    end

    xlabel('Update Step');
    ylabel('Critic Loss (MSE)');
    title('Critic Loss');
    grid on;

    % Add log scale option
    if max(critic_loss) / min(critic_loss(critic_loss > 0)) > 100
        set(gca, 'YScale', 'log');
    end

    %% Actor loss
    subplot(2, 2, 3);
    actor_loss = training_log.actor_loss;
    n_actor = length(actor_loss);

    if n_actor > 0
        if n_actor > 5000
            idx = 1:5:n_actor;
            plot(idx, actor_loss(idx), 'g-', 'LineWidth', 0.5);
        else
            plot(1:n_actor, actor_loss, 'g-', 'LineWidth', 0.5);
        end
    end

    xlabel('Policy Update');
    ylabel('Actor Loss (-Q value)');
    title('Actor Loss');
    grid on;

    %% Reward distribution
    subplot(2, 2, 4);
    if n_ep > 50
        histogram(rewards, 50, 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    else
        histogram(rewards, 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    end

    xlabel('Episode Reward');
    ylabel('Count');
    title('Reward Distribution');
    grid on;

    % Add statistics text
    stats_str = sprintf('Mean: %.2f\nStd: %.2f\nMax: %.2f', ...
        mean(rewards), std(rewards), max(rewards));
    text(0.95, 0.95, stats_str, 'Units', 'normalized', ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
        'BackgroundColor', 'white', 'EdgeColor', 'black');

    sgtitle(title_str, 'FontSize', 14, 'FontWeight', 'bold');

end


function fig = plot_td3_action_evolution(log, title_str)
%PLOT_TD3_ACTION_EVOLUTION Plot how TD3 actions evolve over time
%   fig = plot_td3_action_evolution(log)
%
%   Shows how the TD3 agent adapts ADRC parameters during operation

    if nargin < 2
        title_str = 'TD3 Parameter Adaptation';
    end

    t = log.time;
    actions = log.td3_action;

    if isempty(actions)
        warning('No TD3 actions logged.');
        fig = [];
        return;
    end

    fig = figure('Name', title_str, 'Position', [100 100 1000 600]);

    subplot(3, 1, 1);
    plot(t, actions(1, :), 'r-', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('\omega_{c,roll} [rad/s]');
    title('Roll Controller Bandwidth');
    grid on;

    subplot(3, 1, 2);
    plot(t, actions(2, :), 'g-', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('\omega_{c,pitch} [rad/s]');
    title('Pitch Controller Bandwidth');
    grid on;

    subplot(3, 1, 3);
    plot(t, actions(3, :), 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('\omega_{c,yaw} [rad/s]');
    title('Yaw Controller Bandwidth');
    grid on;

    sgtitle(title_str, 'FontSize', 14, 'FontWeight', 'bold');

end


function fig = plot_disturbance_estimation(log, true_disturbance, title_str)
%PLOT_DISTURBANCE_ESTIMATION Compare estimated vs true disturbance
%   fig = plot_disturbance_estimation(log, true_disturbance)
%
%   Inputs:
%       log             - Controller log with .disturbance field
%       true_disturbance- True disturbance [6 x N]

    if nargin < 3
        title_str = 'Disturbance Estimation';
    end

    t = log.time;
    z3 = log.disturbance;  % ESO estimated disturbance

    fig = figure('Name', title_str, 'Position', [100 100 1000 600]);

    axis_names = {'Roll', 'Pitch', 'Yaw'};

    for i = 1:3
        subplot(3, 1, i);
        plot(t, z3(i, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Estimated (ESO)');

        if nargin >= 2 && ~isempty(true_disturbance)
            hold on;
            % True disturbance torque is indices 4,5,6
            plot(t, true_disturbance(i+3, :), 'r--', 'LineWidth', 1.5, 'DisplayName', 'True');
            hold off;
            legend('Location', 'best');
        end

        xlabel('Time [s]');
        ylabel(sprintf('%s Disturbance', axis_names{i}));
        title(sprintf('%s Axis Disturbance', axis_names{i}));
        grid on;
    end

    sgtitle(title_str, 'FontSize', 14, 'FontWeight', 'bold');

end


function generate_performance_table(metrics_cell, controller_names, save_path)
%GENERATE_PERFORMANCE_TABLE Generate performance comparison table
%   generate_performance_table(metrics_cell, controller_names)
%   generate_performance_table(metrics_cell, controller_names, save_path)

    n_ctrl = length(metrics_cell);

    % Create table data
    Roll_IAE = zeros(n_ctrl, 1);
    Pitch_IAE = zeros(n_ctrl, 1);
    Yaw_IAE = zeros(n_ctrl, 1);
    Total_IAE = zeros(n_ctrl, 1);

    for i = 1:n_ctrl
        m = metrics_cell{i};
        Roll_IAE(i) = m.IAE_roll;
        Pitch_IAE(i) = m.IAE_pitch;
        Yaw_IAE(i) = m.IAE_yaw;
        Total_IAE(i) = m.IAE_total;
    end

    % Calculate improvement over ADRC (assuming index 2 is ADRC)
    if n_ctrl >= 2
        Improvement = (Total_IAE(2) - Total_IAE) / Total_IAE(2) * 100;
    else
        Improvement = zeros(n_ctrl, 1);
    end

    % Create table
    T = table(Roll_IAE, Pitch_IAE, Yaw_IAE, Total_IAE, Improvement, ...
        'RowNames', controller_names);

    % Display
    fprintf('\n========== Performance Comparison ==========\n');
    disp(T);

    % Save if path provided
    if nargin >= 3 && ~isempty(save_path)
        writetable(T, save_path, 'WriteRowNames', true);
        fprintf('Table saved to: %s\n', save_path);
    end

end
