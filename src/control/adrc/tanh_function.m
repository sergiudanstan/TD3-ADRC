function y = tanh_function(e, alpha, delta)
%TANH_FUNCTION Novel parameterized tanh function for ADRC
%   Replaces the discontinuous fal function with a smooth, continuous alternative
%
%   y = tanh_function(e, alpha, delta)
%
%   Formula:
%       y = -(delta^(alpha*e) - delta^(-alpha*e)) / (delta^(alpha*e) + delta^(-alpha*e))
%
%   Properties:
%       - Continuous and smooth everywhere (C-infinity)
%       - Bounded output: y ∈ (-1, 1)
%       - Odd function: tanh_function(-e) = -tanh_function(e)
%       - Higher gain for small errors, saturates for large errors
%       - Shape controlled by alpha and delta parameters
%
%   Parameters:
%       e     - Error signal (scalar or array)
%       alpha - Nonlinearity shaping parameter (typical: 0.5-2.0)
%               Higher alpha = steeper transition
%       delta - Smoothness parameter (0 < delta < 1, typical: 0.3-0.7)
%               Smaller delta = more aggressive nonlinearity
%
%   Inputs:
%       e     - Error signal
%       alpha - Shaping parameter (default: 1.0)
%       delta - Smoothness parameter (default: 0.5)
%
%   Outputs:
%       y     - Nonlinear function output
%
%   Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"
%
%   Example:
%       e = linspace(-5, 5, 1000);
%       y1 = tanh_function(e, 1.0, 0.5);
%       y2 = tanh_function(e, 1.5, 0.3);
%       plot(e, y1, e, y2);
%       legend('alpha=1.0, delta=0.5', 'alpha=1.5, delta=0.3');

    % Default parameters
    if nargin < 2 || isempty(alpha)
        alpha = 1.0;
    end
    if nargin < 3 || isempty(delta)
        delta = 0.5;
    end

    % Validate delta (must be in (0, 1) for proper behavior)
    if delta <= 0 || delta >= 1
        warning('tanh_function: delta should be in (0, 1). Using delta = 0.5');
        delta = 0.5;
    end

    % Compute the function
    % y = -(delta^(alpha*e) - delta^(-alpha*e)) / (delta^(alpha*e) + delta^(-alpha*e))
    %
    % This is equivalent to: y = -tanh(alpha * e * log(delta))
    % But we compute it directly for numerical stability

    % For numerical stability with large |e|, use the equivalent form
    % Rewrite as: y = -tanh(alpha * e * ln(delta))
    % Since ln(delta) < 0 for 0 < delta < 1, and tanh is odd:
    % y = tanh(alpha * e * |ln(delta)|)

    ln_delta = log(delta);  % This is negative for 0 < delta < 1

    % Compute argument
    arg = alpha * e * ln_delta;

    % Compute using exponentials with overflow protection
    y = zeros(size(e));

    for i = 1:numel(e)
        a = arg(i);

        if a > 700  % Overflow protection
            y(i) = -1;
        elseif a < -700
            y(i) = 1;
        else
            exp_pos = exp(a);
            exp_neg = exp(-a);
            y(i) = -(exp_pos - exp_neg) / (exp_pos + exp_neg);
        end
    end

    % Alternative (simpler) computation using MATLAB's tanh:
    % y = tanh(-alpha * e * log(delta));
    % This works because ln(delta) < 0, so -ln(delta) > 0

end


function demo_tanh_function()
%DEMO_TANH_FUNCTION Demonstrate the tanh function properties
%   Compare with traditional fal function

    % Error range
    e = linspace(-5, 5, 1000);

    % Novel tanh function with different parameters
    y_tanh1 = tanh_function(e, 1.0, 0.5);
    y_tanh2 = tanh_function(e, 1.5, 0.3);
    y_tanh3 = tanh_function(e, 0.5, 0.7);

    % Traditional fal function for comparison
    alpha_fal = 0.75;
    delta_fal = 0.1;
    y_fal = arrayfun(@(x) fal_func(x, alpha_fal, delta_fal), e);

    % Standard tanh for reference
    y_std_tanh = tanh(e);

    % Plot comparison
    figure('Name', 'Novel tanh Function vs Traditional fal');

    subplot(2, 1, 1);
    plot(e, y_tanh1, 'b-', 'LineWidth', 2, 'DisplayName', 'tanh (α=1.0, δ=0.5)');
    hold on;
    plot(e, y_tanh2, 'r--', 'LineWidth', 2, 'DisplayName', 'tanh (α=1.5, δ=0.3)');
    plot(e, y_tanh3, 'g-.', 'LineWidth', 2, 'DisplayName', 'tanh (α=0.5, δ=0.7)');
    plot(e, y_std_tanh, 'k:', 'LineWidth', 1, 'DisplayName', 'standard tanh');
    hold off;
    xlabel('Error e');
    ylabel('Output y');
    title('Novel Parameterized tanh Function');
    legend('Location', 'southeast');
    grid on;
    xlim([-5, 5]);
    ylim([-1.5, 1.5]);

    subplot(2, 1, 2);
    plot(e, y_tanh1, 'b-', 'LineWidth', 2, 'DisplayName', 'Novel tanh');
    hold on;
    plot(e, y_fal, 'r--', 'LineWidth', 2, 'DisplayName', 'Traditional fal');
    hold off;
    xlabel('Error e');
    ylabel('Output y');
    title('Comparison: Novel tanh vs Traditional fal');
    legend('Location', 'southeast');
    grid on;
    xlim([-5, 5]);

    % Derivative comparison (smoothness)
    figure('Name', 'Smoothness Comparison');

    dy_tanh = gradient(y_tanh1, e);
    dy_fal = gradient(y_fal, e);

    plot(e, dy_tanh, 'b-', 'LineWidth', 2, 'DisplayName', 'Novel tanh derivative');
    hold on;
    plot(e, dy_fal, 'r--', 'LineWidth', 2, 'DisplayName', 'fal derivative');
    hold off;
    xlabel('Error e');
    ylabel('dy/de');
    title('Derivative Comparison (Smoothness)');
    legend('Location', 'northeast');
    grid on;
    xlim([-2, 2]);

    fprintf('Novel tanh function is smooth (C-infinity) everywhere.\n');
    fprintf('Traditional fal function has discontinuous derivative at |e| = delta.\n');

end


function y = fal_func(e, alpha, delta)
    %FAL_FUNC Traditional fal function for comparison
    if abs(e) <= delta
        y = e / delta^(1 - alpha);
    else
        y = abs(e)^alpha * sign(e);
    end
end
