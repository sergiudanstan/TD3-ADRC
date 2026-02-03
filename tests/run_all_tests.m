%% run_all_tests.m - Run all TD3-ADRC unit tests
% Validates core functionality of all components
%
% Usage:
%   run_all_tests        - Run all tests
%   run_all_tests(true)  - Run with verbose output

function results = run_all_tests(verbose)
    if nargin < 1
        verbose = false;
    end

    fprintf('=========================================\n');
    fprintf('    TD3-ADRC Unit Tests\n');
    fprintf('=========================================\n\n');

    % Setup paths
    setup_paths;

    % Track results
    tests_passed = 0;
    tests_failed = 0;
    test_results = {};

    %% Test 1: Tracking Differentiator
    fprintf('Test 1: Tracking Differentiator...\n');
    try
        td = tracking_differentiator(100, 0.01);

        % Test step response
        for i = 1:100
            [v1, v2] = td.update(1.0, 0.01);
        end

        assert(abs(v1 - 1.0) < 0.1, 'TD should track step input');
        assert(abs(v2) < 1.0, 'TD derivative should be small at steady state');

        tests_passed = tests_passed + 1;
        fprintf('  PASSED\n');
        test_results{end+1} = {'Tracking Differentiator', true, ''};
    catch ME
        tests_failed = tests_failed + 1;
        fprintf('  FAILED: %s\n', ME.message);
        test_results{end+1} = {'Tracking Differentiator', false, ME.message};
    end

    %% Test 2: Extended State Observer
    fprintf('Test 2: Extended State Observer...\n');
    try
        eso = extended_state_observer(50, 1.0, 0.01);

        % Test with known dynamics
        y = 0; u = 1;
        for i = 1:200
            y = y + 0.01 * u;  % Simple integrator
            [z1, z2, z3] = eso.update(y, u, 0.01);
        end

        assert(abs(z1 - y) < 0.1, 'ESO should track output');
        assert(abs(z2 - u) < 0.5, 'ESO should estimate velocity');

        tests_passed = tests_passed + 1;
        fprintf('  PASSED\n');
        test_results{end+1} = {'Extended State Observer', true, ''};
    catch ME
        tests_failed = tests_failed + 1;
        fprintf('  FAILED: %s\n', ME.message);
        test_results{end+1} = {'Extended State Observer', false, ME.message};
    end

    %% Test 3: Nonlinear Feedback
    fprintf('Test 3: Nonlinear Feedback...\n');
    try
        nlsef = nonlinear_feedback(10, 1.0, 0.01);

        % Test with zero error
        [u, u0] = nlsef.compute(0, 0, 0);
        assert(abs(u) < 0.01, 'Control should be zero for zero error');

        % Test with error
        [u, u0] = nlsef.compute(0.1, 0, 0);
        assert(u ~= 0, 'Control should be non-zero for error');

        tests_passed = tests_passed + 1;
        fprintf('  PASSED\n');
        test_results{end+1} = {'Nonlinear Feedback', true, ''};
    catch ME
        tests_failed = tests_failed + 1;
        fprintf('  FAILED: %s\n', ME.message);
        test_results{end+1} = {'Nonlinear Feedback', false, ME.message};
    end

    %% Test 4: Novel tanh Function
    fprintf('Test 4: Novel tanh Function...\n');
    try
        % Test basic properties
        y0 = tanh_function(0, 1.0, 0.5);
        assert(abs(y0) < 0.01, 'tanh(0) should be ~0');

        y_pos = tanh_function(1, 1.0, 0.5);
        y_neg = tanh_function(-1, 1.0, 0.5);
        assert(abs(y_pos + y_neg) < 0.01, 'tanh should be odd function');

        % Test boundedness
        y_large = tanh_function(100, 1.0, 0.5);
        assert(abs(y_large) <= 1.01, 'tanh output should be bounded');

        tests_passed = tests_passed + 1;
        fprintf('  PASSED\n');
        test_results{end+1} = {'Novel tanh Function', true, ''};
    catch ME
        tests_failed = tests_failed + 1;
        fprintf('  FAILED: %s\n', ME.message);
        test_results{end+1} = {'Novel tanh Function', false, ME.message};
    end

    %% Test 5: Quadrotor Parameters
    fprintf('Test 5: Quadrotor Parameters...\n');
    try
        params = quadrotor_params();

        assert(params.m > 0, 'Mass should be positive');
        assert(all(diag(params.I) > 0), 'Inertia should be positive');
        assert(params.kf > 0, 'Thrust coefficient should be positive');
        assert(size(params.B, 1) == 4, 'Mixer matrix should be 4x4');

        tests_passed = tests_passed + 1;
        fprintf('  PASSED\n');
        test_results{end+1} = {'Quadrotor Parameters', true, ''};
    catch ME
        tests_failed = tests_failed + 1;
        fprintf('  FAILED: %s\n', ME.message);
        test_results{end+1} = {'Quadrotor Parameters', false, ME.message};
    end

    %% Test 6: Replay Buffer
    fprintf('Test 6: Replay Buffer...\n');
    try
        buffer = replay_buffer(1000, 6, 3);

        % Add samples
        for i = 1:100
            buffer.add(randn(6,1), randn(3,1), randn, randn(6,1), false);
        end

        assert(buffer.get_size() == 100, 'Buffer size should be 100');

        % Sample batch
        [s, a, r, ns, d] = buffer.sample(32);
        assert(size(s, 1) == 32, 'Batch size should be 32');
        assert(size(a, 2) == 3, 'Action dim should be 3');

        tests_passed = tests_passed + 1;
        fprintf('  PASSED\n');
        test_results{end+1} = {'Replay Buffer', true, ''};
    catch ME
        tests_failed = tests_failed + 1;
        fprintf('  FAILED: %s\n', ME.message);
        test_results{end+1} = {'Replay Buffer', false, ME.message};
    end

    %% Test 7: Actor Network
    fprintf('Test 7: Actor Network...\n');
    try
        action_bounds = [5, 30; 5, 30; 2, 15];
        actor = actor_network(6, 3, [64, 64], action_bounds);

        % Forward pass
        states = randn(10, 6);
        actions = actor.forward(states);

        assert(size(actions, 1) == 10, 'Batch size should match');
        assert(size(actions, 2) == 3, 'Action dim should be 3');
        assert(all(actions(:) >= 2), 'Actions should be above min bound');
        assert(all(actions(:) <= 30), 'Actions should be below max bound');

        tests_passed = tests_passed + 1;
        fprintf('  PASSED\n');
        test_results{end+1} = {'Actor Network', true, ''};
    catch ME
        tests_failed = tests_failed + 1;
        fprintf('  FAILED: %s\n', ME.message);
        test_results{end+1} = {'Actor Network', false, ME.message};
    end

    %% Test 8: Critic Network
    fprintf('Test 8: Critic Network...\n');
    try
        critic = critic_network(6, 3, [64, 64]);

        % Forward pass
        states = randn(10, 6);
        actions = randn(10, 3);
        [Q1, Q2] = critic.forward(states, actions);

        assert(length(Q1) == 10, 'Q1 should have batch size 10');
        assert(length(Q2) == 10, 'Q2 should have batch size 10');

        tests_passed = tests_passed + 1;
        fprintf('  PASSED\n');
        test_results{end+1} = {'Critic Network', true, ''};
    catch ME
        tests_failed = tests_failed + 1;
        fprintf('  FAILED: %s\n', ME.message);
        test_results{end+1} = {'Critic Network', false, ME.message};
    end

    %% Test 9: TD3-ADRC Controller
    fprintf('Test 9: TD3-ADRC Controller...\n');
    try
        ctrl = td3_adrc_controller(0.01, false);  % ADRC only

        ref = deg2rad([10; -10; -15]);
        y = deg2rad([5; -5; -10]);

        [tau, info] = ctrl.compute_control(ref, y, 0.1);

        assert(length(tau) == 3, 'Should output 3 torques');
        assert(all(isfinite(tau)), 'Torques should be finite');

        tests_passed = tests_passed + 1;
        fprintf('  PASSED\n');
        test_results{end+1} = {'TD3-ADRC Controller', true, ''};
    catch ME
        tests_failed = tests_failed + 1;
        fprintf('  FAILED: %s\n', ME.message);
        test_results{end+1} = {'TD3-ADRC Controller', false, ME.message};
    end

    %% Test 10: Quadrotor Dynamics
    fprintf('Test 10: Quadrotor Dynamics...\n');
    try
        params = quadrotor_params();
        x = zeros(12, 1);
        x(3) = 1;  % Initial height

        % Hover thrust
        F_hover = params.m * params.g;
        u = [F_hover; 0; 0; 0];

        [x_dot, outputs] = quadrotor_dynamics(0, x, u, params);

        % At hover, vertical acceleration should be ~0
        assert(abs(x_dot(6)) < 0.1, 'Vertical acceleration should be near zero at hover');

        tests_passed = tests_passed + 1;
        fprintf('  PASSED\n');
        test_results{end+1} = {'Quadrotor Dynamics', true, ''};
    catch ME
        tests_failed = tests_failed + 1;
        fprintf('  FAILED: %s\n', ME.message);
        test_results{end+1} = {'Quadrotor Dynamics', false, ME.message};
    end

    %% Summary
    fprintf('\n=========================================\n');
    fprintf('Test Summary\n');
    fprintf('=========================================\n');
    fprintf('Passed: %d\n', tests_passed);
    fprintf('Failed: %d\n', tests_failed);
    fprintf('Total:  %d\n', tests_passed + tests_failed);
    fprintf('=========================================\n');

    if tests_failed == 0
        fprintf('\nAll tests PASSED!\n');
    else
        fprintf('\nSome tests FAILED. See details above.\n');
    end

    % Return results structure
    results.passed = tests_passed;
    results.failed = tests_failed;
    results.details = test_results;

end
