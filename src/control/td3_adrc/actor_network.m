classdef actor_network < handle
%ACTOR_NETWORK Policy network for TD3
%   Maps states to deterministic actions (bounded by tanh)
%
%   Architecture: state -> FC -> ReLU -> FC -> ReLU -> FC -> tanh -> action
%
%   Reference: Fujimoto et al. (2018) "Addressing Function Approximation
%              Error in Actor-Critic Methods"

    properties
        % Network parameters (weights and biases)
        W1, b1      % First hidden layer
        W2, b2      % Second hidden layer
        W3, b3      % Output layer

        % Network dimensions
        state_dim
        action_dim
        hidden_dims

        % Learning parameters
        learning_rate
        l2_reg

        % Action bounds
        action_low
        action_high
    end

    methods
        function obj = actor_network(state_dim, action_dim, hidden_dims, action_bounds)
            %ACTOR_NETWORK Construct actor network
            %   actor = actor_network(state_dim, action_dim, hidden_dims, action_bounds)
            %
            %   Inputs:
            %       state_dim    - Dimension of state vector
            %       action_dim   - Dimension of action vector
            %       hidden_dims  - Hidden layer sizes [h1, h2] (default: [256, 256])
            %       action_bounds- Action bounds [low; high] (n_actions x 2)

            if nargin < 3 || isempty(hidden_dims)
                hidden_dims = [256, 256];
            end
            if nargin < 4 || isempty(action_bounds)
                action_bounds = repmat([-1, 1], action_dim, 1);
            end

            obj.state_dim = state_dim;
            obj.action_dim = action_dim;
            obj.hidden_dims = hidden_dims;
            obj.action_low = action_bounds(:, 1);
            obj.action_high = action_bounds(:, 2);

            obj.learning_rate = 1e-4;
            obj.l2_reg = 1e-4;

            % Initialize weights using Xavier/He initialization
            obj.initialize_weights();
        end

        function initialize_weights(obj)
            %INITIALIZE_WEIGHTS Initialize network weights
            %   Uses He initialization for ReLU activations

            h1 = obj.hidden_dims(1);
            h2 = obj.hidden_dims(2);

            % He initialization: std = sqrt(2/n_in)
            obj.W1 = randn(obj.state_dim, h1) * sqrt(2 / obj.state_dim);
            obj.b1 = zeros(1, h1);

            obj.W2 = randn(h1, h2) * sqrt(2 / h1);
            obj.b2 = zeros(1, h2);

            % Output layer: smaller initialization for stability
            obj.W3 = randn(h2, obj.action_dim) * 0.003;
            obj.b3 = zeros(1, obj.action_dim);
        end

        function actions = forward(obj, states)
            %FORWARD Forward pass through the network
            %   actions = actor.forward(states)
            %
            %   Inputs:
            %       states  - State matrix (batch_size x state_dim)
            %
            %   Outputs:
            %       actions - Action matrix (batch_size x action_dim)

            % First hidden layer
            z1 = states * obj.W1 + obj.b1;
            a1 = max(0, z1);  % ReLU

            % Second hidden layer
            z2 = a1 * obj.W2 + obj.b2;
            a2 = max(0, z2);  % ReLU

            % Output layer with tanh (bounded to [-1, 1])
            z3 = a2 * obj.W3 + obj.b3;
            a3 = tanh(z3);

            % Scale to action bounds
            actions = obj.scale_action(a3);
        end

        function action = select_action(obj, state, noise_std)
            %SELECT_ACTION Select action for given state (with optional exploration noise)
            %   action = actor.select_action(state) deterministic action
            %   action = actor.select_action(state, noise_std) with Gaussian noise
            %
            %   Inputs:
            %       state     - State vector (1 x state_dim or state_dim x 1)
            %       noise_std - Standard deviation of exploration noise (optional)
            %
            %   Outputs:
            %       action    - Action vector (action_dim x 1)

            state = state(:)';  % Ensure row vector
            action = obj.forward(state)';  % Column vector

            if nargin >= 3 && noise_std > 0
                % Add exploration noise
                noise = noise_std * randn(obj.action_dim, 1);
                action = action + noise;

                % Clip to bounds
                action = max(obj.action_low, min(obj.action_high, action));
            end
        end

        function [grads, loss] = compute_gradients(obj, states, critic)
            %COMPUTE_GRADIENTS Compute policy gradient using critic
            %   [grads, loss] = actor.compute_gradients(states, critic)
            %
            %   Policy gradient: maximize Q(s, actor(s))
            %
            %   Inputs:
            %       states - Batch of states (batch_size x state_dim)
            %       critic - Critic network (uses Q1 for policy update)
            %
            %   Outputs:
            %       grads  - Structure with gradients for each parameter
            %       loss   - Mean Q-value (negative of actual loss)

            batch_size = size(states, 1);

            % Forward pass (store activations for backprop)
            z1 = states * obj.W1 + obj.b1;
            a1 = max(0, z1);

            z2 = a1 * obj.W2 + obj.b2;
            a2 = max(0, z2);

            z3 = a2 * obj.W3 + obj.b3;
            a3 = tanh(z3);
            actions = obj.scale_action(a3);

            % Get Q-value and its gradient w.r.t. actions
            [Q1, dQ_da] = critic.forward_with_action_grad(states, actions);
            loss = -mean(Q1);  % Maximize Q

            % Backpropagation through actor
            % Scale gradient from action space to tanh space
            action_scale = (obj.action_high - obj.action_low)' / 2;
            dL_da3 = -dQ_da .* action_scale / batch_size;  % Negative for maximization

            % Gradient through tanh: d(tanh)/dz = 1 - tanh^2
            dL_dz3 = dL_da3 .* (1 - a3.^2);

            % Output layer gradients
            grads.W3 = a2' * dL_dz3 + obj.l2_reg * obj.W3;
            grads.b3 = sum(dL_dz3, 1);

            % Hidden layer 2
            dL_da2 = dL_dz3 * obj.W3';
            dL_dz2 = dL_da2 .* (z2 > 0);  % ReLU gradient

            grads.W2 = a1' * dL_dz2 + obj.l2_reg * obj.W2;
            grads.b2 = sum(dL_dz2, 1);

            % Hidden layer 1
            dL_da1 = dL_dz2 * obj.W2';
            dL_dz1 = dL_da1 .* (z1 > 0);  % ReLU gradient

            grads.W1 = states' * dL_dz1 + obj.l2_reg * obj.W1;
            grads.b1 = sum(dL_dz1, 1);
        end

        function update(obj, grads)
            %UPDATE Update network parameters using gradients
            %   actor.update(grads)

            obj.W1 = obj.W1 - obj.learning_rate * grads.W1;
            obj.b1 = obj.b1 - obj.learning_rate * grads.b1;
            obj.W2 = obj.W2 - obj.learning_rate * grads.W2;
            obj.b2 = obj.b2 - obj.learning_rate * grads.b2;
            obj.W3 = obj.W3 - obj.learning_rate * grads.W3;
            obj.b3 = obj.b3 - obj.learning_rate * grads.b3;
        end

        function copy_from(obj, source_actor, tau)
            %COPY_FROM Soft copy parameters from another actor
            %   actor.copy_from(source, tau) soft update with tau
            %   actor.copy_from(source, 1) hard copy

            if nargin < 3
                tau = 1;  % Hard copy
            end

            obj.W1 = tau * source_actor.W1 + (1 - tau) * obj.W1;
            obj.b1 = tau * source_actor.b1 + (1 - tau) * obj.b1;
            obj.W2 = tau * source_actor.W2 + (1 - tau) * obj.W2;
            obj.b2 = tau * source_actor.b2 + (1 - tau) * obj.b2;
            obj.W3 = tau * source_actor.W3 + (1 - tau) * obj.W3;
            obj.b3 = tau * source_actor.b3 + (1 - tau) * obj.b3;
        end

        function actions_scaled = scale_action(obj, actions_normalized)
            %SCALE_ACTION Scale from [-1, 1] to action bounds
            % actions_scaled = low + (actions_normalized + 1) * (high - low) / 2

            action_range = (obj.action_high - obj.action_low)' / 2;
            action_mean = (obj.action_high + obj.action_low)' / 2;
            actions_scaled = actions_normalized .* action_range + action_mean;
        end

        function actions_normalized = normalize_action(obj, actions_scaled)
            %NORMALIZE_ACTION Scale from action bounds to [-1, 1]
            action_range = (obj.action_high - obj.action_low)' / 2;
            action_mean = (obj.action_high + obj.action_low)' / 2;
            actions_normalized = (actions_scaled - action_mean) ./ action_range;
        end

        function save(obj, filename)
            %SAVE Save network to file
            actor_data.W1 = obj.W1;
            actor_data.b1 = obj.b1;
            actor_data.W2 = obj.W2;
            actor_data.b2 = obj.b2;
            actor_data.W3 = obj.W3;
            actor_data.b3 = obj.b3;
            actor_data.state_dim = obj.state_dim;
            actor_data.action_dim = obj.action_dim;
            actor_data.hidden_dims = obj.hidden_dims;
            actor_data.action_low = obj.action_low;
            actor_data.action_high = obj.action_high;

            save(filename, 'actor_data');
        end

        function load(obj, filename)
            %LOAD Load network from file
            data = load(filename);

            obj.W1 = data.actor_data.W1;
            obj.b1 = data.actor_data.b1;
            obj.W2 = data.actor_data.W2;
            obj.b2 = data.actor_data.b2;
            obj.W3 = data.actor_data.W3;
            obj.b3 = data.actor_data.b3;
        end
    end

end
