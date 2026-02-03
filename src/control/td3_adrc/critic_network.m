classdef critic_network < handle
%CRITIC_NETWORK Twin Q-networks for TD3
%   Estimates Q(s, a) using two independent networks to reduce overestimation
%
%   Architecture: [state, action] -> FC -> ReLU -> FC -> ReLU -> FC -> Q-value
%
%   Reference: Fujimoto et al. (2018) "Addressing Function Approximation
%              Error in Actor-Critic Methods"

    properties
        % Network 1 parameters
        W1_1, b1_1
        W2_1, b2_1
        W3_1, b3_1

        % Network 2 parameters
        W1_2, b1_2
        W2_2, b2_2
        W3_2, b3_2

        % Network dimensions
        state_dim
        action_dim
        hidden_dims

        % Learning parameters
        learning_rate
        l2_reg
    end

    methods
        function obj = critic_network(state_dim, action_dim, hidden_dims)
            %CRITIC_NETWORK Construct twin critic networks
            %   critic = critic_network(state_dim, action_dim, hidden_dims)
            %
            %   Inputs:
            %       state_dim   - Dimension of state vector
            %       action_dim  - Dimension of action vector
            %       hidden_dims - Hidden layer sizes [h1, h2] (default: [256, 256])

            if nargin < 3 || isempty(hidden_dims)
                hidden_dims = [256, 256];
            end

            obj.state_dim = state_dim;
            obj.action_dim = action_dim;
            obj.hidden_dims = hidden_dims;

            obj.learning_rate = 1e-3;
            obj.l2_reg = 1e-4;

            % Initialize both networks
            obj.initialize_weights();
        end

        function initialize_weights(obj)
            %INITIALIZE_WEIGHTS Initialize network weights for both critics

            input_dim = obj.state_dim + obj.action_dim;
            h1 = obj.hidden_dims(1);
            h2 = obj.hidden_dims(2);

            % Network 1
            obj.W1_1 = randn(input_dim, h1) * sqrt(2 / input_dim);
            obj.b1_1 = zeros(1, h1);
            obj.W2_1 = randn(h1, h2) * sqrt(2 / h1);
            obj.b2_1 = zeros(1, h2);
            obj.W3_1 = randn(h2, 1) * 0.003;
            obj.b3_1 = 0;

            % Network 2
            obj.W1_2 = randn(input_dim, h1) * sqrt(2 / input_dim);
            obj.b1_2 = zeros(1, h1);
            obj.W2_2 = randn(h1, h2) * sqrt(2 / h1);
            obj.b2_2 = zeros(1, h2);
            obj.W3_2 = randn(h2, 1) * 0.003;
            obj.b3_2 = 0;
        end

        function [Q1, Q2] = forward(obj, states, actions)
            %FORWARD Forward pass through both critic networks
            %   [Q1, Q2] = critic.forward(states, actions)
            %
            %   Inputs:
            %       states  - State matrix (batch_size x state_dim)
            %       actions - Action matrix (batch_size x action_dim)
            %
            %   Outputs:
            %       Q1 - Q-values from network 1 (batch_size x 1)
            %       Q2 - Q-values from network 2 (batch_size x 1)

            % Concatenate state and action
            x = [states, actions];

            % Network 1
            z1_1 = x * obj.W1_1 + obj.b1_1;
            a1_1 = max(0, z1_1);
            z2_1 = a1_1 * obj.W2_1 + obj.b2_1;
            a2_1 = max(0, z2_1);
            Q1 = a2_1 * obj.W3_1 + obj.b3_1;

            % Network 2
            z1_2 = x * obj.W1_2 + obj.b1_2;
            a1_2 = max(0, z1_2);
            z2_2 = a1_2 * obj.W2_2 + obj.b2_2;
            a2_2 = max(0, z2_2);
            Q2 = a2_2 * obj.W3_2 + obj.b3_2;
        end

        function Q1 = forward_Q1(obj, states, actions)
            %FORWARD_Q1 Forward pass through critic 1 only (for policy update)
            x = [states, actions];

            z1 = x * obj.W1_1 + obj.b1_1;
            a1 = max(0, z1);
            z2 = a1 * obj.W2_1 + obj.b2_1;
            a2 = max(0, z2);
            Q1 = a2 * obj.W3_1 + obj.b3_1;
        end

        function [Q1, dQ_da] = forward_with_action_grad(obj, states, actions)
            %FORWARD_WITH_ACTION_GRAD Forward Q1 and compute gradient w.r.t. actions
            %   [Q1, dQ_da] = critic.forward_with_action_grad(states, actions)
            %
            %   Used for computing actor policy gradient

            x = [states, actions];
            batch_size = size(states, 1);

            % Forward pass (store activations)
            z1 = x * obj.W1_1 + obj.b1_1;
            a1 = max(0, z1);
            z2 = a1 * obj.W2_1 + obj.b2_1;
            a2 = max(0, z2);
            Q1 = a2 * obj.W3_1 + obj.b3_1;

            % Backprop to get dQ/da
            dQ_dz3 = ones(batch_size, 1);
            dQ_da2 = dQ_dz3 * obj.W3_1';
            dQ_dz2 = dQ_da2 .* (z2 > 0);
            dQ_da1 = dQ_dz2 * obj.W2_1';
            dQ_dz1 = dQ_da1 .* (z1 > 0);
            dQ_dx = dQ_dz1 * obj.W1_1';

            % Extract action gradient (last action_dim columns of x)
            dQ_da = dQ_dx(:, (obj.state_dim + 1):end);
        end

        function [grads1, grads2, loss] = compute_gradients(obj, states, actions, targets)
            %COMPUTE_GRADIENTS Compute gradients for both critics
            %   [grads1, grads2, loss] = critic.compute_gradients(states, actions, targets)
            %
            %   Minimizes MSE between Q(s, a) and target values
            %
            %   Inputs:
            %       states  - Batch of states (batch_size x state_dim)
            %       actions - Batch of actions (batch_size x action_dim)
            %       targets - Target Q-values (batch_size x 1)
            %
            %   Outputs:
            %       grads1 - Gradients for network 1
            %       grads2 - Gradients for network 2
            %       loss   - Mean squared error loss

            batch_size = size(states, 1);
            x = [states, actions];

            %% Network 1 forward pass
            z1_1 = x * obj.W1_1 + obj.b1_1;
            a1_1 = max(0, z1_1);
            z2_1 = a1_1 * obj.W2_1 + obj.b2_1;
            a2_1 = max(0, z2_1);
            Q1 = a2_1 * obj.W3_1 + obj.b3_1;

            %% Network 2 forward pass
            z1_2 = x * obj.W1_2 + obj.b1_2;
            a1_2 = max(0, z1_2);
            z2_2 = a1_2 * obj.W2_2 + obj.b2_2;
            a2_2 = max(0, z2_2);
            Q2 = a2_2 * obj.W3_2 + obj.b3_2;

            % Loss (MSE)
            loss1 = mean((Q1 - targets).^2);
            loss2 = mean((Q2 - targets).^2);
            loss = (loss1 + loss2) / 2;

            %% Network 1 backprop
            dL_dQ1 = 2 * (Q1 - targets) / batch_size;
            dL_da2_1 = dL_dQ1 * obj.W3_1';
            dL_dz2_1 = dL_da2_1 .* (z2_1 > 0);
            dL_da1_1 = dL_dz2_1 * obj.W2_1';
            dL_dz1_1 = dL_da1_1 .* (z1_1 > 0);

            grads1.W3 = a2_1' * dL_dQ1 + obj.l2_reg * obj.W3_1;
            grads1.b3 = sum(dL_dQ1);
            grads1.W2 = a1_1' * dL_dz2_1 + obj.l2_reg * obj.W2_1;
            grads1.b2 = sum(dL_dz2_1, 1);
            grads1.W1 = x' * dL_dz1_1 + obj.l2_reg * obj.W1_1;
            grads1.b1 = sum(dL_dz1_1, 1);

            %% Network 2 backprop
            dL_dQ2 = 2 * (Q2 - targets) / batch_size;
            dL_da2_2 = dL_dQ2 * obj.W3_2';
            dL_dz2_2 = dL_da2_2 .* (z2_2 > 0);
            dL_da1_2 = dL_dz2_2 * obj.W2_2';
            dL_dz1_2 = dL_da1_2 .* (z1_2 > 0);

            grads2.W3 = a2_2' * dL_dQ2 + obj.l2_reg * obj.W3_2;
            grads2.b3 = sum(dL_dQ2);
            grads2.W2 = a1_2' * dL_dz2_2 + obj.l2_reg * obj.W2_2;
            grads2.b2 = sum(dL_dz2_2, 1);
            grads2.W1 = x' * dL_dz1_2 + obj.l2_reg * obj.W1_2;
            grads2.b1 = sum(dL_dz1_2, 1);
        end

        function update(obj, grads1, grads2)
            %UPDATE Update both network parameters
            %   critic.update(grads1, grads2)

            % Update network 1
            obj.W1_1 = obj.W1_1 - obj.learning_rate * grads1.W1;
            obj.b1_1 = obj.b1_1 - obj.learning_rate * grads1.b1;
            obj.W2_1 = obj.W2_1 - obj.learning_rate * grads1.W2;
            obj.b2_1 = obj.b2_1 - obj.learning_rate * grads1.b2;
            obj.W3_1 = obj.W3_1 - obj.learning_rate * grads1.W3;
            obj.b3_1 = obj.b3_1 - obj.learning_rate * grads1.b3;

            % Update network 2
            obj.W1_2 = obj.W1_2 - obj.learning_rate * grads2.W1;
            obj.b1_2 = obj.b1_2 - obj.learning_rate * grads2.b1;
            obj.W2_2 = obj.W2_2 - obj.learning_rate * grads2.W2;
            obj.b2_2 = obj.b2_2 - obj.learning_rate * grads2.b2;
            obj.W3_2 = obj.W3_2 - obj.learning_rate * grads2.W3;
            obj.b3_2 = obj.b3_2 - obj.learning_rate * grads2.b3;
        end

        function copy_from(obj, source_critic, tau)
            %COPY_FROM Soft copy parameters from another critic
            %   critic.copy_from(source, tau) soft update
            %   critic.copy_from(source, 1) hard copy

            if nargin < 3
                tau = 1;
            end

            % Network 1
            obj.W1_1 = tau * source_critic.W1_1 + (1 - tau) * obj.W1_1;
            obj.b1_1 = tau * source_critic.b1_1 + (1 - tau) * obj.b1_1;
            obj.W2_1 = tau * source_critic.W2_1 + (1 - tau) * obj.W2_1;
            obj.b2_1 = tau * source_critic.b2_1 + (1 - tau) * obj.b2_1;
            obj.W3_1 = tau * source_critic.W3_1 + (1 - tau) * obj.W3_1;
            obj.b3_1 = tau * source_critic.b3_1 + (1 - tau) * obj.b3_1;

            % Network 2
            obj.W1_2 = tau * source_critic.W1_2 + (1 - tau) * obj.W1_2;
            obj.b1_2 = tau * source_critic.b1_2 + (1 - tau) * obj.b1_2;
            obj.W2_2 = tau * source_critic.W2_2 + (1 - tau) * obj.W2_2;
            obj.b2_2 = tau * source_critic.b2_2 + (1 - tau) * obj.b2_2;
            obj.W3_2 = tau * source_critic.W3_2 + (1 - tau) * obj.W3_2;
            obj.b3_2 = tau * source_critic.b3_2 + (1 - tau) * obj.b3_2;
        end

        function save(obj, filename)
            %SAVE Save networks to file
            critic_data.W1_1 = obj.W1_1; critic_data.b1_1 = obj.b1_1;
            critic_data.W2_1 = obj.W2_1; critic_data.b2_1 = obj.b2_1;
            critic_data.W3_1 = obj.W3_1; critic_data.b3_1 = obj.b3_1;
            critic_data.W1_2 = obj.W1_2; critic_data.b1_2 = obj.b1_2;
            critic_data.W2_2 = obj.W2_2; critic_data.b2_2 = obj.b2_2;
            critic_data.W3_2 = obj.W3_2; critic_data.b3_2 = obj.b3_2;
            critic_data.state_dim = obj.state_dim;
            critic_data.action_dim = obj.action_dim;
            critic_data.hidden_dims = obj.hidden_dims;

            save(filename, 'critic_data');
        end

        function load(obj, filename)
            %LOAD Load networks from file
            data = load(filename);

            obj.W1_1 = data.critic_data.W1_1; obj.b1_1 = data.critic_data.b1_1;
            obj.W2_1 = data.critic_data.W2_1; obj.b2_1 = data.critic_data.b2_1;
            obj.W3_1 = data.critic_data.W3_1; obj.b3_1 = data.critic_data.b3_1;
            obj.W1_2 = data.critic_data.W1_2; obj.b1_2 = data.critic_data.b1_2;
            obj.W2_2 = data.critic_data.W2_2; obj.b2_2 = data.critic_data.b2_2;
            obj.W3_2 = data.critic_data.W3_2; obj.b3_2 = data.critic_data.b3_2;
        end
    end

end
