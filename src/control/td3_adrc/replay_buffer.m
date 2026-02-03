classdef replay_buffer < handle
%REPLAY_BUFFER Experience replay buffer for TD3
%   Stores transitions (s, a, r, s', done) for off-policy learning
%
%   Reference: Mnih et al. (2015) "Human-level control through deep RL"

    properties
        capacity        % Maximum buffer size
        buffer          % Cell array storing transitions
        position        % Current write position
        size            % Current number of stored transitions

        % State and action dimensions
        state_dim
        action_dim
    end

    methods
        function obj = replay_buffer(capacity, state_dim, action_dim)
            %REPLAY_BUFFER Construct replay buffer
            %   buffer = replay_buffer(capacity, state_dim, action_dim)
            %
            %   Inputs:
            %       capacity   - Maximum number of transitions to store
            %       state_dim  - Dimension of state vector
            %       action_dim - Dimension of action vector

            obj.capacity = capacity;
            obj.state_dim = state_dim;
            obj.action_dim = action_dim;

            % Preallocate arrays for efficiency
            obj.buffer.states = zeros(capacity, state_dim);
            obj.buffer.actions = zeros(capacity, action_dim);
            obj.buffer.rewards = zeros(capacity, 1);
            obj.buffer.next_states = zeros(capacity, state_dim);
            obj.buffer.dones = false(capacity, 1);

            obj.position = 1;
            obj.size = 0;
        end

        function add(obj, state, action, reward, next_state, done)
            %ADD Add a transition to the buffer
            %   buffer.add(state, action, reward, next_state, done)
            %
            %   Inputs:
            %       state      - Current state (1 x state_dim)
            %       action     - Action taken (1 x action_dim)
            %       reward     - Reward received (scalar)
            %       next_state - Next state (1 x state_dim)
            %       done       - Episode termination flag (boolean)

            obj.buffer.states(obj.position, :) = state(:)';
            obj.buffer.actions(obj.position, :) = action(:)';
            obj.buffer.rewards(obj.position) = reward;
            obj.buffer.next_states(obj.position, :) = next_state(:)';
            obj.buffer.dones(obj.position) = done;

            % Update position (circular buffer)
            obj.position = mod(obj.position, obj.capacity) + 1;
            obj.size = min(obj.size + 1, obj.capacity);
        end

        function [states, actions, rewards, next_states, dones] = sample(obj, batch_size)
            %SAMPLE Sample a random batch of transitions
            %   [s, a, r, s', d] = buffer.sample(batch_size)
            %
            %   Inputs:
            %       batch_size - Number of transitions to sample
            %
            %   Outputs:
            %       states      - Batch of states (batch_size x state_dim)
            %       actions     - Batch of actions (batch_size x action_dim)
            %       rewards     - Batch of rewards (batch_size x 1)
            %       next_states - Batch of next states (batch_size x state_dim)
            %       dones       - Batch of done flags (batch_size x 1)

            if obj.size < batch_size
                error('Not enough samples in buffer. Have %d, need %d', ...
                    obj.size, batch_size);
            end

            % Random indices
            indices = randperm(obj.size, batch_size);

            states = obj.buffer.states(indices, :);
            actions = obj.buffer.actions(indices, :);
            rewards = obj.buffer.rewards(indices);
            next_states = obj.buffer.next_states(indices, :);
            dones = obj.buffer.dones(indices);
        end

        function [states, actions, rewards, next_states, dones] = sample_sequential(obj, batch_size, sequence_length)
            %SAMPLE_SEQUENTIAL Sample sequential transitions for RNN training
            %   [s, a, r, s', d] = buffer.sample_sequential(batch_size, seq_len)
            %
            %   Returns sequences of transitions for recurrent policies

            if obj.size < sequence_length
                error('Not enough samples for sequences');
            end

            max_start = obj.size - sequence_length + 1;
            start_indices = randperm(max_start, batch_size);

            states = zeros(batch_size, sequence_length, obj.state_dim);
            actions = zeros(batch_size, sequence_length, obj.action_dim);
            rewards = zeros(batch_size, sequence_length);
            next_states = zeros(batch_size, sequence_length, obj.state_dim);
            dones = false(batch_size, sequence_length);

            for i = 1:batch_size
                idx_range = start_indices(i):(start_indices(i) + sequence_length - 1);
                states(i, :, :) = obj.buffer.states(idx_range, :);
                actions(i, :, :) = obj.buffer.actions(idx_range, :);
                rewards(i, :) = obj.buffer.rewards(idx_range);
                next_states(i, :, :) = obj.buffer.next_states(idx_range, :);
                dones(i, :) = obj.buffer.dones(idx_range);
            end
        end

        function n = get_size(obj)
            %GET_SIZE Return current buffer size
            n = obj.size;
        end

        function is_ready = is_ready_for_training(obj, min_size)
            %IS_READY_FOR_TRAINING Check if buffer has enough samples
            is_ready = obj.size >= min_size;
        end

        function clear(obj)
            %CLEAR Clear all stored transitions
            obj.buffer.states(:) = 0;
            obj.buffer.actions(:) = 0;
            obj.buffer.rewards(:) = 0;
            obj.buffer.next_states(:) = 0;
            obj.buffer.dones(:) = false;

            obj.position = 1;
            obj.size = 0;
        end

        function save(obj, filename)
            %SAVE Save buffer to file
            buffer_data = obj.buffer;
            buffer_meta.capacity = obj.capacity;
            buffer_meta.position = obj.position;
            buffer_meta.size = obj.size;
            buffer_meta.state_dim = obj.state_dim;
            buffer_meta.action_dim = obj.action_dim;

            save(filename, 'buffer_data', 'buffer_meta');
        end

        function load(obj, filename)
            %LOAD Load buffer from file
            data = load(filename);

            obj.buffer = data.buffer_data;
            obj.capacity = data.buffer_meta.capacity;
            obj.position = data.buffer_meta.position;
            obj.size = data.buffer_meta.size;
            obj.state_dim = data.buffer_meta.state_dim;
            obj.action_dim = data.buffer_meta.action_dim;
        end

        function stats = get_statistics(obj)
            %GET_STATISTICS Return buffer statistics
            if obj.size == 0
                stats.mean_reward = 0;
                stats.std_reward = 0;
                stats.min_reward = 0;
                stats.max_reward = 0;
                stats.episode_count = 0;
            else
                valid_rewards = obj.buffer.rewards(1:obj.size);
                stats.mean_reward = mean(valid_rewards);
                stats.std_reward = std(valid_rewards);
                stats.min_reward = min(valid_rewards);
                stats.max_reward = max(valid_rewards);
                stats.episode_count = sum(obj.buffer.dones(1:obj.size));
            end
            stats.size = obj.size;
            stats.capacity = obj.capacity;
            stats.utilization = obj.size / obj.capacity;
        end
    end

end
