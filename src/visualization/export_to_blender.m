function export_to_blender(flight_data, filepath)
%EXPORT_TO_BLENDER Export flight data to JSON for Blender animation
%   export_to_blender(flight_data, filepath)
%
%   Inputs:
%       flight_data - Structure with fields:
%           .time           - Time vector [N x 1] [s]
%           .position       - Position history [N x 3] [m]
%           .euler_angles   - Euler angles [N x 3] [rad]
%           .motor_speeds   - Motor speeds [N x 4] [rad/s] (optional)
%           .controller_params - TD3 actions [N x 3] (optional)
%           .disturbance    - Disturbance estimates [N x 6] (optional)
%           .controller_name- Name of controller (optional)
%       filepath    - Output JSON file path
%
%   Example:
%       flight_data.time = t';
%       flight_data.position = pos_history';
%       flight_data.euler_angles = euler_history';
%       export_to_blender(flight_data, 'results/flight_data.json');

    % Validate inputs
    if ~isfield(flight_data, 'time')
        error('flight_data must have a time field');
    end
    if ~isfield(flight_data, 'position')
        error('flight_data must have a position field');
    end
    if ~isfield(flight_data, 'euler_angles')
        error('flight_data must have an euler_angles field');
    end

    % Ensure column vectors are transposed to [N x dim]
    if size(flight_data.time, 1) == 1
        flight_data.time = flight_data.time';
    end
    if size(flight_data.position, 1) == 3
        flight_data.position = flight_data.position';
    end
    if size(flight_data.euler_angles, 1) == 3
        flight_data.euler_angles = flight_data.euler_angles';
    end

    N = length(flight_data.time);

    % Build JSON structure
    json_data = struct();

    % Metadata
    json_data.metadata.num_frames = N;
    json_data.metadata.duration = flight_data.time(end) - flight_data.time(1);
    json_data.metadata.sample_rate = 1 / mean(diff(flight_data.time));
    json_data.metadata.created = datestr(now, 'yyyy-mm-dd HH:MM:SS');

    if isfield(flight_data, 'controller_name')
        json_data.metadata.controller = flight_data.controller_name;
    else
        json_data.metadata.controller = 'TD3-ADRC';
    end

    % Time data
    json_data.time = flight_data.time(:)';

    % Position data (convert to list of [x, y, z])
    json_data.position = cell(N, 1);
    for i = 1:N
        json_data.position{i} = flight_data.position(i, :);
    end

    % Euler angles (convert to list of [roll, pitch, yaw])
    json_data.euler_angles = cell(N, 1);
    for i = 1:N
        json_data.euler_angles{i} = flight_data.euler_angles(i, :);
    end

    % Optional: motor speeds
    if isfield(flight_data, 'motor_speeds') && ~isempty(flight_data.motor_speeds)
        motor_speeds = flight_data.motor_speeds;
        if size(motor_speeds, 1) == 4
            motor_speeds = motor_speeds';
        end
        json_data.motor_speeds = cell(N, 1);
        for i = 1:N
            json_data.motor_speeds{i} = motor_speeds(i, :);
        end
    end

    % Optional: controller parameters (TD3 actions)
    if isfield(flight_data, 'controller_params') && ~isempty(flight_data.controller_params)
        params = flight_data.controller_params;
        if size(params, 1) == 3
            params = params';
        end
        json_data.controller_params = cell(N, 1);
        for i = 1:N
            json_data.controller_params{i} = params(i, :);
        end
    end

    % Optional: disturbance estimates
    if isfield(flight_data, 'disturbance') && ~isempty(flight_data.disturbance)
        dist = flight_data.disturbance;
        if size(dist, 1) == 6 || size(dist, 1) == 3
            dist = dist';
        end
        json_data.disturbance = cell(size(dist, 1), 1);
        for i = 1:size(dist, 1)
            json_data.disturbance{i} = dist(i, :);
        end
    end

    % Optional: tracking error
    if isfield(flight_data, 'error') && ~isempty(flight_data.error)
        err = flight_data.error;
        if size(err, 1) == 3
            err = err';
        end
        json_data.tracking_error = cell(size(err, 1), 1);
        for i = 1:size(err, 1)
            json_data.tracking_error{i} = err(i, :);
        end
    end

    % Convert to JSON string
    json_str = jsonencode(json_data);

    % Pretty print (add newlines for readability)
    json_str = strrep(json_str, ',"', sprintf(',\n"'));
    json_str = strrep(json_str, '{', sprintf('{\n'));
    json_str = strrep(json_str, '}', sprintf('\n}'));

    % Write to file
    fid = fopen(filepath, 'w');
    if fid == -1
        error('Could not open file for writing: %s', filepath);
    end
    fprintf(fid, '%s', json_str);
    fclose(fid);

    fprintf('Exported %d frames to: %s\n', N, filepath);

end


function export_comparison_to_blender(results_cell, controller_names, output_dir)
%EXPORT_COMPARISON_TO_BLENDER Export multiple controller results for comparison
%   export_comparison_to_blender(results_cell, controller_names, output_dir)
%
%   Creates separate JSON files for each controller, plus a manifest file

    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    n_ctrl = length(results_cell);
    manifest = struct();
    manifest.controllers = controller_names;
    manifest.files = cell(n_ctrl, 1);

    for i = 1:n_ctrl
        % Build flight data from results
        r = results_cell{i};

        flight_data = struct();
        flight_data.time = r.time(:);
        flight_data.position = zeros(length(r.time), 3);  % Placeholder if not available
        flight_data.euler_angles = r.output';
        flight_data.controller_name = controller_names{i};

        if isfield(r, 'control')
            flight_data.control = r.control';
        end
        if isfield(r, 'error')
            flight_data.error = r.error';
        end
        if isfield(r, 'disturbance')
            flight_data.disturbance = r.disturbance';
        end
        if isfield(r, 'td3_action')
            flight_data.controller_params = r.td3_action';
        end

        % Export individual file
        filename = sprintf('%s_flight.json', lower(strrep(controller_names{i}, '-', '_')));
        filepath = fullfile(output_dir, filename);
        export_to_blender(flight_data, filepath);

        manifest.files{i} = filename;
    end

    % Write manifest
    manifest_path = fullfile(output_dir, 'manifest.json');
    manifest_str = jsonencode(manifest);
    fid = fopen(manifest_path, 'w');
    fprintf(fid, '%s', manifest_str);
    fclose(fid);

    fprintf('Exported comparison data for %d controllers to: %s\n', n_ctrl, output_dir);

end


function export_animation_config(config, filepath)
%EXPORT_ANIMATION_CONFIG Export Blender animation configuration
%   export_animation_config(config, filepath)
%
%   Inputs:
%       config - Structure with animation settings:
%           .fps         - Frames per second (default: 60)
%           .camera_mode - 'fixed', 'follow', 'orbit' (default: 'follow')
%           .show_trail  - Show trajectory trail (default: true)
%           .trail_length- Length of trail in seconds (default: 2)
%           .show_reference- Show reference trajectory (default: true)
%           .motor_viz   - Motor speed visualization (default: true)
%           .comparison_spacing - Spacing between drones for comparison

    % Default configuration
    if nargin < 1 || isempty(config)
        config = struct();
    end

    defaults = struct(...
        'fps', 60, ...
        'camera_mode', 'follow', ...
        'show_trail', true, ...
        'trail_length', 2.0, ...
        'show_reference', true, ...
        'motor_viz', true, ...
        'comparison_spacing', 5.0, ...
        'render_resolution', [1920, 1080], ...
        'output_format', 'mp4' ...
    );

    fields = fieldnames(defaults);
    for i = 1:length(fields)
        if ~isfield(config, fields{i})
            config.(fields{i}) = defaults.(fields{i});
        end
    end

    % Write to file
    json_str = jsonencode(config);
    fid = fopen(filepath, 'w');
    fprintf(fid, '%s', json_str);
    fclose(fid);

    fprintf('Animation config saved to: %s\n', filepath);

end
