function export_comparison_to_blender(results_cell, controller_names, output_dir)
%EXPORT_COMPARISON_TO_BLENDER Export multiple controller results for comparison
%   export_comparison_to_blender(results_cell, controller_names, output_dir)
%
%   Creates separate JSON files for each controller, plus a manifest file
%
%   Inputs:
%       results_cell     - Cell array of result structures
%       controller_names - Cell array of controller names
%       output_dir       - Output directory path

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
