%% setup_paths.m - Initialize TD3-ADRC project paths
% Run this script before using any project functions

% Get project root directory
projectRoot = fileparts(mfilename('fullpath'));

% Add all source directories to path
addpath(fullfile(projectRoot, 'params'));
addpath(fullfile(projectRoot, 'src', 'dynamics'));
addpath(fullfile(projectRoot, 'src', 'control', 'adrc'));
addpath(fullfile(projectRoot, 'src', 'control', 'td3_adrc'));
addpath(fullfile(projectRoot, 'src', 'estimation'));
addpath(fullfile(projectRoot, 'src', 'visualization'));
addpath(fullfile(projectRoot, 'src', 'mcp_tools'));
addpath(fullfile(projectRoot, 'demos'));
addpath(fullfile(projectRoot, 'tests'));

% Create results directories if they don't exist
resultsDir = fullfile(projectRoot, 'results');
if ~exist(fullfile(resultsDir, 'figures'), 'dir')
    mkdir(fullfile(resultsDir, 'figures'));
end
if ~exist(fullfile(resultsDir, 'animations'), 'dir')
    mkdir(fullfile(resultsDir, 'animations'));
end

fprintf('TD3-ADRC project paths initialized.\n');
fprintf('Project root: %s\n', projectRoot);
