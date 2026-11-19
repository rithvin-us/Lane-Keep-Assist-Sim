% ==============================================================================
% startup.m
% Initializes the LKA Project environment, paths, and baseline workspace.
% Follows MathWorks Automotive Advisory Board (MAAB) & Simulink Project practices.
% ==============================================================================

fprintf('\n[LKA System] Initializing environment...\n');

% 1. Determine project root directory
projectRoot = fileparts(fileparts(mfilename('fullpath')));

% 2. Add required directories to MATLAB path
pathsToAdd = {
    fullfile(projectRoot, 'config');
    fullfile(projectRoot, 'data');
    fullfile(projectRoot, 'data', 'bus_definitions');
    fullfile(projectRoot, 'data', 'parameters');
    fullfile(projectRoot, 'data', 'scenarios');
    fullfile(projectRoot, 'models');
    fullfile(projectRoot, 'src');
    fullfile(projectRoot, 'src', 'controllers');
    fullfile(projectRoot, 'src', 'perception');
    fullfile(projectRoot, 'src', 'utilities');
    fullfile(projectRoot, 'src', 'plant');
    fullfile(projectRoot, 'src', 'evaluation');
    fullfile(projectRoot, 'src', 'vision');
    fullfile(projectRoot, 'src', 'sensors');
    fullfile(projectRoot, 'src', 'legacy');
    fullfile(projectRoot, 'tests', 'helpers');
    fullfile(projectRoot, 'scripts');
    fullfile(projectRoot, 'tests');
    fullfile(projectRoot, 'tests', 'unit_tests');
    fullfile(projectRoot, 'tests', 'scenario_tests');
};

% every lane-detector approach: perception/<Approach>/deploy holds the ONNX, its sidecar
% and the layer code the ONNX importer generates (it has to be on the path)
deployDirs = dir(fullfile(projectRoot, 'perception', '*', 'deploy'));
pathsToAdd = [pathsToAdd; unique({deployDirs.folder})'];   % dir lists the folders' contents

for i = 1:numel(pathsToAdd)
    if exist(pathsToAdd{i}, 'dir')
        addpath(pathsToAdd{i});
    end
end

% 3. Configure Simulink cache and code generation directory to 'work/'
cacheFolder = fullfile(projectRoot, 'work');
if ~exist(cacheFolder, 'dir')
    mkdir(cacheFolder);
end

try
    Simulink.fileGenControl('set', ...
        'CacheFolder', cacheFolder, ...
        'CodeGenFolder', cacheFolder);
catch ME
    warning('[LKA System] Could not set Simulink.fileGenControl: %s', ME.message);
end

% 4. Load baseline parameters and bus objects into base workspace
try
    evalin('base', 'vehicle_params;');
    evalin('base', 'sensor_params;');
    evalin('base', 'lka_controller_params;');
    evalin('base', 'create_lka_buses;');
    fprintf('[LKA System] Baseline vehicle, controller, and bus parameters loaded successfully.\n');
catch ME
    warning('[LKA System] Parameter loading warning: %s', ME.message);
end

fprintf('[LKA System] Ready. Run scripts/run_lka_mil.m (Simulink LKA test bench) or scripts/run_baseline_simulation.m\n\n');
