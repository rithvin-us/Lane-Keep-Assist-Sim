% ==============================================================================
% shutdown.m
% Cleans up paths, closes open models, and resets the MATLAB workspace.
% ==============================================================================

fprintf('\n[LKA System] Shutting down environment...\n');

projectRoot = fileparts(fileparts(mfilename('fullpath')));

% Close all open LKA Simulink models without saving uncommitted changes
lkaModels = {'LKA_MIL_TestBench', 'LKA_Vision_TestBench', 'LKA_Replay_Harness', 'LKA_Controller', ...
    'VirtualLaneSensor', 'aBaja2026'};
for i = 1:numel(lkaModels)
    if bdIsLoaded(lkaModels{i})
        close_system(lkaModels{i}, 0);
    end
end

% Reset Simulink cache folder back to default
try
    Simulink.fileGenControl('reset');
catch
    % ignored if Simulink is not present
end

% Remove project directories from MATLAB path
pathsToRemove = {
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
    fullfile(projectRoot, 'models', 'perception');
    fullfile(projectRoot, 'tests', 'helpers');
    fullfile(projectRoot, 'scripts');
    fullfile(projectRoot, 'tests');
    fullfile(projectRoot, 'tests', 'unit_tests');
    fullfile(projectRoot, 'tests', 'scenario_tests');
};

for i = 1:numel(pathsToRemove)
    if exist(pathsToRemove{i}, 'dir')
        rmpath(pathsToRemove{i});
    end
end

fprintf('[LKA System] Paths removed and environment successfully cleaned.\n\n');
