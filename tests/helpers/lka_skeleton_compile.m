function [ok, msg] = lka_skeleton_compile(laneSource)
% LKA_SKELETON_COMPILE Updates aBaja2026.slx without RoadRunner (InitFcn swapped).
%
% Syntax:
%   [ok, msg] = lka_skeleton_compile(laneSource)        1 camera, 2 emulated
%
% The skeleton InitFcn needs a live rrSim; it is replaced for the update by the
% supporting setup helper plus a dummy start pose and restored afterwards (model
% closed without saving). Camera detector = classic, so the ONNX add-on is not needed.
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    skel = fullfile(root, 'SoftwareVirtualWorldSimulation-master');
    run(fullfile(root, 'config', 'startup.m'));
    addpath(fullfile(root, 'models'), fullfile(skel, 'Models'), fullfile(skel, 'SupportingFunctions'));
    evalin('base', 'lkaProfile = ''skeleton''; lka_controller_params; create_lka_buses;');
    evalin('base', sprintf('LKA_LANE_SOURCE = %d; LKA.Vision.Detector = ''mobilenet'';', laneSource));
    mdl = 'aBaja2026';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    load_system(fullfile(skel, 'Models', [mdl '.slx']));
    cleanup = onCleanup(@() close_system(mdl, 0));
    set_param(mdl, 'InitFcn', ['helperSLTrajectoryFollowerWithRRScenarioSetup; ' ...
        'initialPosition = [0 0 0]; initialRotation = [0 0 0];']);
    try
        set_param(mdl, 'SimulationCommand', 'update');
        ok = true; msg = '';
    catch ME
        ok = false; msg = regexprep(ME.message, '<[^>]*>', '');
        for k = 1:numel(ME.cause)
            msg = [msg newline regexprep(ME.cause{k}.message, '<[^>]*>', '')]; %#ok<AGROW>
        end
    end
end
