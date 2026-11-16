function results = run_lka_vision(options)
% RUN_LKA_VISION Runs the camera-in-the-loop LKA bench (Unreal) for detectors and scenarios.
%
% Syntax:
%   results = run_lka_vision()
%   results = run_lka_vision(Scenarios={'S1_Nominal','S2_SpeedLimit'}, Detectors={'mobilenet'})
%
% Name-value options:
%   Scenarios  - scenario names from generate_lka_scenarios (default {'S1_Nominal'})
%   Detectors  - detector names (lka_detector_create('list'), default {'mobilenet'})
%   Controller - 1 pure pursuit, 2 Stanley 2025, 3 MPC (default 1)
%   Video      - write results/vision/<scenario>_<detector>.mp4 (default true)
%   Dataset    - record frames for the detector benchmark (default false)
%   RawVideo   - also write the unannotated camera video <scenario>_<detector>_raw.mp4 (default false)
%   OutDir     - default results/vision
%   Rebuild    - rebuild LKA_Vision_TestBench first (default false)
%   Monitor    - publish live telemetry for the dashboard (default true)
% Unreal and the lane network share the GPU (6 GB on an RTX 4050 laptop): with little free
% GPU memory Unreal stops at start-up with "3D Simulation engine was terminated by the
% user (error code: 4)". The run warns below 2.5 GB free; reset(gpuDevice) (after
% clearing old detectors) frees what MATLAB holds.
% Output: struct array (res, kpi); CSV lka_vision_kpis.csv and figures in OutDir.

    arguments
        options.Scenarios cell = {'S1_Nominal'}
        options.Detectors cell = {'mobilenet'}
        options.Controller (1,1) double = 1
        options.Video (1,1) logical = true
        options.Dataset (1,1) logical = false
        options.RawVideo (1,1) logical = false
        options.OutDir char = ''
        options.Rebuild (1,1) logical = false
        options.Monitor (1,1) logical = true
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    if isempty(options.OutDir)
        options.OutDir = fullfile(root, 'results', 'vision');
    end
    run(fullfile(root, 'config', 'startup.m'));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params; create_lka_buses;');
    vehicle = evalin('base', 'vehicle');
    LKA0 = evalin('base', 'LKA');
    mdl = 'LKA_Vision_TestBench';
    if options.Rebuild || ~isfile(fullfile(root, 'models', [mdl '.slx']))
        build_lka_vision_bench();
    end
    load_system(mdl);
    scenarios = generate_lka_scenarios(vehicle);
    scenarios = scenarios(ismember({scenarios.Name}, options.Scenarios));
    results = struct('res', {}, 'kpi', {});
    for s = 1:numel(scenarios)
        sc = scenarios(s);
        r = sc.Road;
        scene = sc.Scene;                               % Unreal scene of this scenario
        if isempty(scene), scene = LKA0.Vision.Bench.ScenePath; end
        set_param([mdl '/Simulation 3D Scene Configuration'], 'RoadRunnerProjectPath', scene);
        assignin('base', 'SCN', sc.P);
        assignin('base', 'ROAD', struct('s', r.s, 'x', r.x, 'y', r.y, 'psi', r.psi, 'kappa', r.kappa, ...
            'dkappa', r.dkappa, 'width', r.width, 'lineWidth', r.lineWidth));
        for d = 1:numel(options.Detectors)
            LKA = LKA0;
            LKA.ControllerSelect = options.Controller;
            LKA.Vision.Detector = options.Detectors{d};
            LKA.Vision.Monitor = options.Monitor;
            tag = sprintf('%s_%s', sc.Name, LKA.Vision.Detector);
            if canUseGPU()
                g = gpuDevice;
                if g.AvailableMemory < 2.5e9
                    warning('lka:vision:lowGpuMemory', ['Only %.1f GB GPU memory free: Unreal may fail ' ...
                        'to start. Clear old detectors and run reset(gpuDevice).'], g.AvailableMemory / 1e9);
                end
            end
            LKA.Vision.Bench.VideoFile = '';
            if options.Video
                LKA.Vision.Bench.VideoFile = fullfile(options.OutDir, [tag '.mp4']);
            end
            LKA.Vision.Bench.RawVideoFile = '';
            if options.RawVideo
                LKA.Vision.Bench.RawVideoFile = fullfile(options.OutDir, [tag '_raw.mp4']);
            end
            LKA.Vision.Bench.DatasetDir = '';
            if options.Dataset
                LKA.Vision.Bench.DatasetDir = fullfile(root, 'data', 'vision_datasets', tag);
            end
            assignin('base', 'LKA', LKA);
            fprintf('[LKA vision] %-16s camera %-8s ... ', sc.Name, LKA.Vision.Detector);
            lka_monitor_publish('start', struct('Name', tag, 'Kind', 'sim', 'Detector', LKA.Vision.Detector, ...
                'Gpu', LKA.Vision.UseGpu && canUseGPU(), 'OutDir', options.OutDir, ...
                'Frames', round(sc.P.StopTime / LKA.Ts), 'Duration', sc.P.StopTime, ...
                'Phase', 'closed-loop simulation (Unreal camera)'));
            t0 = tic;
            simOut = sim(mdl);
            res = lka_extract_results(simOut, sc, LKA, vehicle);
            res.ControllerName = sprintf('%s, camera %s', res.ControllerName, LKA.Vision.Detector);
            kpi = lka_compute_kpis(res);
            vi = squeeze(simOut.logsout.getElement('vision_info').Values.Data);
            if size(vi, 1) == 7, vi = vi'; end
            kpi.Detector = string(LKA.Vision.Detector);
            kpi.VisionMsMean = mean(vi(2:end, 7));
            kpi.BothLinesRatio = mean(vi(:, 3) & vi(:, 4));
            kpi.WallClock_s = toc(t0);
            fprintf('%5.0f s | max|lat| %.3f m | in lane %5.1f %% | %.1f ms/frame\n', kpi.WallClock_s, ...
                kpi.MaxLatOffset_m, 100 * kpi.InLaneRatio, kpi.VisionMsMean);
            lka_monitor_publish('done', struct('OutDir', options.OutDir, 'MaxLatOffset_m', kpi.MaxLatOffset_m, ...
                'RmsLatOffset_m', kpi.RmsLatOffset_m, 'InLaneRatio', kpi.InLaneRatio, 'MeanMs', kpi.VisionMsMean));
            plot_lka_results(res, kpi, options.OutDir);
            close all
            results(end + 1) = struct('res', res, 'kpi', kpi); %#ok<AGROW>
        end
    end
    assignin('base', 'LKA', LKA0);
    if ~isfolder(options.OutDir), mkdir(options.OutDir); end
    % merge with earlier runs: one row per (scenario, controller, detector), newest wins
    csv = fullfile(options.OutDir, 'lka_vision_kpis.csv');
    T = struct2table(arrayfun(@(x) x.kpi, results));
    T.Scenario = string(T.Scenario); T.Controller = string(T.Controller); T.Detector = string(T.Detector);
    if isfile(csv)
        old = readtable(csv, 'TextType', 'string');
        if isequal(sort(old.Properties.VariableNames), sort(T.Properties.VariableNames))
            keep = ~ismember(old.Scenario + "|" + old.Controller, T.Scenario + "|" + T.Controller);
            T = [old(keep, T.Properties.VariableNames); T];
        end
    end
    writetable(T, csv);
end
