function T = run_detector_benchmark(options)
% RUN_DETECTOR_BENCHMARK YOLOP vs team CNN vs classic vs LaneAssist, same frames.
%
% Syntax:
%   T = run_detector_benchmark()                                    % record + score + closed loop
%   T = run_detector_benchmark(Dataset='data/vision_datasets/S1_Nominal_classic', ClosedLoop=false)
%
% Name-value options:
%   Dataset    - recorded dataset folder; empty = record one with run_lka_vision (S1, classic)
%   Detectors  - default: classic, laneassist (ported, docs/laneassist_comparison.md),
%                yolop (if the ONNX add-on exists), cnn (if team_cnn.onnx exists)
%   ClosedLoop - also run run_lka_vision for each detector and append the lane KPIs (default true)
%   OutDir     - default results/benchmark
% Writes detector_benchmark.csv, detector_benchmark.md and detector_benchmark.png to OutDir.

    arguments
        options.Dataset char = ''
        options.Detectors cell = {}
        options.ClosedLoop (1,1) logical = true
        options.OutDir char = ''
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    if isempty(options.OutDir)
        options.OutDir = fullfile(root, 'results', 'benchmark');
    end
    run(fullfile(root, 'config', 'startup.m'));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params;');
    V = evalin('base', 'LKA.Vision');
    if isempty(options.Detectors)
        options.Detectors = {'classic', 'laneassist'};
        if exist('importNetworkFromONNX', 'file') == 2
            options.Detectors{end + 1} = 'yolop';
            if isfile(fullfile(V.DetectorDir, 'team_cnn.onnx'))
                options.Detectors{end + 1} = 'cnn';
            end
        end
    end
    if isempty(options.Dataset)
        run_lka_vision(Scenarios={'S1_Nominal'}, Detectors={'classic'}, Video=false, Dataset=true);
        options.Dataset = fullfile(root, 'data', 'vision_datasets', 'S1_Nominal_classic');
    end

    T = lka_detector_benchmark(options.Dataset, options.Detectors, V);
    if options.ClosedLoop
        n = numel(options.Detectors);
        T.InLaneRatio = nan(n, 1);
        T.MaxLatOffset_m = nan(n, 1);
        T.MinWheelMargin_m = nan(n, 1);
        T.ClosedLoop = repmat("completed", n, 1);
        for d = 1:n
            try
                r = run_lka_vision(Scenarios={'S1_Nominal'}, Detectors=options.Detectors(d), Video=true);
                T.InLaneRatio(d) = r.kpi.InLaneRatio;
                T.MaxLatOffset_m(d) = r.kpi.MaxLatOffset_m;
                T.MinWheelMargin_m(d) = r.kpi.MinWheelMargin_m;
            catch ME
                % Unreal stops when the ego leaves the road mesh (ground following fails):
                % the detector lost the lane and the car ran off the track.
                T.ClosedLoop(d) = "stopped: vehicle left the road";
                fprintf('\n[benchmark] %s closed loop stopped: %s\n', options.Detectors{d}, strtok(ME.message, newline));
            end
        end
    end

    if ~isfolder(options.OutDir), mkdir(options.OutDir); end
    writetable(T, fullfile(options.OutDir, 'detector_benchmark.csv'));
    fid = fopen(fullfile(options.OutDir, 'detector_benchmark.md'), 'w');
    names = T.Properties.VariableNames;
    fprintf(fid, '| %s |\n|%s\n', strjoin(names, ' | '), repmat('---|', 1, numel(names)));
    for i = 1:height(T)
        vals = cellfun(@(c) fmt(T{i, c}), names, 'UniformOutput', false);
        fprintf(fid, '| %s |\n', strjoin(vals, ' | '));
    end
    fclose(fid);

    f = figure('Color', 'w', 'Position', [100 100 1100 420]);
    try
        theme(f, 'light');
    catch
    end
    tiledlayout(f, 1, 3);
    nexttile; bar(categorical(T.Detector), [T.F1 T.PixelAccuracy]); legend('F1', 'pixel accuracy'); title('Segmentation');
    nexttile; bar(categorical(T.Detector), [T.MaeY5_m T.MaeY10_m T.MaeY20_m]); legend('5 m', '10 m', '20 m'); title('Boundary error [m]');
    nexttile; bar(categorical(T.Detector), T.MsMean); title('Detector time [ms/frame]');
    exportgraphics(f, fullfile(options.OutDir, 'detector_benchmark.png'), 'Resolution', 150);
    close(f);
    disp(T);
end

function s = fmt(v)
    if isstring(v) || ischar(v)
        s = char(v);
    else
        s = sprintf('%.3g', v);
    end
end
