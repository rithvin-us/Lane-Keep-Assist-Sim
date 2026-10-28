function T = measure_lka_timing(options)
% MEASURE_LKA_TIMING Per-stage processing time of the camera LKA on this PC.
%
% Syntax:
%   T = measure_lka_timing()
%   T = measure_lka_timing(Repeats=30, Image='results/vision/calibration_raw.png')
%
% Times each stage of the camera chain on one real Unreal frame of the official LKA
% track (1280x720): every available detector (classic, the ported LaneAssist row
% histograms, YOLOP, team UFLDv2), the bird's-eye warp, the lane search + fit, and the
% annotation of the video frame. The
% median over Repeats runs (after one warm-up) is reported with the frame rate that
% stage alone would allow. Writes results/timing/lka_timing.csv.
%
% The LKA itself runs at a fixed 20 Hz (LKA.Ts = 0.05 s): camera, detector and
% controller. A detector is fast enough when detector + warp + fit < 50 ms.

    arguments
        options.Repeats (1,1) double = 30
        options.Image char = ''
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    run(fullfile(root, 'config', 'startup.m'));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params;');
    V = evalin('base', 'LKA.Vision');
    if isempty(options.Image)
        options.Image = fullfile(root, 'results', 'vision', 'calibration_raw.png');
    end
    img = imread(options.Image);
    cam = lka_camera_model(V.Camera, V.Bev);

    detectors = {'classic', 'laneassist'};
    if lka_onnx_runtime_available()
        detectors{end + 1} = 'yolop';
        if isfile(fullfile(V.DetectorDir, 'team_cnn.onnx'))
            detectors{end + 1} = 'cnn';
        end
    end

    rows = {};
    prob = [];
    for d = 1:numel(detectors)
        Vd = V; Vd.Detector = detectors{d};
        det = lka_detector_create(Vd);
        p = det.Detect(img);                                     % warm-up
        ms = timeit(@() det.Detect(img), options.Repeats);
        rows(end + 1, :) = {sprintf('detector: %s', det.Name), ms}; %#ok<AGROW>
        if d == 1
            prob = p;
        end
    end
    bev = lka_bev_warp(cam, prob);
    rows(end + 1, :) = {'bird''s-eye warp (lka_bev_warp)', timeit(@() lka_bev_warp(cam, prob), options.Repeats)};
    rows(end + 1, :) = {'lane search + robust fit (lka_vision_lane_step)', ...
        timeit(@() lka_vision_lane_step(bev, cam, V, lka_vision_state_init(V), V.Ts), options.Repeats)};
    sensor = lka_vision_lane_step(bev, cam, V, lka_vision_state_init(V), V.Ts);
    preview = imresize(max(bev, [], 3), V.PreviewSize);
    rows(end + 1, :) = {'video annotation (recorder only)', ...
        timeit(@() lka_annotate_frame(img, preview, sensor, struct('SteerDeg', 0, 'State', 2), cam), options.Repeats)};

    T = cell2table(rows, 'VariableNames', {'Stage', 'Median_ms'});
    T.MaxRate_Hz = 1000 ./ T.Median_ms;
    disp(T);
    outDir = fullfile(root, 'results', 'timing');
    if ~isfolder(outDir), mkdir(outDir); end
    writetable(T, fullfile(outDir, 'lka_timing.csv'));
end

function ms = timeit(f, n)
    t = zeros(n, 1);
    for k = 1:n
        t0 = tic;
        f();
        t(k) = toc(t0);
    end
    ms = 1000 * median(t);
end
