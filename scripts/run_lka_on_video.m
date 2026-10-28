function out = run_lka_on_video(videoFile, options)
% RUN_LKA_ON_VIDEO Lane detection + LKA (open loop) on a recorded lane video.
%
% Syntax:
%   out = run_lka_on_video('track.mp4')
%   out = run_lka_on_video('track.mp4', Detector='cnn', Camera='auto', HFovDeg=90)
%   out = run_lka_on_video('zed.mp4', StereoEye='left', SpeedLog='speed.csv')
%
% For every frame (resampled to the LKA rate, 20 Hz): lane detection (classic / team CNN
% / YOLOP) -> metric bird's-eye -> left and right lane lines; vehicle motion from the
% road texture (bird's-eye visual odometry) unless a speed log is given; then the
% unchanged LKA_Controller is replayed open loop (lka_replay_controller): the car in the
% video does not react, so the steering and turning radius are what the LKA WOULD
% command at each moment.
%
% Name-value options:
%   Detector   'cnn' (default when the team model and an ONNX back end exist) | 'classic' | 'yolop'
%   Camera     camera struct (fields as LKA.Vision.Camera), or 'auto' to estimate the pitch
%              from the lane lines (unknown mount)
%   HFovDeg    horizontal field of view used to build the intrinsics with Camera='auto'
%              (default 70, a phone main camera; simulator camera 60; ZED 2i HD720 ~ 87)
%   LaneWidth  known lane width [m] -> the camera height is solved from it (Camera='auto').
%              Default NaN: CameraHeight is trusted and the lane width is measured. One of
%              the two must be known: a single camera cannot tell a narrow lane seen from
%              low down from a wide lane seen from high up.
%   CameraHeight  lens height above the road [m] with Camera='auto' (default 1.2)
%   MaxRange   furthest lane point used [m] (default 20; tight bends, low-resolution video)
%   StereoEye  'none' | 'left' | 'right': crop one eye of a side-by-side stereo video
%   Speed      constant speed [m/s] instead of visual odometry (optional)
%   SpeedLog   CSV with columns time_s, speed_mps [, yaw_rate_radps] (optional)
%   StartTime, EndTime  [s] (default whole video)
%   OutDir     default results/video/<video name>
%   Video      write the annotated video lka_video_annotated.mp4 (default true)
%   UseGpu     run the network, bird's-eye warps and odometry on the GPU (default true)
%   Batch      frames per network call (default 0 = 8 on the GPU, 1 otherwise)
%   Monitor    publish live telemetry for the dashboard (lka_monitor_publish, default true)
%   TrackProfile  'tight' (default: painted test lane with bends down to MinRadius, e.g. the
%              college snake track) | 'official' (aBaja LKA track, R 127 m+): see
%              lka_set_track_profile. With 'official' a 3-8 m radius bend comes out
%              almost straight.
%   MinRadius  tightest bend radius of the track [m] for TrackProfile='tight' (default 3)
%   CalibrationFrames  'spread' (default: AutoFrames frames over the whole clip, so the
%              bends average out) | 'start' (the first AutoFrames frames at 20 Hz, for a
%              clip that starts on a straight)
%   ShapeCheck measure each frame how far the fitted lines are from the painted lines
%              (lka_lane_shape_error, default true): the straight-lane alarm
% Output: struct with Table (one row per processed frame, also lka_video_results.csv),
%   Camera (used / estimated), Replay (controller outputs). Figures lka_video_overview.png
%   (signals over time) and lka_video_path.png (dead-reckoned path with the detected lines).

    arguments
        videoFile (1,:) char
        options.Detector char = ''
        options.Camera = []
        options.HFovDeg (1,1) double = 70
        options.LaneWidth (1,1) double = NaN
        options.CameraHeight (1,1) double = 1.2
        options.MaxRange (1,1) double = 20
        options.StereoEye char {mustBeMember(options.StereoEye, {'none', 'left', 'right'})} = 'none'
        options.Speed double = []
        options.SpeedLog char = ''
        options.StartTime (1,1) double = 0
        options.EndTime (1,1) double = inf
        options.OutDir char = ''
        options.Video (1,1) logical = true
        options.AutoFrames (1,1) double = 20
        options.UseGpu (1,1) logical = true
        options.Batch (1,1) double = 0
        options.Monitor (1,1) logical = true
        options.TrackProfile char {mustBeMember(options.TrackProfile, {'tight', 'official'})} = 'tight'
        options.MinRadius (1,1) double {mustBePositive} = 3
        options.CalibrationFrames char {mustBeMember(options.CalibrationFrames, {'spread', 'start'})} = 'spread'
        options.ShapeCheck (1,1) logical = true
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    run(fullfile(root, 'config', 'startup.m'));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params; create_lka_buses;');
    LKA = evalin('base', 'LKA');
    Vcal = LKA.Vision;                 % camera auto-calibration keeps the long default fit
    LKA = lka_set_track_profile(LKA, options.TrackProfile, MinRadius=options.MinRadius);
    V = LKA.Vision;
    [~, name] = fileparts(videoFile);
    if isempty(options.OutDir)
        options.OutDir = fullfile(root, 'results', 'video', name);
    end
    if ~isfolder(options.OutDir), mkdir(options.OutDir); end
    if isempty(options.Detector)
        options.Detector = 'classic';
        if lka_onnx_runtime_available() && isfile(fullfile(V.DetectorDir, 'team_cnn.onnx'))
            options.Detector = 'cnn';
        end
    end
    V.Detector = options.Detector;
    det = lka_detector_create(V);
    fprintf('[LKA video] %s | detector %s | track profile %s\n', videoFile, det.Name, options.TrackProfile);

    % ---------------- frames at (up to) the LKA rate
    vr = VideoReader(videoFile);
    tEnd = min(options.EndTime, vr.Duration);
    readNext = makeReader(vr, options.StartTime, tEnd, LKA.Ts, options.StereoEye);

    % ---------------- camera
    first = readNext();
    if isempty(first.img)
        error('lka:video:empty', 'No frames between %.1f and %.1f s.', options.StartTime, tEnd);
    end
    if ischar(options.Camera) || isstring(options.Camera)
        C0 = V.Camera;
        [h, w, ~] = size(first.img);
        f = (w / 2) / tand(options.HFovDeg / 2);
        C0.ImageSize = [h w]; C0.PrincipalPoint = [w h] / 2; C0.FocalLength = [f f];
        C0.Height = options.CameraHeight;
        imgs = calibrationFrames(videoFile, options.StartTime, tEnd, options.AutoFrames, LKA.Ts, ...
            options.StereoEye, strcmp(options.CalibrationFrames, 'spread'));
        probs = cellfun(@(im) det.Detect(fitToCamera(im, C0)), imgs, 'UniformOutput', false);
        Vcal.Ts = LKA.Ts;
        [C, cal] = lka_autocalibrate_camera(probs, C0, Vcal, LaneWidth=options.LaneWidth);
        W = cal.LaneWidth;                       % lane width for the search and plausibility gates
        LKA = lka_set_lane_width(LKA, W, MinFactor=0.5, MaxFactor=2);   % lanes widen in bends
        V.Track = LKA.Vision.Track;
        fprintf('[LKA video] auto-calibrated camera: pitch %.1f deg, height %.2f m (f %.0f px), lane width %.2f m (%d frames, %s)\n', ...
            C.PitchDeg, C.Height, C.FocalLength(1), W, numel(imgs), options.CalibrationFrames);
    elseif isstruct(options.Camera)
        C = options.Camera;
    else
        C = V.Camera;
    end
    V.Camera = C;
    cam = lka_camera_model(C, V.Bev);
    cam.XFar = min([cam.XFar, det.MaxRange, options.MaxRange]);

    % ---------------- pass 1: detection + odometry, Batch frames at a time
    % network, bird's-eye warps and odometry on the GPU; the lane search / fit (sequential,
    % it tracks from frame to frame) on the CPU.
    useGpu = options.UseGpu && canUseGPU();
    nb = options.Batch;
    if nb <= 0
        nb = 1 + 7 * det.OnGpu;
    end
    shortName = detectorLabel(det);
    if nb > 1
        det.DetectBatch(zeros([C.ImageSize 3 nb], 'uint8'));      % trace the batched network once
    end
    fprintf('[LKA video] %s, %d frame(s) per network call, warp/odometry on the %s\n', ...
        shortName, nb, ternary(useGpu, 'GPU', 'CPU'));
    state = lka_vision_state_init(V);
    tf = zeros(0, 1); sensors = struct('Left', {}, 'Right', {}, 'FrameValid', {});
    previews = {}; overlays = {}; odo = zeros(0, 4); stageMs = zeros(0, 3); shapeErr = zeros(0, 2);
    useOdometry = isempty(options.Speed) && isempty(options.SpeedLog);
    gPrev = [];
    nTotal = max(1, round((tEnd - max(options.StartTime, first.t)) / LKA.Ts) + 1);
    if options.Monitor
        lka_monitor_publish('start', struct('Name', name, 'Kind', 'video', 'Detector', shortName, ...
            'Gpu', useGpu, 'OutDir', options.OutDir, 'Frames', nTotal, 'Duration', tEnd - options.StartTime, ...
            'Phase', 'detecting lanes'));
    end
    fr = first;
    t0 = tic;
    while ~isempty(fr.img)
        imgs = zeros([C.ImageSize 3 nb], 'uint8');
        tc = zeros(nb, 1);
        m = 0;
        while m < nb && ~isempty(fr.img)
            m = m + 1;
            imgs(:, :, :, m) = fitToCamera(fr.img, C);
            tc(m) = fr.t;
            fr = readNext();
        end
        imgs = imgs(:, :, :, 1:m);
        tc = tc(1:m);
        ta = tic;
        [probs, info] = det.DetectBatch(imgs);
        tDet = toc(ta);
        tb = tic;
        P = probs;
        if useGpu
            P = gpuArray(P);
        end
        bev = gather(lka_bev_warp(cam, P));               % rows x cols x pages x m
        odoChunk = nan(m, 4);
        if useOdometry
            g = im2single(imgs);
            if useGpu
                g = gpuArray(g);
            end
            gray = 0.298936 * g(:, :, 1, :) + 0.587043 * g(:, :, 2, :) + 0.114021 * g(:, :, 3, :);
            G = lka_bev_warp(cam, reshape(gray, C.ImageSize(1), C.ImageSize(2), m));
            if isempty(gPrev)
                prevStack = G(:, :, 1:m - 1); curStack = G(:, :, 2:m); off = 1;
            else
                prevStack = cat(3, gPrev, G(:, :, 1:m - 1)); curStack = G; off = 0;
            end
            if ~isempty(curStack)
                mo = lka_bev_odometry(prevStack, curStack, cam);
                odoChunk(1 + off:end, :) = [mo.dx' mo.dy' mo.dpsi' mo.Peak'];
            end
            gPrev = G(:, :, m);
        end
        tWarp = toc(tb);
        tc2 = tic;
        for j = 1:m
            [s, state] = lka_vision_lane_step(bev(:, :, :, j), cam, V, state, LKA.Ts);
            sensors(end + 1, 1) = s; %#ok<AGROW>
            shapeErr(end + 1, :) = [NaN NaN]; %#ok<AGROW>
            if options.ShapeCheck
                shapeErr(end, :) = lka_lane_shape_error(bev(:, :, :, j), cam, s);
            end
            previews{end + 1, 1} = single(imresize(max(bev(:, :, :, j), [], 3), V.PreviewSize)); %#ok<AGROW>
            overlays{end + 1, 1} = struct('Lanes', {info(j).Lanes}, 'Ids', info(j).Ids); %#ok<AGROW>
        end
        tStep = toc(tc2);
        tf = [tf; tc]; %#ok<AGROW>
        odo = [odo; odoChunk]; %#ok<AGROW>
        stageMs = [stageMs; repmat(1000 * [tDet tWarp tStep] / m, m, 1)]; %#ok<AGROW>
        k = numel(tf);
        if options.Monitor
            hud = struct('SteerDeg', 0, 'State', 1, 'Time', tc(end), 'Detector', shortName, 'Ms', sum(stageMs(end, :)));
            lka_monitor_publish('sample', liveSample(tc(end), s, stageMs(end, :), odoChunk(end, :), LKA.Ts, k, nTotal), ...
                @() lka_annotate_frame(imgs(:, :, :, m), previews{end}, s, hud, cam, overlays{end}));
        end
        if floor(k / 100) > floor((k - m) / 100)
            fprintf('[LKA video] %d frames (%.1f s of video), %.0f ms/frame (detect %.0f, warp+odometry %.0f, lane fit %.0f)\n', ...
                k, tc(end), 1000 * toc(t0) / k, mean(stageMs(:, 1)), mean(stageMs(:, 2)), mean(stageMs(:, 3)));
        end
    end
    n = numel(tf);

    % ---------------- speed and yaw rate
    dt = [LKA.Ts; diff(tf)];
    if ~isempty(options.SpeedLog)
        L = readtable(options.SpeedLog);
        speed = interp1(L{:, 1}, L{:, 2}, tf, 'linear', 'extrap');
        yawRate = zeros(n, 1);
        if width(L) >= 3
            yawRate = interp1(L{:, 1}, L{:, 3}, tf, 'linear', 'extrap');
        end
        source = "log";
    elseif ~isempty(options.Speed)
        speed = options.Speed * ones(n, 1);
        yawRate = zeros(n, 1);
        source = "constant";
    else
        speed = hypot(odo(:, 1), odo(:, 2)) ./ dt;
        yawRate = odo(:, 3) ./ dt;
        bad = ~isfinite(speed) | ~isfinite(yawRate);   % first frame, unmatched pairs
        if all(bad)
            warning('lka:video:noOdometry', 'Visual odometry found no motion; speed and yaw set to 0.');
            speed(:) = 0; yawRate(:) = 0;
        elseif any(bad)
            speed = fillmissing(speed, 'linear', 'EndValues', 'nearest');
            yawRate = fillmissing(yawRate, 'linear', 'EndValues', 'nearest');
            if nnz(bad(2:end)) > 0                    % frame 1 has no predecessor
                fprintf('[LKA video] odometry: %d of %d frame pairs unmatched, interpolated\n', nnz(bad(2:end)), n - 1);
            end
        end
        speed = movmean(movmedian(speed, 5), 5);
        yawRate = movmean(movmedian(yawRate, 5), 5);
        source = "visual odometry";
    end

    % ---------------- controller replay on the 20 Hz grid
    tg = (tf(1):LKA.Ts:tf(end))';
    ig = discretize(tg + 1e-9, [tf; inf]);      % latest frame at or before each grid time
    rep = lka_replay_controller(tg, sensors(ig), speed(ig), yawRate(ig), LKA);
    back = @(v) interp1(tg, v, tf, 'previous', 'extrap');

    % ---------------- results table
    Ls = [sensors.Left]'; Rs = [sensors.Right]';
    kRoad = back(rep.LaneCurvature);
    T = table(tf, [Ls.LateralOffset]', rad2deg([Ls.HeadingAngle]'), [Ls.Curvature]', [Ls.Strength]', ...
        [Rs.LateralOffset]', rad2deg([Rs.HeadingAngle]'), [Rs.Curvature]', [Rs.Strength]', ...
        back(rep.LaneOffset), rad2deg(back(rep.LaneHeading)), kRoad, clipRadius(kRoad), ...
        rad2deg(back(rep.SteerCmd)), back(rep.TurnRadius), back(rep.State), ...
        speed, 3.6 * speed, rad2deg(yawRate), rad2deg(cumtrapz(tf, yawRate)), odo(:, 4), ...
        stageMs(:, 1), stageMs(:, 2), stageMs(:, 3), ...
        [Ls.ViewRange]', [Rs.ViewRange]', shapeErr(:, 1), shapeErr(:, 2), ...
        'VariableNames', {'Time_s', 'LeftOffset_m', 'LeftHeading_deg', 'LeftCurvature_1pm', 'LeftStrength', ...
        'RightOffset_m', 'RightHeading_deg', 'RightCurvature_1pm', 'RightStrength', ...
        'LaneCentreOffset_m', 'HeadingError_deg', 'RoadCurvature_1pm', 'RoadRadius_m', ...
        'SteerCmd_deg', 'TurnRadius_m', 'LkaState', 'Speed_mps', 'Speed_kmh', 'YawRate_degps', ...
        'YawAngle_deg', 'OdometryPeak', 'Detect_ms', 'BevOdometry_ms', 'LaneFit_ms', ...
        'LeftViewRange_m', 'RightViewRange_m', 'LeftShapeError_m', 'RightShapeError_m'});
    writetable(T, fullfile(options.OutDir, 'lka_video_results.csv'));
    out = struct('Table', T, 'Camera', C, 'Replay', rep, 'SpeedSource', source, 'Detector', det.Name, ...
        'LaneWidth', V.Track.WidthInit, 'TrackProfile', options.TrackProfile);
    reportShape(shapeErr, options.TrackProfile);
    save(fullfile(options.OutDir, 'lka_video_results.mat'), 'out');
    plotOverview(T, source, det.Name, fullfile(options.OutDir, 'lka_video_overview.png'));
    plotPath(T, cam.XNear, source, fullfile(options.OutDir, 'lka_video_path.png'));

    % ---------------- pass 2: annotated video
    kCmd = 1 ./ T.TurnRadius_m;              % arc drawn = the HUD turning radius (after rate limit)
    if options.Video
        vw = VideoWriter(fullfile(options.OutDir, 'lka_video_annotated.mp4'), 'MPEG-4');
        % frames are kept at (up to) the 20 Hz grid, so the source spacing alternates on a
        % 30 fps clip (33 / 67 ms): the median gap gave 17 fps and an 18 % slow video
        vw.FrameRate = max(1, (n - 1) / max(tf(end) - tf(1), LKA.Ts));
        vw.Quality = 90;
        open(vw);
        closeVideo = onCleanup(@() close(vw));   % finalise the MP4 even when a frame fails
        if options.Monitor
            lka_monitor_publish('phase', 'writing annotated video');
        end
        readNext = makeReader(VideoReader(videoFile), options.StartTime, tEnd, LKA.Ts, options.StereoEye);
        for k = 1:n
            fr = readNext();
            if isempty(fr.img), break; end
            hud = struct('SteerDeg', T.SteerCmd_deg(k), 'State', T.LkaState(k), ...
                'TurnRadius', T.TurnRadius_m(k), 'SpeedKmh', T.Speed_kmh(k), 'CurvatureCmd', kCmd(k), ...
                'LaneOffset', T.LaneCentreOffset_m(k), 'HeadingDeg', T.HeadingError_deg(k), ...
                'Detector', shortName, 'Ms', sum(stageMs(k, :)), 'Time', T.Time_s(k));
            frame = lka_annotate_frame(fitToCamera(fr.img, C), previews{k}, sensors(k), hud, cam, overlays{k});
            writeVideo(vw, frame);
            if options.Monitor
                lka_monitor_publish('sample', controlSample(T(k, :), k, n), frame);
            end
        end
        clear closeVideo
    end
    if options.Monitor
        lka_monitor_publish('done', struct('OutDir', options.OutDir, 'Frames', n, ...
            'BothLines_pct', 100 * mean(T.LeftStrength > 0 & T.RightStrength > 0), ...
            'MeanMs', mean(sum(stageMs, 2))));
    end
    fprintf('[LKA video] %d frames -> %s\n', n, options.OutDir);
end

function s = detectorLabel(det)
% Short detector name for the HUD: text before ':' plus the back end in brackets.
    s = det.Name;
    b = regexp(s, '\[[^\]]*\]\s*$', 'match', 'once');
    if contains(s, ':')
        s = strtrim([extractBefore(s, ':') ' ' b]);
    end
end

function v = ternary(c, a, b)
    if c, v = a; else, v = b; end
end

function s = liveSample(t, sensor, ms, odo, Ts, k, n)
% Telemetry of pass 1 (detection): lane lines, timing, raw odometry speed.
    s = struct('Time', t, 'Frame', k, 'Frames', n, ...
        'Left', sensor.Left.LateralOffset * (sensor.Left.Strength > 0), 'LeftOk', sensor.Left.Strength > 0, ...
        'Right', sensor.Right.LateralOffset * (sensor.Right.Strength > 0), 'RightOk', sensor.Right.Strength > 0, ...
        'DetectMs', ms(1), 'WarpMs', ms(2), 'FitMs', ms(3), 'Speed', hypot(odo(1), odo(2)) / Ts * 3.6);
end

function s = controlSample(row, k, n)
% Telemetry of pass 2 (annotated video): the replayed LKA outputs.
    s = struct('Time', row.Time_s, 'Frame', k, 'Frames', n, 'Left', row.LeftOffset_m, 'LeftOk', row.LeftStrength > 0, ...
        'Right', row.RightOffset_m, 'RightOk', row.RightStrength > 0, 'Centre', row.LaneCentreOffset_m, ...
        'Heading', row.HeadingError_deg, 'Steer', row.SteerCmd_deg, 'Radius', row.TurnRadius_m, ...
        'State', row.LkaState, 'Speed', row.Speed_kmh, 'YawRate', row.YawRate_degps, 'Yaw', row.YawAngle_deg, ...
        'DetectMs', row.Detect_ms, 'WarpMs', row.BevOdometry_ms, 'FitMs', row.LaneFit_ms);
end

function readNext = makeReader(vr, tStart, tEnd, Ts, eye)
% Returns frames at the first video timestamp at or after each Ts grid time.
    if tStart > 0
        vr.CurrentTime = tStart;
    end
    nextT = tStart;
    readNext = @next;
    function fr = next()
        fr = struct('img', [], 't', NaN);
        while hasFrame(vr)
            t = vr.CurrentTime;
            img = readFrame(vr);
            if t > tEnd
                return
            end
            if t + 1e-6 >= nextT
                nextT = nextT + Ts * max(1, floor((t - nextT) / Ts) + 1);
                fr = struct('img', cropEye(img, eye), 't', t);
                return
            end
        end
    end
end

function imgs = calibrationFrames(videoFile, tStart, tEnd, n, Ts, eye, spread)
% Frames for the camera auto-calibration: n frames spread evenly over [tStart, tEnd]
% (bends to the left and right average out) or the first n frames at Ts.
    vr = VideoReader(videoFile);
    if spread
        ts = linspace(tStart, tEnd, n + 2);
        ts = ts(2:end - 1);
    else
        ts = tStart + (0:n - 1) * Ts;
    end
    imgs = {};
    for t = ts(ts < vr.Duration)
        vr.CurrentTime = t;
        if hasFrame(vr)
            imgs{end + 1} = cropEye(readFrame(vr), eye); %#ok<AGROW>
        end
    end
    if isempty(imgs)
        error('lka:video:empty', 'No frames for the camera calibration between %.1f and %.1f s.', tStart, tEnd);
    end
end

function reportShape(shapeErr, profile)
% Straight-lane alarm: the fitted lines must stay on the paint (lka_lane_shape_error).
    e = shapeErr(isfinite(shapeErr));
    if isempty(e)
        return
    end
    p90 = sortedPercentile(e, 0.9);
    fprintf('[LKA video] fitted line vs painted line: median %.2f m, 90 %% of lines within %.2f m\n', median(e), p90);
    if p90 > 0.25
        hint = '';
        if strcmp(profile, 'official')
            hint = ' Try TrackProfile=''tight''.';
        end
        warning('lka:video:laneShape', ['The fitted lane lines are more than %.2f m off the painted lines in ' ...
            '10 %% of the frames: the lane model does not follow this track (for example a bend tighter ' ...
            'than MinRadius).%s'], p90, hint);
    end
end

function v = sortedPercentile(x, p)
    x = sort(x(:));
    v = x(max(1, ceil(p * numel(x))));
end

function img = cropEye(img, eye)
    w = size(img, 2);
    switch eye
        case 'left',  img = img(:, 1:floor(w / 2), :);
        case 'right', img = img(:, floor(w / 2) + 1:end, :);
    end
    if size(img, 3) == 1
        img = repmat(img, 1, 1, 3);
    end
end

function img = fitToCamera(img, C)
    if ~isequal([size(img, 1) size(img, 2)], C.ImageSize)
        img = imresize(img, C.ImageSize);
    end
end

function R = clipRadius(k)
    R = 1 ./ k;
    R(abs(R) > 1000 | ~isfinite(R)) = 1000;
end

function plotOverview(T, source, detName, file)
    f = figure('Color', 'w', 'Position', [60 60 1400 900], 'Visible', 'off');
    try
        theme(f, 'light');
    catch
    end
    tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact');
    title(tl, sprintf('LKA on video (%s) - speed from %s', detName, source), 'Interpreter', 'none');
    nexttile; plot(T.Time_s, T.LaneCentreOffset_m, T.Time_s, T.LeftOffset_m, ':', T.Time_s, T.RightOffset_m, ':');
    grid on; ylabel('[m] (+ left)'); legend('lane centre (tracked)', 'left line', 'right line'); title('Lane position at the rear axle');
    nexttile; plot(T.Time_s, T.HeadingError_deg); grid on; ylabel('[deg]'); title('Lane heading relative to the vehicle');
    nexttile; semilogy(T.Time_s, abs(T.RoadRadius_m), T.Time_s, abs(T.TurnRadius_m), '.'); grid on; ylim([10 2000]);
    ylabel('|R| [m]'); legend('road radius (from lane)', 'LKA turning-radius command'); title('Radius (1000 m = straight)');
    nexttile; plot(T.Time_s, T.SteerCmd_deg); grid on; ylabel('[deg] (+ left)'); title('LKA road-wheel angle command');
    nexttile; plot(T.Time_s, T.Speed_kmh); grid on; ylabel('[km/h]'); yline(33, 'r--'); title('Vehicle speed');
    nexttile; yyaxis left; plot(T.Time_s, T.YawRate_degps); ylabel('yaw rate [deg/s]');
    yyaxis right; plot(T.Time_s, T.YawAngle_deg); ylabel('yaw angle [deg]'); grid on; title('Yaw (integrated from the start)');
    xlabel(tl, 'time [s]');
    exportgraphics(f, file, 'Resolution', 130);
    close(f);
end

function plotPath(T, xNear, source, file)
% Path integrated from speed and yaw rate (drifts slowly with visual odometry) and the
% detected lines at the first visible row xNear, placed in the same frame.
    t = T.Time_s;
    psi = deg2rad(T.YawAngle_deg);
    dt = [0; diff(t)];
    px = cumsum(T.Speed_mps .* cos(psi) .* dt);
    py = cumsum(T.Speed_mps .* sin(psi) .* dt);
    f = figure('Color', 'w', 'Position', [60 60 1300 650], 'Visible', 'off');
    try
        theme(f, 'light');
    catch
    end
    ax = axes(f); hold(ax, 'on');
    sides = {'Left', 'Right'}; colors = {[0 0.6 0], [0 0.45 0.9]};
    for k = 1:2
        off = T.([sides{k} 'Offset_m']); head = deg2rad(T.([sides{k} 'Heading_deg']));
        curv = T.([sides{k} 'Curvature_1pm']); ok = T.([sides{k} 'Strength']) > 0;
        yl = off + tan(head) * xNear + curv / 2 * xNear^2;
        wx = px + cos(psi) * xNear - sin(psi) .* yl;
        wy = py + sin(psi) * xNear + cos(psi) .* yl;
        plot(ax, wx(ok), wy(ok), '.', 'Color', colors{k}, 'MarkerSize', 6, 'DisplayName', [lower(sides{k}) ' line (detected)']);
    end
    plot(ax, px, py, 'k-', 'LineWidth', 1.5, 'DisplayName', 'vehicle (rear axle)');
    plot(ax, px(1), py(1), 'ko', 'MarkerFaceColor', 'g', 'DisplayName', 'start');
    axis(ax, 'equal'); axis(ax, 'tight'); grid(ax, 'on'); xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
    xlim(ax, xlim(ax) + [-3 3]); ylim(ax, ylim(ax) + [-3 3]);
    legend(ax, 'Location', 'best');
    title(ax, sprintf('Path (speed and yaw from %s) and detected lane lines, %.0f m driven', ...
        source, sum(T.Speed_mps .* dt)), 'Interpreter', 'none');
    exportgraphics(f, file, 'Resolution', 130);
    close(f);
end
