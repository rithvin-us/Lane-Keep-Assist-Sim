function out = run_lka_live(options)
% RUN_LKA_LIVE Live camera LKA: lane maps from the Python lane node -> lane fit -> LKA steering.
%
% Syntax:
%   out = run_lka_live()                                          % node on 127.0.0.1:5555
%   out = run_lka_live(CameraHeight=0.95, PitchDeg=10, CameraX=1.2, LaneWidth=2.0)
%   out = run_lka_live(Speed=3, Duration=120)
%
% Start the node first (perception/Mobilenet_Approach/realtime/lane_node.py, see
% docs/real_time_guide.md). Per frame the node sends the lane-line probability map and the
% camera frame at the network size (480 x 288); here they go through the unchanged MATLAB
% chain: metric bird's-eye (lka_bev_warp) -> lane search and fit (lka_vision_lane_step) ->
% the controller functions LKA_Controller is built from, in the same order
% (lka_validate_boundaries, lane-width filter, lka_estimate_lane_center,
% lka_lane_tracker_step, lka_pure_pursuit, lka_curvature_limits, lka_curvature_to_steer).
% The live steering is that pure-pursuit chain WITHOUT the Stateflow supervisor and the
% actuator manager: a preview. When the run stops (Duration, node closed, Ctrl+C) the whole
% session is replayed through the real LKA_Controller model (lka_replay_controller) and
% that is the result to judge (live_results.csv, live_overview.png).
%
% Camera: intrinsics come from the node (its --calib file or --hfov); the mount has to be
% given here and measured on the vehicle (docs/vehicle_measurements.md):
%   CameraHeight [m] lens above ground, PitchDeg [deg] nose-down, CameraX [m] ahead of the
%   rear axle. LaneWidth [m] of the track (default: the tight college profile, 2.0 m).
% Speed: constant [m/s], or NaN (default) for bird's-eye visual odometry from the frames.
% The live run is published to the dashboard (start_lka_dashboard, http://127.0.0.1:8050).

    arguments
        options.Host char = '127.0.0.1'
        options.Port (1,1) double = 5555
        options.CameraHeight (1,1) double = 1.2
        options.PitchDeg (1,1) double = 8
        options.CameraX (1,1) double = 1.6
        options.HFovDeg (1,1) double = 102        % used only when the node sends no intrinsics
        options.LaneWidth (1,1) double = 2.0
        options.TrackProfile char {mustBeMember(options.TrackProfile, {'tight', 'official'})} = 'tight'
        options.MinRadius (1,1) double {mustBePositive} = 3
        options.Speed (1,1) double = NaN
        options.Duration (1,1) double = inf       % [s]
        options.MaxRange (1,1) double = 20
        options.OutDir char = ''
        options.Monitor (1,1) logical = true
        options.Replay (1,1) logical = true       % full LKA_Controller replay at the end
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    evalin('base', sprintf('run(''%s'');', fullfile(root, 'config', 'startup.m')));   % script: paths and variables in base
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params; create_lka_buses;');
    LKA = evalin('base', 'LKA');
    LKA = lka_set_track_profile(LKA, options.TrackProfile, MinRadius=options.MinRadius);
    LKA = lka_set_lane_width(LKA, options.LaneWidth, MinFactor=0.5, MaxFactor=2);
    V = LKA.Vision;
    if isempty(options.OutDir)
        options.OutDir = fullfile(root, 'results', 'live', char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
    end
    if ~isfolder(options.OutDir), mkdir(options.OutDir); end

    % ---------------- connect, camera model from the node's hello
    fprintf('[LKA live] connecting to the lane node on %s:%d ...\n', options.Host, options.Port);
    tc = tcpclient(options.Host, options.Port, 'Timeout', 10, 'ConnectTimeout', 10);
    hello = jsondecode(char(read(tc, double(typecast(read(tc, 4, 'uint8'), 'uint32')), 'uint8')));
    C = V.Camera;
    C.ImageSize = [hello.height hello.width];
    if isfield(hello, 'fx')
        C.FocalLength = [hello.fx hello.fy];
        C.PrincipalPoint = [hello.cx hello.cy];
    else
        f = (hello.width / 2) / tand(options.HFovDeg / 2);
        C.FocalLength = [f f * hello.height / hello.width * hello.eye(1) / hello.eye(2)];
        C.PrincipalPoint = [hello.width hello.height] / 2;
    end
    C.Height = options.CameraHeight; C.PitchDeg = options.PitchDeg; C.X = options.CameraX;
    V.Camera = C;
    cam = lka_camera_model(C, V.Bev);
    cam.XFar = min(cam.XFar, options.MaxRange);
    fprintf('[LKA live] %s, %dx%d, f = [%.0f %.0f] px, camera %.2f m high, pitch %.1f deg, %.2f m ahead of the rear axle\n', ...
        hello.source, hello.width, hello.height, C.FocalLength, C.Height, C.PitchDeg, C.X);

    % ---------------- state
    state = lka_vision_state_init(V);
    kf = struct('x', zeros(4, 1), 'P', eye(4), 'st', zeros(3, 1));
    mem = struct('offL', 0, 'offR', 0, 'validL', false, 'validR', false, ...
        'width', LKA.Lane.WidthNominal, 'kappa', 0, 'speed', 0);
    header = 33;                                  % '<4sIdfffHHB' (lane_node.py)
    n = 0; tPrev = NaN; gPrev = []; tStart = tic;
    log = struct('t', zeros(0, 1), 'sensor', struct('Left', {}, 'Right', {}, 'FrameValid', {}), ...
        'speed', zeros(0, 1), 'yaw', zeros(0, 1), 'row', zeros(0, 13));
    if options.Monitor
        lka_monitor_publish('start', struct('Name', char(hello.source), 'Kind', 'live', 'Detector', V.Detector, ...
            'Gpu', false, 'OutDir', options.OutDir, 'Frames', 0, 'Duration', options.Duration, 'Phase', 'live'));
    end
    % the session is saved and replayed when the function exits, also on Ctrl+C: the
    % cleanup holds a handle (box) with a snapshot of the log, refreshed every 20 frames
    box = containers.Map('KeyType', 'char', 'ValueType', 'any');
    box('S') = [];
    finisher = onCleanup(@() finishSession(box('S')));
    fprintf('[LKA live] running (Ctrl+C stops; results go to %s)\n', options.OutDir);

    while toc(tStart) < options.Duration
        try
            h = read(tc, header, 'uint8');
            tRead0 = tic;
        catch
            break                                 % node closed or timed out
        end
        if numel(h) < header || ~isequal(char(h(1:4)), 'LKA1'), break, end
        fid = double(typecast(h(5:8), 'uint32'));
        tCap = typecast(h(9:16), 'double');
        preMs = double(typecast(h(17:20), 'single')); netMs = double(typecast(h(21:24), 'single'));
        ageMs = double(typecast(h(25:28), 'single'));   % camera capture -> sent by the node
        H = double(typecast(h(29:30), 'uint16')); W = double(typecast(h(31:32), 'uint16'));
        lane = read(tc, H * W, 'uint8');
        rgb = [];
        if h(33)
            rgb = permute(reshape(read(tc, H * W * 3, 'uint8'), 3, W, H), [3 2 1]);
        end
        tRead = toc(tRead0);
        t0 = tic;
        prob = single(reshape(lane, W, H)') / 255;
        dt = LKA.Ts;
        if isfinite(tPrev), dt = max(1e-3, tCap - tPrev); end
        tPrev = tCap;

        % perception: bird's-eye, lane search and fit
        bev = lka_bev_warp(cam, prob);
        tWarp = toc(t0);
        [s, state] = lka_vision_lane_step(bev, cam, V, state, dt);
        tFit = toc(t0) - tWarp;

        % speed / yaw rate: constant, or bird's-eye odometry between consecutive frames
        speed = options.Speed; yawRate = 0;
        if isnan(speed)
            speed = mem.speed;
            if ~isempty(rgb)
                g = lka_bev_warp(cam, im2single(rgb2gray(rgb)));
                if ~isempty(gPrev)
                    mo = lka_bev_odometry(gPrev, g, cam);
                    if isfinite(mo.dx)
                        speed = mem.speed + 0.3 * (hypot(mo.dx, mo.dy) / dt - mem.speed);
                        yawRate = mo.dpsi / dt;
                    end
                end
                gPrev = g;
            end
        end
        mem.speed = speed;

        % controller preview (the LKA_Controller functions, no supervisor)
        [steer, kappa, est, quality, mem, kf] = controllerStep(s, speed, yawRate, dt, LKA, mem, kf);
        tTotal = toc(t0);

        n = n + 1;
        log.t(n, 1) = tCap; log.sensor(n, 1) = s; log.speed(n, 1) = speed; log.yaw(n, 1) = yawRate;
        log.row(n, :) = [fid tCap s.Left.LateralOffset * (s.Left.Strength > 0) s.Right.LateralOffset * (s.Right.Strength > 0) ...
            est.CenterOffset rad2deg(est.HeadingAngle) rad2deg(steer) radiusOf(kappa) speed ...
            preMs + netMs 1000 * tWarp 1000 * tFit 1000 * tTotal];
        if options.Monitor
            hud = struct('SteerDeg', rad2deg(steer), 'State', 1 + (quality > 0.5), 'TurnRadius', radiusOf(kappa), ...
                'SpeedKmh', 3.6 * speed, 'CurvatureCmd', kappa, 'LaneOffset', est.CenterOffset, ...
                'HeadingDeg', rad2deg(est.HeadingAngle), 'Detector', 'mobilenet (node)', ...
                'Ms', preMs + netMs + 1000 * tTotal, 'Time', tCap);
            img = rgb;
            if isempty(img), img = repmat(uint8(255 * prob), 1, 1, 3); end
            sample = struct('Time', tCap, 'Frame', n, 'Frames', 0, ...
                'Left', log.row(n, 3), 'LeftOk', s.Left.Strength > 0, 'Right', log.row(n, 4), 'RightOk', s.Right.Strength > 0, ...
                'Centre', est.CenterOffset, 'Heading', rad2deg(est.HeadingAngle), 'Steer', rad2deg(steer), ...
                'Radius', radiusOf(kappa), 'State', hud.State, 'Speed', 3.6 * speed, 'YawRate', rad2deg(yawRate), ...
                'DetectMs', preMs + netMs, 'WarpMs', 1000 * tWarp, 'FitMs', 1000 * (tTotal - tWarp), ...
                'AgeMs', ageMs + 1000 * (toc(t0) + tRead));   % capture -> steering command
            lka_monitor_publish('sample', sample, ...
                @() lka_annotate_frame(img, single(imresize(max(bev, [], 3), V.PreviewSize)), s, hud, cam));
        end
        if mod(n, 20) == 0
            box('S') = struct('n', n, 'log', log, 'options', options, 'LKA', LKA, 'C', C);
        end
        if mod(n, 40) == 0
            r = log.row(max(1, n - 39):n, :);
            fprintf('[LKA live] %5d frames | lines L %3.0f%% R %3.0f%% | steer %+5.1f deg | node %4.1f ms + MATLAB %4.1f ms (warp %3.1f fit %3.1f) | %.1f km/h\n', ...
                n, 100 * mean(r(:, 3) ~= 0), 100 * mean(r(:, 4) ~= 0), r(end, 7), median(r(:, 10)), median(r(:, 13)), ...
                median(r(:, 11)), median(r(:, 12)), 3.6 * speed);
        end
    end
    box('S') = struct('n', n, 'log', log, 'options', options, 'LKA', LKA, 'C', C); %#ok<NASGU> read by the cleanup
    out = struct('OutDir', options.OutDir, 'Frames', n);
end

function finishSession(S)
% Writes live_frames.csv, replays the session through LKA_Controller (live_results.csv,
% live_overview.png) and saves live_session.mat.
    if isempty(S) || S.n == 0
        fprintf('[LKA live] no frames received\n');
        return
    end
    n = S.n; log = S.log; options = S.options; LKA = S.LKA; C = S.C;
    log.row = log.row(1:n, :); log.t = log.t(1:n); log.sensor = log.sensor(1:n);
    log.speed = log.speed(1:n); log.yaw = log.yaw(1:n);
    T = array2table(log.row, 'VariableNames', {'Frame', 'Time_s', 'LeftOffset_m', 'RightOffset_m', ...
        'LaneCentreOffset_m', 'HeadingError_deg', 'PreviewSteer_deg', 'PreviewTurnRadius_m', 'Speed_mps', ...
        'Node_ms', 'Bev_ms', 'LaneFit_ms', 'Matlab_ms'});
    T.Time_s = T.Time_s - T.Time_s(1);
    writetable(T, fullfile(options.OutDir, 'live_frames.csv'));
    fprintf('[LKA live] %d frames, %.1f s: both lines %.1f %%, node %.1f ms + MATLAB %.1f ms per frame (median)\n', ...
        n, T.Time_s(end), 100 * mean(T.LeftOffset_m ~= 0 & T.RightOffset_m ~= 0), median(T.Node_ms), median(T.Matlab_ms));
    if options.Replay && n >= 10
        % the real LKA_Controller over the session, on the 20 Hz grid
        tf = T.Time_s;
        tg = (0:LKA.Ts:tf(end))';
        ig = discretize(tg + 1e-9, [tf; inf]);
        rep = lka_replay_controller(tg, log.sensor(ig), log.speed(ig), log.yaw(ig), LKA);
        R = table(tg, rad2deg(rep.SteerCmd), rep.TurnRadius, rep.State, rep.LaneOffset, rad2deg(rep.LaneHeading), ...
            'VariableNames', {'Time_s', 'SteerCmd_deg', 'TurnRadius_m', 'LkaState', 'LaneCentreOffset_m', 'HeadingError_deg'});
        writetable(R, fullfile(options.OutDir, 'live_results.csv'));
        f = figure('Visible', 'off', 'Color', 'w', 'Position', [60 60 1200 700]);
        tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact');
        title(tl, 'Live camera session: LKA_Controller replay', 'Interpreter', 'none');
        nexttile; plot(R.Time_s, R.SteerCmd_deg, T.Time_s, T.PreviewSteer_deg, ':'); grid on;
        ylabel('steer [deg]'); legend('LKA_Controller', 'live preview', 'Interpreter', 'none');
        nexttile; plot(T.Time_s, T.LeftOffset_m, '.', T.Time_s, T.RightOffset_m, '.', R.Time_s, R.LaneCentreOffset_m); grid on;
        ylabel('[m] (+ left)'); legend('left line', 'right line', 'lane centre (tracked)');
        nexttile; plot(T.Time_s, T.Node_ms, T.Time_s, T.Matlab_ms); yline(1000 * LKA.Ts, 'r--', '20 Hz budget');
        grid on; ylabel('[ms]'); xlabel('t [s]'); legend('node (pre + network)', 'MATLAB (warp, fit, controller)');
        exportgraphics(f, fullfile(options.OutDir, 'live_overview.png'));
        close(f);
        fprintf('[LKA live] LKA_Controller replay: max |steer| %.1f deg, LKA active %.0f %% -> %s\n', ...
            max(abs(R.SteerCmd_deg)), 100 * mean(R.LkaState >= 2), options.OutDir);
    end
    save(fullfile(options.OutDir, 'live_session.mat'), 'log', 'C', 'options');
    if options.Monitor
        lka_monitor_publish('done', struct('OutDir', options.OutDir, 'Frames', n));
    end
end

function [steer, kappa, est, quality, mem, kf] = controllerStep(s, v, yawRate, dt, LKA, mem, kf)
% One step of the LKA_Controller signal chain (scripts/build_lka_models.m), supervisor excluded.
    [validL, validR, widthMeas, widthValid] = lka_validate_boundaries(s.Left, s.Right, ...
        mem.offL, mem.offR, mem.validL, mem.validR, LKA.Val, LKA.Lane);
    validL = validL && s.FrameValid; validR = validR && s.FrameValid;
    if validL, mem.offL = s.Left.LateralOffset; end
    if validR, mem.offR = s.Right.LateralOffset; end
    mem.validL = validL; mem.validR = validR;
    if widthValid
        mem.width = mem.width + dt / LKA.Lane.WidthTau * (widthMeas - mem.width);
    end
    [z, zValid, noiseScale] = lka_estimate_lane_center(s.Left, s.Right, validL, validR, mem.width);
    [kf.x, kf.P, kf.st, quality] = lka_lane_tracker_step(kf.x, kf.P, kf.st, z, zValid, noiseScale, v, yawRate, LKA.Ts, LKA.KF);
    est = struct('CenterOffset', kf.x(1), 'HeadingAngle', kf.x(2), 'Curvature', kf.x(3), ...
        'CurvatureDerivative', kf.x(4), 'LaneWidth', mem.width, 'Quality', quality);
    LdBase = interp1(LKA.PP.LdSpeed, LKA.PP.LdBase, min(max(v, LKA.PP.LdSpeed(1)), LKA.PP.LdSpeed(end)));
    kappaReq = lka_pure_pursuit(est, LdBase, LKA.PP);
    [kMax, kRate] = lka_curvature_limits(v, LKA.Veh, LKA.Lim);
    kappa = min(max(kappaReq, -kMax), kMax);
    kappa = mem.kappa + min(max(kappa - mem.kappa, -kRate * dt), kRate * dt);
    mem.kappa = kappa;
    steer = lka_curvature_to_steer(kappa, v, LKA.Veh);
end

function R = radiusOf(kappa)
    R = 1 / kappa;
    if ~isfinite(R) || abs(R) > 1000, R = 1000; end
end
