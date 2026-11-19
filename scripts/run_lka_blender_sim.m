function out = run_lka_blender_sim(options)
% RUN_LKA_BLENDER_SIM Closed-loop LKA on the lighting track with Blender as the camera.
%
% Syntax:
%   out = run_lka_blender_sim()                          % 0 -> 8 km/h at 0.5 m/s^2, whole track
%   out = run_lka_blender_sim(Speed=10, Accel=0.4, Controller=3)
%   out = run_lka_blender_sim(StopTime=20)               % quick check
%
% Closed loop, nothing scripted: the Baja plant moves the car, Blender renders what the
% camera sees at that pose (tools/blender/render_server.py), the Mobilenet detector finds
% the lines, the unchanged LKA_Controller steers, the plant integrates the steering. Lane
% failures therefore come from the detector AND the controller. Live view on the dashboard
% (start_lka_dashboard: camera, steering, lane, latency, and the path traced over the
% track). Results in OutDir: annotated video, overview / perception figures, path figure,
% KPIs (lka_compute_kpis: max / RMS lateral offset, wheel margin, in-lane ratio, rulebook
% score) and blender_sim.mat.
%
% Name-value options:
%   Speed       target speed [km/h] (default 8)          Accel  [m/s^2] from standstill (0.5)
%   Controller  1 pure pursuit | 2 Stanley 2025 | 3 MPC (LKA.ControllerSelect, default 1)
%   StopTime    [s] (default: until 10 m before the end of the track)
%   Width/Height  render size (default 640 x 360; camera intrinsics are scaled to it)
%   Samples     Cycles samples per frame (default 12)
%   TrackProfile 'tight' (default, MinRadius 12 m: the bends are R 15-25 m) | 'official'
%   Port        render server port (default 5560); OutDir (default results/blender/<time>)
%   Video, Monitor  annotated video / dashboard telemetry (default true)

    arguments
        options.Speed (1,1) double {mustBePositive} = 8
        options.Accel (1,1) double {mustBePositive} = 0.5
        options.Controller (1,1) double {mustBeMember(options.Controller, [1 2 3])} = 1
        options.StopTime (1,1) double = NaN
        options.Width (1,1) double = 640
        options.Height (1,1) double = 360
        options.Samples (1,1) double = 12
        options.TrackProfile char {mustBeMember(options.TrackProfile, {'tight', 'official'})} = 'tight'
        options.Port (1,1) double = 5560
        options.OutDir char = ''
        options.Video (1,1) logical = true
        options.Monitor (1,1) logical = true
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    mdl = 'LKA_Blender_TestBench';
    if ~isfile(fullfile(root, 'models', [mdl '.slx']))
        build_lka_blender_bench();       % before LKA is set below: the build reloads the base parameters
    end
    evalin('base', sprintf('run(''%s'');', fullfile(root, 'config', 'startup.m')));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params; create_lka_buses;');
    vehicle = evalin('base', 'vehicle');
    LKA0 = evalin('base', 'LKA');
    if isempty(options.OutDir)
        options.OutDir = fullfile(root, 'results', 'blender', char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
    end
    if ~isfolder(options.OutDir), mkdir(options.OutDir); end

    % ---------------- scenario: lighting track, gentle constant acceleration to Speed
    sc = generate_lka_scenarios(vehicle);
    sc = sc(strcmp({sc.Name}, 'S6_LightingTrack'));
    v = options.Speed / 3.6;
    sc.P.V0 = 0; sc.P.VSet = v; sc.P.Accel = options.Accel;
    if isnan(options.StopTime)
        d = sc.Road.length - 3 - 10;                       % start at s = 3 m, stop 10 m early
        options.StopTime = floor(v / options.Accel + (d - v^2 / (2 * options.Accel)) / v);
    end
    sc.P.StopTime = options.StopTime;
    r = sc.Road;
    assignin('base', 'SCN', sc.P);
    assignin('base', 'ROAD', struct('s', r.s, 'x', r.x, 'y', r.y, 'psi', r.psi, 'kappa', r.kappa, ...
        'dkappa', r.dkappa, 'width', r.width, 'lineWidth', r.lineWidth));

    % ---------------- LKA: tight-track tuning, 3.0 m lane, camera at the render size
    LKA = lka_set_track_profile(LKA0, options.TrackProfile, MinRadius=12, LaneWidth=3.0);
    LKA.ControllerSelect = options.Controller;
    k = options.Width / LKA.Vision.Camera.ImageSize(2);
    LKA.Vision.Camera.ImageSize = [options.Height options.Width];
    LKA.Vision.Camera.FocalLength = LKA.Vision.Camera.FocalLength * k;
    LKA.Vision.Camera.PrincipalPoint = [options.Width options.Height] / 2;
    LKA.Vision.Detector = 'mobilenet';
    LKA.Vision.UseGpu = false;
    LKA.Vision.Monitor = options.Monitor;
    LKA.Vision.Bench.VideoFile = '';
    if options.Video
        LKA.Vision.Bench.VideoFile = fullfile(options.OutDir, 'blender_sim_annotated.mp4');
    end
    LKA.Vision.Bench.RawVideoFile = '';
    LKA.Vision.Bench.DatasetDir = '';
    assignin('base', 'LKA', LKA);
    restore = onCleanup(@() assignin('base', 'LKA', LKA0));

    % ---------------- Blender render server
    server = startServer(root, options);
    stopServer = onCleanup(@() quitServer(options.Port, server));

    load_system(mdl);
    set_param(mdl, 'StopTime', num2str(options.StopTime));
    if options.Monitor
        lka_monitor_publish('start', struct('Name', sprintf('lighting track, %g km/h', options.Speed), ...
            'Kind', 'sim', 'Detector', 'mobilenet (Blender camera)', 'Gpu', false, 'OutDir', options.OutDir, ...
            'Frames', round(options.StopTime / LKA.Ts), 'Duration', options.StopTime, ...
            'Phase', 'closed loop: Blender camera -> Mobilenet -> LKA_Controller -> Baja plant'));
        lka_monitor_publish('road', struct('xL', r.xL, 'yL', r.yL, 'xR', r.xR, 'yR', r.yR));
    end
    fprintf('[LKA blender] %s, %.0f s simulated, %g km/h at %.1f m/s^2, controller %d\n', ...
        sc.Name, options.StopTime, options.Speed, options.Accel, options.Controller);
    t0 = tic;
    simOut = sim(mdl);
    wall = toc(t0);

    res = lka_extract_results(simOut, sc, LKA, vehicle);
    kpi = lka_compute_kpis(res);
    vi = squeeze(simOut.logsout.getElement('vision_info').Values.Data);
    if size(vi, 1) == 7, vi = vi'; end
    kpi.Detector = "mobilenet";
    kpi.VisionMsMean = mean(vi(2:end, 7));
    kpi.BothLinesRatio = mean(vi(:, 3) & vi(:, 4));
    kpi.WallClock_s = wall;
    plot_lka_results(res, kpi, options.OutDir);
    plotPath(res, r, kpi, fullfile(options.OutDir, 'blender_sim_path.png'));
    close all
    save(fullfile(options.OutDir, 'blender_sim.mat'), 'res', 'kpi', 'options');
    writetable(struct2table(kpi, 'AsArray', true), fullfile(options.OutDir, 'blender_sim_kpis.csv'));
    if options.Monitor
        lka_monitor_publish('done', struct('OutDir', options.OutDir, 'MaxLatOffset_m', kpi.MaxLatOffset_m, ...
            'InLaneRatio', kpi.InLaneRatio, 'MeanMs', kpi.VisionMsMean));
    end
    fprintf(['[LKA blender] %.0f s wall | max |lat| %.2f m, RMS %.2f m | in lane %.1f %% | ' ...
        'both lines %.1f %% | detector %.0f ms/frame -> %s\n'], wall, kpi.MaxLatOffset_m, ...
        kpi.RmsLatOffset_m, 100 * kpi.InLaneRatio, 100 * kpi.BothLinesRatio, kpi.VisionMsMean, options.OutDir);
    out = struct('res', res, 'kpi', kpi, 'OutDir', options.OutDir);
end

function pid = startServer(root, o)
    if canConnect(o.Port)
        fprintf('[LKA blender] render server already running on port %d\n', o.Port);
        pid = '';
        return
    end
    blender = fullfile(getenv('HOME'), '.local', 'bin', 'blender');
    if ~isfile(blender), blender = 'blender'; end
    log = fullfile(tempdir, 'lka_render_server.log');
    cmd = sprintf(['"%s" -b --factory-startup --python "%s" -- "%s" --port %d --width %d --height %d ' ...
        '--samples %d > "%s" 2>&1 & echo $!'], blender, fullfile(root, 'tools', 'blender', 'render_server.py'), ...
        fullfile(root, 'data', 'scenes', 'LightingTrack', 'track.json'), o.Port, o.Width, o.Height, o.Samples, log);
    [~, pid] = system(cmd);
    pid = strtrim(pid);
    fprintf('[LKA blender] starting Blender render server (pid %s, log %s) ...\n', pid, log);
    for k = 1:120
        if canConnect(o.Port), return, end
        pause(0.5);
    end
    error('lka:blender:noServer', 'Blender render server did not start, see %s', log);
end

function ok = canConnect(port)
    try
        c = tcpclient('127.0.0.1', port, 'ConnectTimeout', 1); %#ok<NASGU>
        ok = true;
    catch
        ok = false;
    end
end

function quitServer(port, pid)
    if isempty(pid), return, end                      % not ours
    try
        c = tcpclient('127.0.0.1', port, 'ConnectTimeout', 2, 'Timeout', 5);
        writeline(c, '{"cmd":"quit"}');
        readline(c);
    catch
        system(sprintf('kill %s 2>/dev/null', pid));
    end
end

function plotPath(res, road, kpi, file)
    f = figure('Color', 'w', 'Position', [60 60 1400 700], 'Visible', 'off');
    try, theme(f, 'light'); catch, end
    ax = axes(f); hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
    plot(ax, road.xL, road.yL, 'Color', [0.4 0.4 0.4]); plot(ax, road.xR, road.yR, 'Color', [0.4 0.4 0.4]);
    plot(ax, road.x, road.y, ':', 'Color', [0.6 0.6 0.6]);
    p = scatter(ax, res.gt.x, res.gt.y, 6, abs(res.gt.latOffset), 'filled');   % CG path, 10 ms
    colormap(ax, turbo); cb = colorbar(ax); cb.Label.String = '|lateral offset from the lane centre| [m]';
    clim(ax, [0 max(0.5, max(abs(res.gt.latOffset)))]);
    xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
    title(ax, sprintf('Path driven by the LKA (closed loop, Blender camera): max |offset| %.2f m, in lane %.1f %%', ...
        kpi.MaxLatOffset_m, 100 * kpi.InLaneRatio));
    legend(ax, {'lane lines', '', 'lane centre', 'vehicle (CG)'}, 'Location', 'best');
    p.DisplayName = 'vehicle';
    exportgraphics(f, file);
    close(f);
end
