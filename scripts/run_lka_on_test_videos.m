function S = run_lka_on_test_videos(options)
% RUN_LKA_ON_TEST_VIDEOS Lane detection + LKA replay on every recorded track video.
%
% Syntax:
%   S = run_lka_on_test_videos()
%   S = run_lka_on_test_videos(Folder='vids', OutRoot='test_res', Detector='cnn')
%   S = run_lka_on_test_videos(Files={'clip.mp4'})        % only these files of Folder
%
% Runs run_lka_on_video on each *.mp4 / *.mov in Folder (unknown phone mount: camera
% pitch auto-calibrated, lens height CameraHeight, horizontal field of view HFovDeg)
% and writes, per video, OutRoot/<video name>/:
%   lka_video_annotated.mp4   camera view with the detected lines and the LKA commands
%   lka_video_overview.png    lane position, heading, radius, steering, speed, yaw
%   lka_video_path.png        path from visual odometry with the detected lines
%   lka_video_results.csv     one row per processed frame (20 Hz)
% plus OutRoot/summary.csv: one row per video (detection rates, speed, yaw, radius, and
% the straight-lane alarm: ShapeErrorP90_m = distance between the fitted and the painted
% lines that 90 % of the lines stay within; StraightAlarm = true above 0.25 m).
% TrackProfile / MinRadius go to run_lka_on_video ('tight', 3 m: the college snake track;
% use 'official' for a video of the aBaja LKA track).
% Summary only (no processing): S = run_lka_on_test_videos(Files={}).

    arguments
        options.Folder char = ''
        options.OutRoot char = ''
        options.Files = []
        options.Detector char = 'cnn'
        options.HFovDeg (1,1) double = 70
        options.CameraHeight (1,1) double = 1.2
        options.LaneWidth (1,1) double = NaN
        options.Video (1,1) logical = true
        options.TrackProfile char {mustBeMember(options.TrackProfile, {'tight', 'official'})} = 'tight'
        options.MinRadius (1,1) double {mustBePositive} = 3
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    if isempty(options.Folder), options.Folder = fullfile(root, 'vids'); end
    if isempty(options.OutRoot), options.OutRoot = fullfile(root, 'test_res'); end
    if ~isfolder(options.OutRoot), mkdir(options.OutRoot); end

    if isempty(options.Files) && ~iscell(options.Files)
        d = [dir(fullfile(options.Folder, '*.mp4')); dir(fullfile(options.Folder, '*.mov'))];
        files = {d.name};
    else
        files = cellstr(options.Files);
    end
    for k = 1:numel(files)
        [~, name] = fileparts(files{k});
        run_lka_on_video(fullfile(options.Folder, files{k}), Detector=options.Detector, Camera='auto', ...
            HFovDeg=options.HFovDeg, CameraHeight=options.CameraHeight, LaneWidth=options.LaneWidth, ...
            OutDir=fullfile(options.OutRoot, name), Video=options.Video, ...
            TrackProfile=options.TrackProfile, MinRadius=options.MinRadius);
    end

    % ---------------- summary of every result folder in OutRoot
    d = dir(fullfile(options.OutRoot, '*', 'lka_video_results.mat'));
    rows = cell(numel(d), 1);
    for k = 1:numel(d)
        r = load(fullfile(d(k).folder, d(k).name), 'out');
        o = r.out; T = o.Table;
        [~, name, ext] = fileparts(d(k).folder);     % video names contain dots
        name = [name ext];
        both = T.LeftStrength > 0 & T.RightStrength > 0;
        active = T.LkaState == 2;
        steer = abs(T.SteerCmd_deg(active));
        rCmd = abs(T.TurnRadius_m(active));
        shape = NaN; viewRange = NaN;             % columns added 2026-10-16 (older results: NaN)
        if ismember('LeftShapeError_m', T.Properties.VariableNames)
            e = [T.LeftShapeError_m; T.RightShapeError_m];
            e = sort(e(isfinite(e)));
            if ~isempty(e), shape = e(max(1, ceil(0.9 * numel(e)))); end
            v = [T.LeftViewRange_m(T.LeftStrength > 0); T.RightViewRange_m(T.RightStrength > 0)];
            if ~isempty(v), viewRange = median(v); end
        end
        rows{k} = table(string(name), string(o.Detector), height(T), T.Time_s(end) - T.Time_s(1), ...
            o.Camera.PitchDeg, o.Camera.Height, getfieldOr(o, "LaneWidth", NaN), ...
            100 * mean(both), 100 * mean(T.LeftStrength > 0 | T.RightStrength > 0), 100 * mean(active), ...
            trapz(T.Time_s, T.Speed_mps), mean(T.Speed_kmh), max(T.Speed_kmh), ...
            max(abs(T.YawRate_degps)), max(T.YawAngle_deg) - min(T.YawAngle_deg), ...
            maxOrNaN(steer), minOrNaN(rCmd), minOrNaN(abs(T.RoadRadius_m(both))), ...
            string(getfieldOr(o, "TrackProfile", "official")), viewRange, shape, shape > 0.25, ...
            'VariableNames', {'Video', 'Detector', 'Frames', 'Duration_s', 'CameraPitch_deg', ...
            'CameraHeight_m', 'LaneWidth_m', 'BothLines_pct', 'AnyLine_pct', 'LkaActive_pct', ...
            'Distance_m', 'MeanSpeed_kmh', 'MaxSpeed_kmh', 'MaxYawRate_degps', 'YawRange_deg', ...
            'MaxSteerCmd_deg', 'MinTurnRadiusCmd_m', 'MinRoadRadius_m', ...
            'TrackProfile', 'MedianViewRange_m', 'ShapeErrorP90_m', 'StraightAlarm'});
    end
    S = vertcat(rows{:});
    if ~isempty(S)
        writetable(S, fullfile(options.OutRoot, 'summary.csv'));
        disp(S);
    end
end

function v = maxOrNaN(x)
    v = NaN;
    if ~isempty(x), v = max(x); end
end

function v = minOrNaN(x)
    v = NaN;
    if ~isempty(x), v = min(x); end
end

function v = getfieldOr(s, name, default)
    v = default;
    if isfield(s, name), v = s.(name); end
end
