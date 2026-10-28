function t = watch_lka_videos(options)
% WATCH_LKA_VIDEOS Runs the LKA on every new video copied into a folder, automatically.
%
% Syntax:
%   t = watch_lka_videos()                         % watch vids/ every 30 s (MATLAB stays usable)
%   t = watch_lka_videos(Folder='D:\clips', Period=60, TrackProfile='official')
%   watch_lka_videos(Once=true)                    % one pass and return (matlab -batch, Task Scheduler)
%   watch_lka_videos(Stop=true)                    % stop watching
%
% Copy a phone video (*.mp4, *.mov) into Folder. At the next check it goes through
% run_lka_on_test_videos (lane detection, LKA replay, annotated video, CSV and figures in
% OutRoot/<video name>/, and OutRoot/summary.csv with the straight-lane alarm). A video
% counts as new while OutRoot/<name>/lka_video_results.csv is missing or older than it.
% A file whose size still changes (being copied) waits for the next check. A video that
% fails gets OutRoot/<name>/FAILED.txt and is retried only when the file changes. Every
% action is logged in OutRoot/watch_log.txt.
%
% Name-value options:
%   Folder        videos to watch (default vids/)
%   OutRoot       results (default test_res/)
%   Period        seconds between checks (default 30)
%   Detector      '' (default: team CNN when available, else classic) | 'cnn' | 'classic' | 'yolop'
%   TrackProfile  'tight' (default, college track) | 'official' (lka_set_track_profile)
%   MinRadius     tightest bend of the track [m] (default 3)
%   Video         write the annotated video (default true)
%   Once          one pass instead of a background timer (default false)
%   Stop          stop a running watcher (default false)
% Output: the MATLAB timer (empty with Once or Stop).

    arguments
        options.Folder char = ''
        options.OutRoot char = ''
        options.Period (1,1) double {mustBePositive} = 30
        options.Detector char = ''
        options.TrackProfile char {mustBeMember(options.TrackProfile, {'tight', 'official'})} = 'tight'
        options.MinRadius (1,1) double {mustBePositive} = 3
        options.Video (1,1) logical = true
        options.Once (1,1) logical = false
        options.Stop (1,1) logical = false
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    if isempty(options.Folder), options.Folder = fullfile(root, 'vids'); end
    if isempty(options.OutRoot), options.OutRoot = fullfile(root, 'test_res'); end
    t = [];

    old = timerfindall('Tag', 'lka_video_watch');
    if options.Stop || ~options.Once
        if ~isempty(old)
            stop(old);
            delete(old);
            fprintf('[LKA watch] stopped the previous watcher\n');
        end
    end
    if options.Stop
        return
    end
    if ~isfolder(options.OutRoot), mkdir(options.OutRoot); end

    if options.Once
        sizes = fileSizes(options.Folder);
        pause(2);                                  % a file still being copied grows meanwhile
        processNew(options, sizes);
        return
    end
    lastSizes = fileSizes(options.Folder);
    t = timer('Tag', 'lka_video_watch', 'ExecutionMode', 'fixedSpacing', 'Period', options.Period, ...
        'BusyMode', 'drop', 'TimerFcn', @(~, ~) tick());
    start(t);
    logLine(options.OutRoot, sprintf('watching %s every %g s', options.Folder, options.Period));
    fprintf('[LKA watch] watching %s every %g s. Stop with watch_lka_videos(Stop=true)\n', ...
        options.Folder, options.Period);

    function tick()
        try
            processNew(options, lastSizes);
        catch ME
            logLine(options.OutRoot, ['watcher error: ' ME.message]);
        end
        lastSizes = fileSizes(options.Folder);
    end
end

function processNew(opt, lastSizes)
% Runs every new, completely copied video once.
    d = videos(opt.Folder);
    for k = 1:numel(d)
        [~, name] = fileparts(d(k).name);
        outDir = fullfile(opt.OutRoot, name);
        done = dir(fullfile(outDir, 'lka_video_results.csv'));
        if ~isempty(done) && done.datenum >= d(k).datenum
            continue                                    % already processed
        end
        failed = dir(fullfile(outDir, 'FAILED.txt'));
        if ~isempty(failed) && failed.datenum >= d(k).datenum
            continue                                    % failed before, file unchanged
        end
        if ~isKey(lastSizes, d(k).name) || lastSizes(d(k).name) ~= d(k).bytes
            continue                                    % new or still being copied: next check
        end
        logLine(opt.OutRoot, ['processing ' d(k).name]);
        try
            run_lka_on_test_videos(Folder=opt.Folder, OutRoot=opt.OutRoot, Files={d(k).name}, ...
                Detector=opt.Detector, Video=opt.Video, TrackProfile=opt.TrackProfile, ...
                MinRadius=opt.MinRadius);
            if isfile(fullfile(outDir, 'FAILED.txt'))
                delete(fullfile(outDir, 'FAILED.txt'));
            end
            logLine(opt.OutRoot, ['done ' d(k).name ' -> ' outDir]);
        catch ME
            if ~isfolder(outDir), mkdir(outDir); end
            fid = fopen(fullfile(outDir, 'FAILED.txt'), 'w');
            fprintf(fid, '%s\n%s\n', ME.identifier, ME.message);
            fclose(fid);
            logLine(opt.OutRoot, ['FAILED ' d(k).name ': ' ME.message]);
        end
    end
end

function d = videos(folder)
    d = [dir(fullfile(folder, '*.mp4')); dir(fullfile(folder, '*.mov')); ...
         dir(fullfile(folder, '*.MP4')); dir(fullfile(folder, '*.MOV'))];
    [~, keep] = unique({d.name});                   % case-insensitive file systems list twice
    d = d(keep);
end

function m = fileSizes(folder)
    m = containers.Map('KeyType', 'char', 'ValueType', 'double');
    d = videos(folder);
    for k = 1:numel(d)
        m(d(k).name) = d(k).bytes;
    end
end

function logLine(outRoot, msg)
    line = sprintf('%s  %s', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), msg);
    fprintf('[LKA watch] %s\n', msg);
    fid = fopen(fullfile(outRoot, 'watch_log.txt'), 'a');
    if fid > 0
        fprintf(fid, '%s\n', line);
        fclose(fid);
    end
end
