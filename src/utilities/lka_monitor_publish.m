function lka_monitor_publish(action, data, frame)
% LKA_MONITOR_PUBLISH Live telemetry for the LKA dashboard (tools/lka_dashboard.py).
%
% Syntax:
%   lka_monitor_publish('start', info)          new run: info.Name, Kind ('video' | 'sim'),
%                                               Detector, Gpu, OutDir, Frames, Duration, Phase
%   lka_monitor_publish('ensure', info)         'start' only when no run is active (e.g. the
%                                               camera bench started from the Simulink toolbar)
%   lka_monitor_publish('phase', text)          e.g. 'writing annotated video' (clears the series)
%   lka_monitor_publish('sample', s)            one frame: struct of scalars (Time, Steer, ...)
%   lka_monitor_publish('sample', s, frame)     ... with the annotated camera frame (uint8, or a
%                                               function handle that makes it, only called when
%                                               the frame is actually written)
%   lka_monitor_publish('road', r)             track for the path view: r.xL, r.yL, r.xR, r.yR [m]
%   lka_monitor_publish('pose', [x y])          append the vehicle position to the path trace
%   lka_monitor_publish('extra', s)             scalars merged into the next 'sample' (another
%                                               block's values, e.g. the detector timing)
%   lka_monitor_publish('done', summary)
%
% Writes results/monitor/live.json (run, phase, latest sample, the last 1200 samples of
% every field) and results/monitor/live.jpg. Files are written to a temporary name and
% renamed, so the dashboard never reads half a file. JSON at most every 0.2 s, the frame
% at most every 0.4 s (start / phase / done are always written), so publishing costs the
% LKA loop well under 1 ms per frame on average. Errors never stop a run: the first one
% is reported as a warning (lka:monitor:failed), later ones are ignored.
    persistent run series latest lastJson lastFrame phase warned path road extra
    try
        dirOut = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results', 'monitor');
        if ~isfolder(dirOut)
            mkdir(dirOut);
        end
        now_ = posixtime(datetime('now', 'TimeZone', 'UTC'));   % epoch seconds, as Python time.time()
        force = false;
        if strcmpi(action, 'ensure')
            if ~isempty(run) && ~strcmp(phase, 'done')
                return
            end
            action = 'start';
        end
        switch lower(action)
            case 'start'
                run = data;
                run.Started = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
                series = struct();
                latest = struct();
                path = zeros(0, 2);
                road = [];
                extra = struct();
                phase = 'running';
                if isfield(data, 'Phase'), phase = data.Phase; end
                lastJson = 0; lastFrame = 0;
                force = true;
            case 'phase'
                phase = data;
                series = struct();
                force = true;
            case 'sample'
                if isempty(run)
                    return
                end
                if isstruct(extra)                          % values other blocks published
                    fx = fieldnames(extra);
                    for i = 1:numel(fx)
                        if ~isfield(data, fx{i}), data.(fx{i}) = extra.(fx{i}); end
                    end
                end
                latest = data;
                f = fieldnames(data);
                for i = 1:numel(f)
                    v = double(data.(f{i}));
                    if ~isscalar(v), continue; end
                    if ~isfield(series, f{i}), series.(f{i}) = zeros(1, 0); end
                    series.(f{i})(end + 1) = v;
                    if numel(series.(f{i})) > 1200
                        series.(f{i}) = series.(f{i})(end - 1199:end);
                    end
                end
                if nargin >= 3 && now_ - lastFrame >= 0.4
                    if isa(frame, 'function_handle')
                        frame = frame();
                    end
                    writeAtomic(fullfile(dirOut, 'live.jpg'), @(p) imwrite(frame, p, 'jpg', 'Quality', 80));
                    lastFrame = now_;
                end
            case 'road'
                k = 1:max(1, floor(numel(data.xL) / 600)):numel(data.xL);   % <= ~600 points a side
                road = struct('xL', data.xL(k), 'yL', data.yL(k), 'xR', data.xR(k), 'yR', data.yR(k));
                force = true;
            case 'extra'
                f = fieldnames(data);
                for i = 1:numel(f), extra.(f{i}) = data.(f{i}); end
                return
            case 'pose'
                if isempty(path) || hypot(data(1) - path(end, 1), data(2) - path(end, 2)) >= 0.1
                    path(end + 1, :) = double(data(1:2));
                    if size(path, 1) > 20000, path = path(2:2:end, :); end   % keep it bounded
                end
                return                                       % written with the next sample
            case 'done'
                phase = 'done';
                run.Summary = data;
                run.Finished = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
                force = true;
            otherwise
                error('lka:monitor:action', 'Unknown action "%s".', action);
        end
        if force || now_ - lastJson >= 0.2
            s = struct('Run', run, 'Phase', phase, 'Latest', latest, 'Series', series, 'Updated', now_);
            if ~isempty(road), s.Road = road; end
            if ~isempty(path), s.Path = struct('x', path(:, 1)', 'y', path(:, 2)'); end
            txt = jsonencode(s);
            writeAtomic(fullfile(dirOut, 'live.json'), @(p) writeText(p, txt));
            lastJson = now_;
        end
    catch err
        if isempty(warned)
            warned = true;
            warning('lka:monitor:failed', 'Live dashboard telemetry disabled: %s', err.message);
        end
    end
end

function writeAtomic(file, writer)
% The dashboard may hold the target open for a moment (Windows refuses the rename then):
% retry a few times before giving up on this update.
    [d, n, e] = fileparts(file);
    tmp = fullfile(d, [n '.tmp' e]);
    writer(tmp);
    for attempt = 1:5
        [ok, msg] = movefile(tmp, file, 'f');
        if ok
            return
        end
        pause(0.01 * attempt);
    end
    error('lka:monitor:rename', '%s', msg);
end

function writeText(file, txt)
    fid = fopen(file, 'w', 'n', 'UTF-8');
    fwrite(fid, txt, 'char');
    fclose(fid);
end
