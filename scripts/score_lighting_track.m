function S = score_lighting_track(resultDir, gtCsv, trackJson)
% SCORE_LIGHTING_TRACK Lane detection on the Blender lighting track against its ground truth.
%
% Syntax:
%   S = score_lighting_track('test_res/mobilenet/lighting_track', ...
%       'test_res/mobilenet/lighting_track/ground_truth.csv', 'data/scenes/LightingTrack/track.json')
%
% resultDir holds lka_video_results.csv from run_lka_on_video on the rendered drive
% (tools/blender/render_drive.py, which also writes ground_truth.csv). Every frame is
% labelled by what the camera sees 4-12 m ahead: the lighting patch there (normal /
% bright / glare / dark) and the track section (straight, 30 deg, 60 deg, S-bend).
% Reported per label: frames, both-lines %, and the mean |error| of each reported line
%   at the rear axle (Axle*: what the controller uses; on a bend this includes the
%   extrapolation from the first visible row ~5.5 m ahead back to the axle) and
%   at 8 m ahead (Ahead*: where the camera sees the line; the perception error),
% plus frames with a reported line more than 0.5 m off at 8 m. Writes
% lighting_track_score.csv.

    R = readtable(fullfile(resultDir, 'lka_video_results.csv'));
    G = readtable(gtCsv);
    J = jsondecode(fileread(trackJson));
    gi = interp1(G.t_s, (1:height(G))', R.Time_s, 'nearest', 'extrap');
    G = G(gi, :);
    camX = 1.6;                                    % camera ahead of the rear axle [m]
    look = G.s_m + camX + [4 12];                  % road range the detector looks at

    cond = repmat("normal", height(R), 1);
    for p = J.patches'
        hit = look(:, 1) <= p.S(2) & look(:, 2) >= p.S(1);
        cond(hit & cond == "normal") = string(p.Type);
    end
    sect = repmat("straight", height(R), 1);
    mid = mean(look, 2);
    for c = J.sections'
        if c.deg > 0 && any(mid >= c.s0 & mid <= c.s1)
            in = mid >= c.s0 & mid <= c.s1;
            sect(in) = sprintf('%d deg R%d', c.deg, abs(c.R));
        end
    end

    okL = R.LeftStrength > 0; okR = R.RightStrength > 0;
    eL = abs(R.LeftOffset_m - G.left_line_m); eR = abs(R.RightOffset_m - G.right_line_m);
    % true line offsets 8 m ahead, in the vehicle frame (pose rebuilt from s, offset, heading)
    road = lka_lighting_track();
    xa = 8;
    trueL = nan(height(R), 1); trueR = trueL;
    for k = 1:height(R)
        psiR = interp1(road.s, road.psi, G.s_m(k));
        xr = interp1(road.s, road.x, G.s_m(k)) - G.vehicle_offset_m(k) * sin(psiR);
        yr = interp1(road.s, road.y, G.s_m(k)) + G.vehicle_offset_m(k) * cos(psiR);
        h = psiR + deg2rad(G.heading_error_deg(k));
        toVeh = @(x, y) deal(cos(h) * (x - xr) + sin(h) * (y - yr), -sin(h) * (x - xr) + cos(h) * (y - yr));
        [vx, vy] = toVeh(road.xL, road.yL); trueL(k) = lineAt(vx, vy, xa);
        [vx, vy] = toVeh(road.xR, road.yR); trueR(k) = lineAt(vx, vy, xa);
    end
    at = @(c0, hdeg, kap) c0 + tand(hdeg) * xa + kap / 2 * xa^2;
    aL = abs(at(R.LeftOffset_m, R.LeftHeading_deg, R.LeftCurvature_1pm) - trueL);
    aR = abs(at(R.RightOffset_m, R.RightHeading_deg, R.RightCurvature_1pm) - trueR);
    groups = [{"all"}, cellstr(unique(cond))', cellstr(unique(sect))'];
    rows = {};
    for k = 1:numel(groups)
        g = string(groups{k});
        if g == "all", m = true(height(R), 1);
        elseif any(cond == g), m = cond == g;
        else, m = sect == g;
        end
        rows(end + 1, :) = {g, nnz(m), 100 * mean(okL(m) & okR(m)), 100 * mean(okL(m) | okR(m)), ...
            mean(aL(m & okL), 'omitnan'), mean(aR(m & okR), 'omitnan'), mean(eL(m & okL)), mean(eR(m & okR)), ...
            nnz(m & ((okL & aL > 0.5) | (okR & aR > 0.5))), 100 * mean(R.LkaState(m) >= 2)}; %#ok<AGROW>
    end
    S = cell2table(rows, 'VariableNames', {'Label', 'Frames', 'BothLines_pct', 'AnyLine_pct', ...
        'AheadErrL_m', 'AheadErrR_m', 'AxleErrL_m', 'AxleErrR_m', 'Off05m_frames', 'LkaActive_pct'});
    writetable(S, fullfile(resultDir, 'lighting_track_score.csv'));
    disp(S)
end

function y = lineAt(vx, vy, xa)
% Lateral offset of a boundary polyline (vehicle frame) at x = xa, nearest crossing ahead.
    y = NaN;
    c = find(sign(vx(1:end - 1) - xa) ~= sign(vx(2:end) - xa) & vx(1:end - 1) > 0 & abs(vy(1:end - 1)) < 8, 1);
    if ~isempty(c)
        a = (xa - vx(c)) / (vx(c + 1) - vx(c));
        y = vy(c) + a * (vy(c + 1) - vy(c));
    end
end
