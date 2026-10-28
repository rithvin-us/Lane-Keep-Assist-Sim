function [found, info] = lka_horizontal_detect(bevProb, cam, laneWidth, D)
% LKA_HORIZONTAL_DETECT Stop line / horizontal marking ahead, in metres in the BEV.
%
% Syntax:
%   [found, info] = lka_horizontal_detect(bevProb, cam, laneWidth, D)
%                                                            D = LKA.Vision.Marking
%
% A stop line, a pedestrian crossing bar or an intersection edge shows up in the metric
% bird's-eye view as a band of rows painted across most of the lane, unlike a lane
% boundary, which covers one column. Per BEV row the share of columns above
% D.ProbThreshold within +-laneWidth/2 of the vehicle axis is the coverage; a run of rows
% with coverage >= D.HorCoverage whose length lies in [D.HorMin, D.HorMax] is reported,
% with its distance from the rear axle.
%
% tsiggi/LaneAssist do this in the image plane with a slope-limited point search
% (horizontal_detection, max_allowed_slope 0.25), and never call it. In the BEV a
% horizontal marking is simply a wide row, so no slope test is needed, and the distance
% comes out in metres (finding L8 of docs/laneassist_comparison.md). Nothing in the
% controller consumes this yet: it is reported, not acted on.
%
% Outputs:
%   found - a horizontal marking is in view
%   info  - struct Distance [m from the rear axle, NaN when none], Length [m],
%           Coverage (of the reported band), Profile (coverage per BEV row)

    info = struct('Distance', NaN, 'Length', 0, 'Coverage', 0, 'Profile', []);
    found = false;

    rowX = cam.RowX;
    cols = abs(cam.ColY) <= laneWidth / 2;
    if ~any(cols)
        return
    end
    profile = mean(bevProb(:, cols) >= D.ProbThreshold, 2);
    info.Profile = profile;

    wide = profile >= D.HorCoverage;
    if ~any(wide)
        return
    end
    ds = abs(median(diff(rowX)));
    [vals, lens, first] = runs(wide);
    for k = find(vals(:)')
        len = lens(k) * ds;
        if len < D.HorMin || len > D.HorMax
            continue
        end
        band = first(k):first(k) + lens(k) - 1;
        x = mean(rowX(band));
        if ~found || x < info.Distance            % the nearest band wins
            found = true;
            info.Distance = x;
            info.Length = len;
            info.Coverage = mean(profile(band));
        end
    end
end

function [vals, lens, first] = runs(x)
% Run-length encoding of a logical column, without bwlabel (no toolbox needed).
    x = logical(x(:));
    edges = [true; diff(x) ~= 0];
    first = find(edges);
    vals = x(first);
    lens = diff([first; numel(x) + 1]);
end
