function [dashed, info] = lka_dashed_detect(bevProb, cam, b, D)
% LKA_DASHED_DETECT Is this lane boundary dashed? Measured in metres in the BEV.
%
% Syntax:
%   [dashed, info] = lka_dashed_detect(bevProb, cam, b, D)      D = LKA.Vision.Marking
%
% The fitted boundary y(x) is followed through the metric bird's-eye view and a corridor
% of +-D.HalfWidth around it is reduced to one occupancy flag per BEV row. The flags are
% run-length encoded, so a dash and a gap are lengths in metres, and the boundary counts
% as dashed when at least D.MinCycles paint runs lie inside [D.DashMin, D.DashMax] and the
% gaps between them inside [D.GapMin, D.GapMax].
%
% tsiggi/LaneAssist classify a dash by counting points and gaps in image rows, which
% makes the thresholds depend on the mount and the range; in the BEV the same test is in
% metres and transfers between cameras. Their classifier is written but never called;
% we had none at all (finding L8 of docs/laneassist_comparison.md). Nothing in the
% controller consumes this yet: it is reported, not acted on.
%
% Inputs:
%   bevProb - lka_bev_warp output, H x W (one page)
%   cam     - lka_camera_model
%   b       - one LaneBoundaryBus boundary (LateralOffset, HeadingAngle, Curvature,
%             CurvatureDerivative), Strength > 0
% Outputs:
%   dashed  - true when the paint along this boundary alternates like a dashed line
%   info    - struct DashLengths, GapLengths [m], Cycles, Coverage (share of rows painted)

    info = struct('DashLengths', [], 'GapLengths', [], 'Cycles', 0, 'Coverage', 0);
    dashed = false;
    if b.Strength <= 0
        return
    end

    rowX = cam.RowX;
    colY = cam.ColY;
    y = b.LateralOffset + tan(b.HeadingAngle) * rowX + b.Curvature / 2 * rowX .^ 2 ...
        + b.CurvatureDerivative / 6 * rowX .^ 3;
    inRange = rowX >= cam.XNear & rowX <= min(cam.XFar, b.ViewRange);
    rows = find(inRange);
    occupied = false(numel(rows), 1);
    for k = 1:numel(rows)
        r = rows(k);
        cols = abs(colY - y(r)) <= D.HalfWidth;
        occupied(k) = sum(bevProb(r, cols) >= D.ProbThreshold) >= D.MinRowMass;
    end
    if isempty(occupied)
        return
    end
    info.Coverage = mean(occupied);

    % row spacing is constant in the BEV, so a run length is metres
    ds = abs(median(diff(rowX)));
    [vals, lens] = runs(occupied);
    info.DashLengths = lens(vals) * ds;
    info.GapLengths = lens(~vals) * ds;

    dash = info.DashLengths >= D.DashMin & info.DashLengths <= D.DashMax;
    gap = info.GapLengths >= D.GapMin & info.GapLengths <= D.GapMax;
    % a gap at either end of the view is not a measured gap
    if ~isempty(gap) && ~vals(1)
        gap(1) = false;
    end
    if ~isempty(gap) && ~vals(end)
        gap(end) = false;
    end
    info.Cycles = min(sum(dash), sum(gap));
    dashed = info.Cycles >= D.MinCycles;
end

function [vals, lens] = runs(x)
% Run-length encoding of a logical column, without bwlabel (no toolbox needed).
    x = logical(x(:));
    edges = [true; diff(x) ~= 0];
    idx = find(edges);
    vals = x(idx);
    lens = diff([idx; numel(x) + 1]);
end
