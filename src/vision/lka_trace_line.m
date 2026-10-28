function [xs, ys] = lka_trace_line(bevProb, cam, y0, band, window, threshold, maxGap)
% LKA_TRACE_LINE Follows one painted line through the bird's-eye view without a curve model.
%
% Syntax:
%   [xs, ys] = lka_trace_line(bevProb, cam, y0)
%   [xs, ys] = lka_trace_line(bevProb, cam, y0, band, window, threshold, maxGap)
%
% An independent check of the polynomial fit (lka_fit_boundary): a fit that cannot bend
% enough (the straight-lane problem on the college track) shows up as a distance between
% the fitted line and this trace. From cam.XNear, band by band (band [m], default 0.25),
% the run of probability >= threshold (default 0.5) nearest to the predicted position is
% taken, within +-window [m] (default 0.35). The prediction is the last point plus the
% smoothed slope of the trace, so the trace can turn as sharply as the paint does. It
% stops after maxGap [m] without paint (default 1.5).
%
% Inputs:
%   bevProb - bird's-eye probability image (rows = cam.RowX, columns = cam.ColY)
%   cam     - lka_camera_model (RowX, ColY, XNear, XFar)
%   y0      - lateral position of the line at the first rows [m] (+ left)
% Outputs:
%   xs, ys  - points of the traced line [m] (column vectors, x increasing)

    if nargin < 4, band = 0.25; end
    if nargin < 5, window = 0.35; end
    if nargin < 6, threshold = 0.5; end
    if nargin < 7, maxGap = 1.5; end
    rowX = cam.RowX;
    colY = cam.ColY;
    edges = cam.XNear:band:cam.XFar;
    xs = zeros(0, 1);
    ys = zeros(0, 1);
    slope = 0;
    gap = 0;
    for k = 1:numel(edges) - 1
        rows = rowX >= edges(k) & rowX < edges(k + 1);
        if ~any(rows)
            continue
        end
        xm = 0.5 * (edges(k) + edges(k + 1));
        yPred = y0;
        if ~isempty(xs)
            yPred = ys(end) + slope * (xm - xs(end));
        end
        cIdx = find(abs(colY - yPred) <= window);
        block = bevProb(rows, cIdx);
        block(block < threshold) = 0;
        prof = sum(block, 1);
        runs = bwlabel(prof > 0);
        best = NaN;
        bestDist = inf;
        for j = 1:max(runs)
            m = runs == j;
            mass = sum(prof(m));
            if mass < 1
                continue
            end
            yRun = sum(prof(m) .* colY(cIdx(m))) / mass;
            if abs(yRun - yPred) < bestDist
                best = yRun;
                bestDist = abs(yRun - yPred);
            end
        end
        if isnan(best)
            gap = gap + band;
            if gap > maxGap
                break
            end
            continue
        end
        gap = 0;
        if ~isempty(xs)
            sNew = min(max((best - ys(end)) / (xm - xs(end)), -2), 2);
            if numel(xs) > 1
                slope = 0.5 * slope + 0.5 * sNew;
            else
                slope = sNew;
            end
        end
        xs(end + 1, 1) = xm; %#ok<AGROW>
        ys(end + 1, 1) = best; %#ok<AGROW>
    end
end
