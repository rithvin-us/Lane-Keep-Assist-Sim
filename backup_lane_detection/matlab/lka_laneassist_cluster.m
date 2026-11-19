function [sel, lanes, out] = lka_laneassist_cluster(pts, slice, rows, imgSize, A, prev)
% LKA_LANEASSIST_CLUSTER LaneAssist point clustering, lane choice, fit and certainty.
%
% Syntax:
%   [sel, lanes, out] = lka_laneassist_cluster(pts, slice, rows, imgSize, A, prev)
%
% Port of peaks_clustering / find_best_qualified_points / verify_with_expected_value /
% choose_correct_lanes / fit_polyfit / lanes_post_processing of tsiggi/LaneAssist (MIT,
% Christos-Alexandros Tsingiropoulos, VROOM - BFMC 2023); see
% third_party/LaneAssist_LICENSE.txt.
%
% Slice by slice, bottom to top, every lane takes at most one of that slice's points and
% every point goes to at most one lane: a candidate must pass a slope gate
% (A.MaxAllowedWidthPerc of the image width) and an extrapolation gate, and conflicts are
% resolved by the cost
%   A.WidthWeight * slope distance + A.ExpectedWeight * extrapolation error.
% Unmatched points open new lanes, so an arbitrary number of lines is tracked at once
% (unlike our two-sided window search in lka_search_boundary). The left lane is the
% right-most long lane left of the image centre, the right lane the left-most long one
% right of it; each is fitted as x = a*y^2 + b*y + c in image pixels (degree 1 when the
% lane is shorter than A.OptimalPeakPerc of the slices) and scored against the previous
% frame's coefficients.
%
% Inputs:
%   pts, slice, rows - lka_laneassist_peaks output
%   imgSize          - [H W]
%   prev             - struct LeftCoef, RightCoef ([] when none), PrevTrustLk
% Outputs:
%   sel   - struct Left, Right: the chosen lanes' [col row] points
%   lanes - every clustered lane, cell of [col row]
%   out   - LeftCoef, RightCoef ([a b c] or []), LeftCert, RightCert [0-100],
%           TrustLeft, TrustRight, TrustLk, BottomWidth, TopWidth [px]

    w = imgSize(2);
    step = fix(-(imgSize(1) * A.BottomPerc / A.Slices));
    realSlices = max(1, numel(rows) - 1);
    maxDist = A.MaxAllowedWidthPerc * w;

    lanes = {};
    for k = 1:numel(rows)
        p = sort(pts(slice == k, 1))';
        if isempty(p)
            continue
        end
        lanes = clusterRow(lanes, p, rows(k), step, realSlices, maxDist, A);
    end

    [left, right] = chooseLanes(lanes, w, A);
    sel = struct('Left', left, 'Right', right);

    leftCoef = fitLane(left, A);
    rightCoef = fitLane(right, A);
    lCert = certainty(leftCoef, prev.LeftCoef, size(left, 1), realSlices, A);
    rCert = certainty(rightCoef, prev.RightCoef, size(right, 1), realSlices, A);

    trustL = true;
    trustR = true;
    if abs(lCert - rCert) > A.AllowedCertaintyDif
        trustL = lCert > rCert;
        trustR = ~trustL;
    end
    okLk = (lCert > A.MinSingleCertainty || rCert > A.MinSingleCertainty) ...
        || (lCert > A.MinDualCertainty && rCert > A.MinDualCertainty);

    out = struct('LeftCoef', leftCoef, 'RightCoef', rightCoef, ...
        'LeftCert', lCert, 'RightCert', rCert, ...
        'TrustLeft', trustL && ~isempty(leftCoef), 'TrustRight', trustR && ~isempty(rightCoef), ...
        'TrustLk', prev.PrevTrustLk && okLk, 'OkLk', okLk, ...
        'BottomWidth', NaN, 'TopWidth', NaN);

    % bottom_width / top_width re-estimated from the fitted coefficients (their
    % update_b_and_top_through_coefs); only with both lanes trusted
    if out.TrustLeft && out.TrustRight
        bot = rows(1);
        top = rows(end);
        lb = polyval(leftCoef, bot); lt = polyval(leftCoef, top);
        rb = polyval(rightCoef, bot); rt = polyval(rightCoef, top);
        out.BottomWidth = (rb - lb) / 2;
        out.TopWidth = (rt - lt) / 2;
    end
end

% ------------------------------------------------------------------------------------ %

function lanes = clusterRow(lanes, p, height, step, realSlices, maxDist, A)
    if isempty(lanes)
        for j = 1:numel(p)
            lanes{j} = [p(j), height];
        end
        return
    end
    nL = numel(lanes);
    nP = numel(p);
    lanePoint = -ones(1, nL);       % index of the lane's best point
    laneDist = -ones(1, nL);
    pointUsed = false(1, nP);
    pointLane = -ones(1, nP);

    [lanePoint, laneDist, pointUsed, pointLane, again] = ...
        match(lanes, p, height, step, realSlices, maxDist, A, lanePoint, laneDist, pointUsed, pointLane);
    if again
        [lanePoint, ~, pointUsed, ~] = ...
            match(lanes, p, height, step, realSlices, maxDist, A, lanePoint, laneDist, pointUsed, pointLane);
    end

    for j = 1:nL
        if lanePoint(j) > 0
            lanes{j} = [lanes{j}; p(lanePoint(j)), height];
        end
    end
    for j = find(~pointUsed)
        lanes = insertLane(lanes, p(j), height);
    end
end

function [lanePoint, laneDist, pointUsed, pointLane, again] = ...
        match(lanes, p, height, step, realSlices, maxDist, A, lanePoint, laneDist, pointUsed, pointLane)
% One pass of find_best_qualified_points: a better pairing steals the point or the lane
% from the previous one and asks for a second pass.
    again = false;
    for li = 1:numel(lanes)
        if lanePoint(li) > 0
            continue
        end
        atLeastOne = false;
        last = lanes{li}(end, :);
        for pi = 1:numel(p)
            dy = height - last(2);
            widthDist = abs((p(pi) - last(1)) * step / dy);
            if widthDist >= maxDist
                if atLeastOne
                    break
                end
                continue
            end
            [okPt, expErr] = expectedError(lanes{li}, height, p(pi), step, realSlices, maxDist);
            if ~okPt
                continue
            end
            atLeastOne = true;
            d = A.WidthWeight * widthDist + A.ExpectedWeight * expErr;

            if ~pointUsed(pi)
                if lanePoint(li) < 0
                    [lanePoint, laneDist, pointUsed, pointLane] = take(lanePoint, laneDist, pointUsed, pointLane, li, pi, d);
                elseif laneDist(li) > d                       % 2 points, 1 lane
                    again = true;
                    prevPt = lanePoint(li);
                    pointUsed(prevPt) = false;
                    pointLane(prevPt) = -1;
                    [lanePoint, laneDist, pointUsed, pointLane] = take(lanePoint, laneDist, pointUsed, pointLane, li, pi, d);
                end
            else
                prevLane = pointLane(pi);
                if lanePoint(li) < 0                          % 1 point, 2 lanes
                    if laneDist(prevLane) > d
                        again = true;
                        lanePoint(prevLane) = -1;
                        laneDist(prevLane) = -1;
                        [lanePoint, laneDist, pointUsed, pointLane] = take(lanePoint, laneDist, pointUsed, pointLane, li, pi, d);
                    end
                elseif ~(pointLane(pi) == li && lanePoint(li) == pi)   % 2 points, 2 lanes
                    prevPt = lanePoint(li);
                    if laneDist(li) > d && laneDist(prevLane) > d
                        again = true;
                        pointUsed(prevPt) = false;
                        pointLane(prevPt) = -1;
                        lanePoint(prevLane) = -1;
                        laneDist(prevLane) = -1;
                        [lanePoint, laneDist, pointUsed, pointLane] = take(lanePoint, laneDist, pointUsed, pointLane, li, pi, d);
                    end
                end
            end
        end
    end
    if again && (all(lanePoint > 0) || all(pointUsed))
        again = false;
    end
end

function [lanePoint, laneDist, pointUsed, pointLane] = take(lanePoint, laneDist, pointUsed, pointLane, li, pi, d)
    lanePoint(li) = pi;
    laneDist(li) = d;
    pointUsed(pi) = true;
    pointLane(pi) = li;
end

function [ok, dist] = expectedError(lane, height, x, step, realSlices, maxDist)
% Extrapolation gate: up to three straight-line estimates of where this lane should be
% in the current slice (first-to-last, last two, the two before them), punished for a
% single-point lane and for every slice skipped since the lane's last point.
    dx = lane(end, 1) - lane(1, 1);
    dy = lane(end, 2) - lane(1, 2);
    xDivY = 0;
    if dy ~= 0
        xDivY = dx / dy;
    end
    d = abs(x - ((height - lane(end, 2)) * xDivY + lane(end, 1)));

    punish = 0;
    if xDivY == 0
        punish = floor(maxDist / 2);
    end
    punish = punish + ((height - lane(end, 2)) / step - 1) / realSlices * maxDist;

    if size(lane, 1) > 3
        d2 = slopeEstimate(lane(end - 1, :), lane(end, :), height, x);
        d3 = slopeEstimate(lane(end - 2, :), lane(end - 1, :), lane(end - 1, 2), x);
        d = min([d, d2, d3]);
    end

    limit = maxDist;
    if xDivY ~= 0
        limit = floor(maxDist / 2);
    end
    ok = d < limit;
    dist = d + punish;
end

function d = slopeEstimate(pA, pB, height, x)
    dy = pB(2) - pA(2);
    xDivY = 0;
    if dy ~= 0
        xDivY = (pB(1) - pA(1)) / dy;
    end
    d = abs(x - ((height - pB(2)) * xDivY + pB(1)));
end

function lanes = insertLane(lanes, x, height)
% A new lane is inserted at the position its x has in this slice. Later slices append to
% existing lanes, so the order can drift from a strict x sort on a crossing pair; the
% original clusters the same way and chooseLanes walks the list in the same order, so the
% drift is kept rather than corrected.
    last = cellfun(@(L) L(end, 1), lanes);
    k = find(x < last, 1);
    if isempty(k)
        lanes{end + 1} = [x, height];
    else
        lanes = [lanes(1:k - 1), {[x, height]}, lanes(k:end)];
    end
end

function [left, right] = chooseLanes(lanes, w, A)
    left = zeros(0, 2);
    right = zeros(0, 2);
    lanes = lanes(cellfun(@(L) size(L, 1) >= A.MinPeaksForLane, lanes));
    long = A.Slices * A.OptimalPeakPerc;
    for k = 1:numel(lanes)
        lane = lanes{k};
        n = size(lane, 1);
        if lane(1, 1) <= w / 2
            if isempty(left)
                left = lane;
            elseif n > long || n > size(left, 1)
                left = lane;
            end
        else
            if isempty(right)
                right = lane;
                if n > long
                    break
                end
            elseif n > long
                right = lane;
                break
            end
        end
    end
end

function coef = fitLane(lane, A)
% x = a*y^2 + b*y + c in image pixels; degree 1 (padded with a = 0) for a short lane.
    coef = [];
    if isempty(lane)
        return
    end
    y = lane(:, 2);
    x = lane(:, 1);
    if size(lane, 1) > A.Slices * A.OptimalPeakPerc
        c = [y.^2, y, ones(size(y))] \ x;
        if abs(c(1)) < A.ExtremeCoef2
            coef = c';
        end
    else
        c = [y, ones(size(y))] \ x;
        if abs(c(1)) < A.ExtremeCoef1
            coef = [0, c'];
        end
    end
end

function perc = certainty(coef, prevCoef, nPts, realSlices, A)
% Half from how many slices carried a point, half from how much the fit moved since the
% previous frame. This temporal half is what our Strength in lka_fit_boundary lacks.
    perc = 0;
    if isempty(coef) || isempty(prevCoef)
        return
    end
    err = sqrt(mean((coef - prevCoef) .^ 2));
    similarity = max(0, 100 - err);
    peaksPerc = nPts / realSlices * 100;
    perc = round((A.CertaintyFromPeaks * peaksPerc + (1 - A.CertaintyFromPeaks) * similarity) * 100) / 100;
end
