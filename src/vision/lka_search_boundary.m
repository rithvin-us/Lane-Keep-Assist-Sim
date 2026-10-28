function [pts, nHit] = lka_search_boundary(bevProb, cam, prior, S)
% LKA_SEARCH_BOUNDARY Sliding-window search of one lane boundary in the metric BEV.
%
% Syntax:
%   [pts, nHit] = lka_search_boundary(bevProb, cam, prior, S)      S = LKA.Vision.Search
%
% The BEV is cut into SliceLength slices from cam.XNear to cam.XFar. In each slice a
% window of +-prior.HalfWidth around the expected boundary position is searched; the
% column profile above ProbThreshold is split into runs and the run closest to the
% window centre is taken (a second line or a road arrow inside the window is ignored).
% The expected position comes from the prior for the first hit, then from a straight
% line through the last (up to 6) slice peaks. A cold prior (no previous fit) keeps the
% window on its own side of the vehicle (|y| > MinCenterDist) until the first hit.
%
% With S.TaperExp > 0 the two pixel-count thresholds follow the range: the same painted
% ground area maps from fewer camera pixels the further away it is, so beyond S.TaperRef
% the required slice mass is divided by (x/TaperRef)^TaperExp and the half-width that
% feeds the fit is multiplied by it, up to S.LineHalfWidthMax. TaperExp = 0 (the default)
% keeps both constant over range, which tightens the effective threshold with distance
% (finding L3 of docs/laneassist_comparison.md; their image-plane equivalent is a pulse
% width limit that narrows with image height).
%
% Inputs:
%   bevProb - BEV probability image (double, rows = cam.RowX, columns = cam.ColY)
%   prior   - .Fun @(x) expected y [m], .HalfWidth [m], .Side +1/-1, .Cold logical
% Outputs:
%   pts     - [x y w] support pixels [m, m, probability]
%   nHit    - number of slices with a detection

    rowX = cam.RowX;
    colY = cam.ColY;
    edges = cam.XNear:S.SliceLength:cam.XFar;
    pts = zeros(0, 3);
    nHit = 0;
    px = zeros(1, 0); py = zeros(1, 0);    % slice peaks found so far
    gap = 0;
    for k = 1:numel(edges) - 1
        rows = rowX >= edges(k) & rowX < edges(k + 1);
        if ~any(rows)
            continue
        end
        xMid = 0.5 * (edges(k) + edges(k + 1));
        minMass = S.MinSliceMass;
        halfLine = S.LineHalfWidth;
        if S.TaperExp > 0
            f = max(1, (xMid / S.TaperRef) ^ S.TaperExp);
            minMass = max(1, minMass / f);
            halfLine = min(S.LineHalfWidthMax, halfLine * f);
        end
        yc = expected(xMid);
        cols = colY >= yc - prior.HalfWidth & colY <= yc + prior.HalfWidth;
        if prior.Cold && isempty(px)
            % first hit only: on a tight curve the line may later cross the vehicle axis
            cols = cols & prior.Side * colY > S.MinCenterDist;
        end
        cIdx = find(cols);
        block = bevProb(rows, cols);
        block(block < S.ProbThreshold) = 0;
        prof = sum(block, 1);
        runs = bwlabel(prof > 0);
        best = 0; bestDist = inf; yPeak = NaN;
        for j = 1:max(runs)
            m = runs == j;
            mass = sum(prof(m));
            if mass < minMass
                continue
            end
            yRun = sum(prof(m) .* colY(cIdx(m))) / mass;
            if abs(yRun - yc) < bestDist
                best = j; bestDist = abs(yRun - yc); yPeak = yRun;
            end
        end
        if best == 0
            gap = gap + S.SliceLength;
            if gap > S.MaxGap && nHit > 0
                break
            end
            continue
        end
        gap = 0;
        nHit = nHit + 1;
        px(end + 1) = xMid; py(end + 1) = yPeak; %#ok<AGROW>
        [r, cc] = find(block);
        wv = block(block > 0);
        rIdx = find(rows);
        xs = reshape(rowX(rIdx(r)), [], 1);
        ys = reshape(colY(cIdx(cc)), [], 1);
        near = abs(ys - yPeak) <= halfLine;
        pts = [pts; xs(near), ys(near), wv(near)]; %#ok<AGROW>
    end

    function y = expected(xq)
        if numel(px) >= 3
            n = min(numel(px), 6);
            p = polyfit(px(end - n + 1:end), py(end - n + 1:end), 1);
            y = polyval(p, xq);
        elseif ~isempty(px)
            y = prior.Fun(xq) + (py(end) - prior.Fun(px(end)));
        else
            y = prior.Fun(xq);
        end
    end
end
