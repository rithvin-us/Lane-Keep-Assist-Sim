function [sensor, state, info] = lka_vision_lane_step(bevProb, cam, V, state, dt)
% LKA_VISION_LANE_STEP One camera frame: bird's-eye probability map -> LaneSensorBus.
%
% Syntax:
%   [sensor, state, info] = lka_vision_lane_step(bevProb, cam, V, state, dt)
%
% Per side: sliding-window search around the previous DIRECT fit (warm, +-WarmHalfWidth,
% at most Track.HoldTime old) or, cold, around the nominal lane +-width/2 and around the
% nearest line on that side in the first Search.SeedRange metres (any lane width), the
% second tried when the first fails (Search.SeedFirst swaps them); robust fit,
% plausibility (heading, curvature, side of the vehicle at the first visible row). With
% both sides found, the
% pair width at the first BEV row must match the width estimate (the weaker side is
% dropped otherwise) and updates it. The estimate is seeded from the first plausible
% pair and re-seeded after Track.WidthReseedFrames consecutive rejections, so a road
% whose width differs from WidthInit cannot lock out one side. Only direct fits become the next prior, which
% fixes ADAS_2026 finding M1 (a reconstructed side could lock out the real line).
% One-line operation, coasting and take-over stay in LKA_Controller (lane tracker,
% supervisor), so a missing side is reported with Strength 0.
%
% V.Track.TrustFrames > 1 reports a side only after that many consecutive plausible fits,
% and one miss resets the count (finding L1 of docs/laneassist_comparison.md: a wrong line
% that the geometric Strength cannot tell from a right one reached the controller
% unchallenged). The prior and the width estimate still use every direct fit. 1 is the
% default and reports every plausible fit, as before.
%
% Inputs:
%   bevProb - lka_bev_warp(cam, probabilityMap): H x W (one map for both sides) or
%             H x W x 2 (page 1 searched for the left line, page 2 for the right one,
%             e.g. the team CNN's ego-left / ego-right lane slots)
%   cam     - lka_camera_model
%   V       - LKA.Vision
%   state   - lka_vision_state_init / previous call
%   dt      - frame period [s]
% Outputs:
%   sensor  - LaneSensorBus struct (Left, Right, FrameValid)
%   info    - [nPtsLeft nPtsRight foundLeft foundRight widthEstimate]

    bevProb = double(gather(bevProb));
    names = {'Left', 'Right'};
    sides = [1, -1];
    W = state.Width;
    found = false(1, 2);
    bnd = cell(1, 2);
    nPts = zeros(1, 2);
    for k = 1:2
        map = bevProb(:, :, min(k, size(bevProb, 3)));
        prev = state.(names{k});
        if prev.Valid && prev.Age <= V.Track.HoldTime
            cf = prev.Coeffs;
            priors = struct('Fun', @(x) cf(1) + cf(2) * x + cf(3) * x.^2 + cf(4) * x.^3, ...
                'HalfWidth', V.Search.WarmHalfWidth, 'Cold', false);
        else
            % cold: nominal lane position and the nearest line on this side, one as the
            % fallback of the other (Search.SeedFirst)
            coldHalf = V.Search.ColdHalfWidth;
            if V.Search.ColdHalfWidthFromWidth
                % a 3.5 m window around a 2 m lane reaches past the far line
                coldHalf = max(0.35, min(coldHalf, 0.4 * W));
            end
            priors = struct('Fun', @(x) sides(k) * W / 2 + 0 * x, ...
                'HalfWidth', coldHalf, 'Cold', true);
            ySeed = nearestPeak(map, cam, sides(k), V.Search);
            if isfinite(ySeed)
                seed = struct('Fun', @(x) ySeed + 0 * x, 'HalfWidth', V.Search.SeedHalfWidth, 'Cold', true);
                if V.Search.SeedFirst || (V.Search.SeedFirstWhenUnseeded && ~state.WidthSeeded)
                    priors = [seed, priors]; %#ok<AGROW>
                else
                    priors = [priors, seed]; %#ok<AGROW>
                end
            end
        end
        for p = 1:numel(priors)
            prior = priors(p);
            prior.Side = sides(k);
            pts = lka_search_boundary(map, cam, prior, V.Search);
            nPts(k) = size(pts, 1);
            % only a prior still inside HoldTime is a "previous frame" for the
            % consistency term; after a longer coast it would score against a stale fit
            if prev.Valid && prev.Age <= V.Track.HoldTime
                [b, ok] = lka_fit_boundary(pts, V.Fit, prev.Coeffs);
            else
                [b, ok] = lka_fit_boundary(pts, V.Fit, zeros(1, 4));
            end
            % side check at the first visible row: on tight bends the boundary extrapolated
            % back to the rear axle can end up on the other side of the vehicle axis
            found(k) = ok && abs(b.HeadingAngle) <= V.Track.HeadingMax ...
                && abs(b.Curvature) <= V.Track.CurvatureMax && sides(k) * yAt(b, cam.XNear) > 0;
            bnd{k} = b;
            if found(k)
                break
            end
        end
    end

    if all(found)
        wPair = yAt(bnd{1}, cam.XNear) - yAt(bnd{2}, cam.XNear);
        plausible = wPair >= V.Track.WidthMin && wPair <= V.Track.WidthMax;
        if plausible && (~state.WidthSeeded || state.WidthRejects >= V.Track.WidthReseedFrames)
            state.Width = wPair;                      % first pair, or the road width changed
            state.WidthSeeded = true;
            state.WidthRejects = 0;
        elseif plausible && abs(wPair - W) <= V.Track.WidthTol
            state.Width = W + min(1, dt / V.Track.WidthTau) * (wPair - W);
            state.WidthRejects = 0;
        else
            state.WidthRejects = state.WidthRejects + plausible;
            [~, weak] = min([bnd{1}.Strength, bnd{2}.Strength]);
            found(weak) = false;
        end
    end

    % a side is reported once TrustFrames consecutive plausible fits agree; one miss
    % resets the count, so a side that flickers never reaches the controller
    report = found;
    for k = 1:2
        if found(k)
            state.(names{k}).Hits = state.(names{k}).Hits + 1;
        else
            state.(names{k}).Hits = 0;
        end
        report(k) = found(k) && state.(names{k}).Hits >= V.Track.TrustFrames;
    end

    empty = struct('LateralOffset', 0, 'HeadingAngle', 0, 'Curvature', 0, ...
        'CurvatureDerivative', 0, 'Strength', 0, 'ViewRange', 0);
    sensor = struct('Left', empty, 'Right', empty, 'FrameValid', any(report));
    for k = 1:2
        if found(k)
            b = bnd{k};
            if report(k)
                sensor.(names{k}) = b;
            end
            state.(names{k}).Valid = true;
            state.(names{k}).Age = 0;
            state.(names{k}).Coeffs = [b.LateralOffset, tan(b.HeadingAngle), b.Curvature / 2, b.CurvatureDerivative / 6];
        else
            state.(names{k}).Age = state.(names{k}).Age + dt;
            if state.(names{k}).Age > V.Track.HoldTime
                state.(names{k}).Valid = false;
            end
        end
    end
    info = [nPts, double(report), state.Width];
end

function y = yAt(b, x)
    y = b.LateralOffset + tan(b.HeadingAngle) * x + b.Curvature / 2 * x.^2 + b.CurvatureDerivative / 6 * x.^3;
end

function y = nearestPeak(bevProb, cam, side, S)
% Classic histogram start: the vehicle is inside its lane, so the nearest line on each
% side in the first SeedRange metres is that side's boundary (a kerb or the next lane
% line lies further out), whatever the lane width. NaN when that side shows no line.
    rows = cam.RowX >= cam.XNear & cam.RowX < cam.XNear + S.SeedRange;
    block = bevProb(rows, :);
    block(block < S.ProbThreshold) = 0;
    prof = sum(block, 1);
    prof(side * cam.ColY <= S.MinCenterDist) = 0;
    runs = bwlabel(prof > 0);
    y = NaN;
    for j = 1:max(runs)
        m = runs == j;
        if sum(prof(m)) >= S.SeedMass
            yRun = sum(prof(m) .* cam.ColY(m)) / sum(prof(m));
            if ~(abs(yRun) >= abs(y))
                y = yRun;
            end
        end
    end
end
