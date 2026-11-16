function det = lka_detector_laneassist(V)
% LKA_DETECTOR_LANEASSIST LaneAssist square-pulse row-histogram detector.
%
% Syntax:
%   det = lka_detector_laneassist(V)        V = LKA.Vision (uses V.LaneAssist)
%
% Port of the detector of tsiggi/LaneAssist (MIT, Christos-Alexandros Tsingiropoulos,
% VROOM - BFMC 2023); see third_party/LaneAssist_LICENSE.txt. Reference checkout in
% LaneAssist/ (git-ignored), comparison in docs/laneassist_comparison.md.
%
% Grayscale row histograms -> square pulses (lka_laneassist_peaks) -> clustering, left /
% right choice, image-plane polynomial and certainty (lka_laneassist_cluster) -> the two
% chosen polynomials painted as a probability map, page 1 left, page 2 right. A side
% their chooser rejected stays empty: no fallback page, so the rest of the chain sees
% Strength 0 and runs on one line, exactly as their lane keeping does.
%
% Unlike the other detectors this one is stateful by design (previous coefficients for
% the certainty, adaptive pulse threshold); det.Reset() clears that state, which the
% per-frame scoring in lka_detector_benchmark uses.
%
% The original lowers square_pulses_min_height towards (darkest accepted lane pixel - 20)
% each frame, but guards it with `tmp >= self.minimum`, where self.minimum is the
% configured threshold itself: as shipped the branch can only fire when tmp equals the
% threshold exactly, so the adaptation never lowers anything, while the reset branch
% (fewer than 10 chosen points) raises the threshold back to the configured value and
% destroys the lower setting their own real-world example applies after construction.
% A.AdaptiveFloor is therefore a separate floor, so the idea can be measured instead of
% the defect. A.AdaptiveFloor = A.MinHeight reproduces the original behaviour.

    A = V.LaneAssist;
    minHeight0 = A.MinHeight;
    prev = emptyPrev();

    det.Name = 'laneassist (LaneAssist square-pulse row histograms, ported)';
    det.Backend = 'matlab';
    det.OnGpu = false;
    det.Pages = 2;
    det.MaxRange = inf;                    % trusted look-ahead [m from the rear axle]
    det.Detect = @detect;
    det.DetectBatch = @detectBatch;
    det.Reset = @reset;

    function reset()
        A.MinHeight = minHeight0;
        prev = emptyPrev();
    end

    function [probs, info] = detectBatch(rgbs)
        n = size(rgbs, 4);
        probs = zeros(size(rgbs, 1), size(rgbs, 2), det.Pages, n, 'single');
        info = repmat(emptyInfo(), n, 1);
        for k = 1:n
            [probs(:, :, :, k), info(k)] = detect(rgbs(:, :, :, k));
        end
    end

    function [prob, info] = detect(rgb)
        t0 = tic;
        gray = double(gather(im2gray(rgb)));
        if ~isinteger(rgb) && max(gray(:)) <= 1
            gray = 255 * gray;             % single / double input in [0, 1]
        end
        imgSize = size(gray);

        [pts, slice, rows] = lka_laneassist_peaks(gray, A);
        [sel, lanes, res] = lka_laneassist_cluster(pts, slice, rows, imgSize, A, prev);

        prev.LeftCoef = res.LeftCoef;
        prev.RightCoef = res.RightCoef;
        prev.PrevTrustLk = res.OkLk;
        if A.Adaptive
            adapt(gray, sel);
        end

        painted = {};
        channels = {};
        if ~isempty(res.LeftCoef)
            painted{end + 1} = samplePoly(res.LeftCoef, sel.Left, rows, res.LeftCert);
            channels{end + 1} = [true, false, false];
        end
        if ~isempty(res.RightCoef)
            painted{end + 1} = samplePoly(res.RightCoef, sel.Right, rows, res.RightCert);
            channels{end + 1} = [false, true, false];
        end
        if isempty(painted)
            prob = zeros([imgSize det.Pages], 'single');
        else
            rgbMap = lka_rasterize_lanes(painted, imgSize, A.LineWidthPx, channels);
            prob = rgbMap(:, :, 1:det.Pages);
        end

        info = emptyInfo();
        info.Lanes = lanes;
        info.Selected = sel;
        info.Certainty = [res.LeftCert, res.RightCert];
        info.Trust = [res.TrustLeft, res.TrustRight, res.TrustLk];
        info.Coeffs = {res.LeftCoef, res.RightCoef};
        info.Width = [res.BottomWidth, res.TopWidth];
        info.MinHeight = A.MinHeight;
        info.Ms = 1000 * toc(t0);
    end

    function adapt(gray, sel)
        pick = @(P) gray(sub2ind(size(gray), P(:, 2), P(:, 1)));
        hl = pick(sel.Left);
        hr = pick(sel.Right);
        if ~isempty(hl) && ~isempty(hr)
            tmp = min(min(hl), min(hr)) - A.AdaptiveMargin;
            if tmp >= A.AdaptiveFloor && tmp <= minHeight0
                A.MinHeight = tmp;
            end
        end
        if size(sel.Left, 1) + size(sel.Right, 1) < A.AdaptiveResetPoints
            A.MinHeight = minHeight0;
        end
    end
end

% ------------------------------------------------------------------------------------ %

function P = samplePoly(coef, lane, rows, cert)
% The fitted polynomial over the rows the lane actually covers (no extrapolation), as
% [x y prob anchor] for lka_rasterize_lanes. prob carries the lane certainty so a shaky
% lane paints weaker, with a floor that keeps it above V.Search.ProbThreshold.
    if isempty(lane)
        y = rows(:);
    else
        y = rows(rows >= min(lane(:, 2)) & rows <= max(lane(:, 2)));
        y = y(:);
    end
    x = polyval(coef, y);
    w = max(0.6, min(1, cert / 100));
    P = [x, y, repmat(w, numel(y), 1), (1:numel(y))'];
end

function p = emptyPrev()
    p = struct('LeftCoef', [], 'RightCoef', [], 'PrevTrustLk', true);
end

function info = emptyInfo()
    info = struct('Lanes', {{}}, 'Ids', zeros(1, 0), 'Ms', 0, ...
        'Selected', struct('Left', zeros(0, 2), 'Right', zeros(0, 2)), ...
        'Certainty', [0 0], 'Trust', [false false false], ...
        'Coeffs', {{[], []}}, 'Width', [NaN NaN], 'MinHeight', NaN);
end
