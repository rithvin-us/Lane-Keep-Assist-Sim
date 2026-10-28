function [prob, offset] = lka_paint_probability(rgb, P)
% LKA_PAINT_PROBABILITY Probability of bright white / yellow road paint per pixel.
%
% Syntax:
%   prob = lka_paint_probability(rgb)        rgb uint8 H x W x 3 (CPU or gpuArray) -> single H x W
%   [prob, offset] = lka_paint_probability(rgb, P)                 P = LKA.Vision.Classic
%
% p = logistic((V - boxmean41(V) - 0.12) / 0.03) on pixels that are white (S < 0.25) or
% yellow (hue 0.10-0.19, S > 0.35); V, S, hue from rgb2hsv. The classic detector and the
% paint check of the team CNN (lka_detector_ufld) use it; on a gpuArray it runs on the GPU.
%
% Called with P, the three constants come from P.Offset, P.Scale and P.BoxSize, and
% P.Adaptive moves the contrast offset with the frame: offset = P.Factor x the
% P.Percentile percentile of the positive contrast of the paint-coloured pixels, clamped
% to [P.OffsetMin, P.Offset]. A frame in shadow or behind a veiling glare has a lower
% paint contrast than the fixed 0.12 expects, which costs recall (finding L2 of
% docs/laneassist_comparison.md). tsiggi/LaneAssist adapt their pulse threshold towards
% the darkest pixel that still belonged to a chosen lane; we have no such geometric
% feedback inside the detector, so the frame's own contrast distribution stands in for it.
% P.OffsetMin matters: on a frame with no paint at all the percentile is asphalt texture,
% and only the clamp keeps the map from filling with it. P.Adaptive = false (the default)
% keeps the fixed constants.
%
% P.RowRange = [r0 r1] computes the map on those image rows only and returns zeros
% elsewhere. Everything above the horizon is discarded by the bird's-eye warp anyway, so
% the rows below it are the only ones worth the colour conversion and the box filter
% (finding L7 of docs/laneassist_comparison.md: we paid for 100 % of the pixels and used
% a fraction). Empty (the default) is the whole frame.

    if nargin < 2
        P = struct('Offset', 0.12, 'Scale', 0.03, 'BoxSize', 41, 'Adaptive', false);
    end
    if isfield(P, 'RowRange') && ~isempty(P.RowRange)
        r0 = max(1, P.RowRange(1));
        r1 = min(size(rgb, 1), P.RowRange(2));
        prob = zeros(size(rgb, 1), size(rgb, 2), 'single');
        if isgpuarray(rgb)
            prob = gpuArray(prob);
        end
        offset = P.Offset;
        if r1 < r0                      % horizon below the frame: nothing to compute
            return
        end
        Pc = P; Pc.RowRange = [];
        [prob(r0:r1, :), offset] = lka_paint_probability(rgb(r0:r1, :, :), Pc);
        return
    end
    hsv = rgb2hsv(rgb);
    val = hsv(:, :, 3);
    sat = hsv(:, :, 2);
    hue = hsv(:, :, 1);
    if isgpuarray(val)
        n = P.BoxSize;
        contrast = val - imfilter(val, ones(n, 'like', val) / n^2, 'replicate');   % imboxfilt has no GPU version
    else
        contrast = val - imboxfilt(val, P.BoxSize);
    end
    paint = sat < 0.25 | (hue > 0.10 & hue < 0.19 & sat > 0.35);

    offset = P.Offset;
    if P.Adaptive
        c = contrast(paint & contrast > 0);
        if ~isempty(c)
            cs = sort(c(:));
            idx = max(1, ceil(P.Percentile / 100 * numel(cs)));
            offset = min(P.Offset, max(P.OffsetMin, P.Factor * gather(cs(idx))));
        end
    end
    prob = single(1 ./ (1 + exp(-(contrast .* paint - offset) / P.Scale)));
end
