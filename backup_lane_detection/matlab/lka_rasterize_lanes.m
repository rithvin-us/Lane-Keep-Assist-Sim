function prob = lka_rasterize_lanes(lanes, imgSize, widthPx, channels)
% LKA_RASTERIZE_LANES Lane point lists -> lane-line probability map(s).
%
% Syntax:
%   prob = lka_rasterize_lanes(lanes, imgSize, widthPx)            H x W, all lanes
%   prob = lka_rasterize_lanes(lanes, imgSize, widthPx, channels)  H x W x 3
%
% lanes from lka_ufld_decode. Consecutive points of a lane (anchor gap <= 3) are joined
% by a widthPx line whose value is the smaller existence probability of its two ends.
% channels{k} (1 x 3 logical) selects the output channels lane k is drawn into, e.g.
% [ego-left, ego-right, all]: several maps from the same single insertShape call per
% frame (where lines of different channels cross, the later lane wins the pixel).
    if nargin < 4
        channels = repmat({[true true true]}, 1, numel(lanes));
        nOut = 1;
    else
        nOut = 3;
    end
    segs = zeros(0, 4);
    cols = zeros(0, 3);
    for k = 1:numel(lanes)
        P = lanes{k};
        if size(P, 1) < 2
            continue
        end
        join = diff(P(:, 4)) <= 3;
        a = P(1:end - 1, :);
        b = P(2:end, :);
        v = round(255 * min(a(join, 3), b(join, 3)));
        segs = [segs; a(join, 1:2), b(join, 1:2)]; %#ok<AGROW>
        cols = [cols; v .* double(channels{k})]; %#ok<AGROW>
    end
    canvas = zeros([imgSize 3], 'uint8');
    if ~isempty(segs)
        canvas = insertShape(canvas, 'Line', segs, 'Color', cols, 'LineWidth', widthPx, 'SmoothEdges', false);
    end
    prob = single(canvas(:, :, 1:nOut)) / 255;
end
