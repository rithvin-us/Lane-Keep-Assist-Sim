function [lanes, ids] = lka_ufld_decode(o, cfg, imgSize)
% LKA_UFLD_DECODE UFLDv2 anchor outputs -> lane points in image pixels.
%
% Syntax:
%   [lanes, ids] = lka_ufld_decode(o, cfg, imgSize)   o from lka_ufld_outputs
%
% Port of pred2coords (cfzd/Ultra-Fast-Lane-Detection-v2). An anchor exists when
% exist channel 2 beats channel 1. A row lane (cfg.row_lanes) is used when more than half
% of its row anchors exist, a column lane (cfg.col_lanes) when more than a quarter of its
% column anchors exist. Position = expectation of the softmax over +-local_width cells
% around the arg-max cell, + 0.5, scaled by (num_grid - 1) to the image size.
% Output: cell array, one [u v p k] matrix per lane: MATLAB pixel coordinates (u, v),
% existence probability p, anchor index k; ids(i) = network lane slot of lanes{i}
% (TuSimple order, left to right: 1 left-left, 2 ego-left, 3 ego-right, 4 right-right).
% pred2coords returns 0-based continuous pixel
% positions; + 0.5 converts them to MATLAB's 1-based pixel-centre convention.
% The loc heads must be raw logits (as exported by deploy/pt2onnx.py): the local
% window softmax is taken here.
    H = imgSize(1);
    W = imgSize(2);
    rowAnchor = linspace(cfg.row_anchor_px(1), cfg.row_anchor_px(2), cfg.num_cls_row) / cfg.anchor_image_height;
    colAnchor = linspace(0, 1, cfg.num_cls_col);
    lanes = {};
    ids = zeros(1, 0);
    for lane = reshape(cfg.row_lanes, 1, [])
        q = decodeLane(o.LocRow(:, :, lane), o.ExistRow(:, :, lane), cfg.num_cls_row / 2, W, cfg.local_width);
        if ~isempty(q)
            lanes{end + 1} = [q(:, 1) + 0.5, reshape(rowAnchor(q(:, 3)), [], 1) * H + 0.5, q(:, 2:3)]; %#ok<AGROW>
            ids(end + 1) = lane; %#ok<AGROW>
        end
    end
    for lane = reshape(cfg.col_lanes, 1, [])
        q = decodeLane(o.LocCol(:, :, lane), o.ExistCol(:, :, lane), cfg.num_cls_col / 4, H, cfg.local_width);
        if ~isempty(q)
            lanes{end + 1} = [reshape(colAnchor(q(:, 3)), [], 1) * W + 0.5, q(:, 1) + 0.5, q(:, 2:3)]; %#ok<AGROW>
            ids(end + 1) = lane; %#ok<AGROW>
        end
    end
end

function q = decodeLane(loc, ex, minCount, extent, localWidth)
% q = [position (0-based pixels), existence probability, anchor index]
    G = size(loc, 1);
    valid = ex(2, :) > ex(1, :);
    q = zeros(0, 3);
    if nnz(valid) <= minCount
        return
    end
    pEx = 1 ./ (1 + exp(ex(1, :) - ex(2, :)));
    for k = find(valid)
        [~, m] = max(loc(:, k));
        idx = (max(1, m - localWidth):min(G, m + localWidth))';
        e = exp(loc(idx, k) - max(loc(idx, k)));
        cell0 = sum(e .* (idx - 1)) / sum(e) + 0.5;
        q(end + 1, :) = [cell0 / (G - 1) * extent, pEx(k), k]; %#ok<AGROW>
    end
end
