function det = lka_detector_ufld(sidecarFile, useGpu, backend)
% LKA_DETECTOR_UFLD Team CNN (UFLDv2) lane detector -> lane-line probability maps.
%
% Syntax:
%   det = lka_detector_ufld(sidecarFile, useGpu)
%   det = lka_detector_ufld(sidecarFile, useGpu, backend)   'auto' | 'matlab' | 'onnxruntime'
%   [prob, info] = det.Detect(rgb)             rgb uint8 H x W x 3 -> single H x W x P
%   [probs, info] = det.DetectBatch(rgbs)      rgbs H x W x 3 x N -> single H x W x P x N
%
% Preprocessing as the UFLDv2 test transform: resize to (input_size(1)/crop_ratio) x
% input_size(2), keep the bottom input_size(1) rows, scale and ImageNet-normalise
% (RGB); on the GPU when the network is. The four heads are decoded (lka_ufld_decode, a
% port of pred2coords) and rasterised (lka_rasterize_lanes), so the team CNN uses the same
% bird's-eye / fit / tracker chain as YOLOP.
% Lane identity (sidecar "ego_lanes", default [2 3]): UFLDv2 orders its lanes left to
% right, so slot 2 is the ego lane's left line and slot 3 its right line. Then P = 2:
% page 1 holds the ego-left line, page 2 the ego-right line, and the lane step searches
% each side only in its own page, so a kerb or the next lane (slots 1, 4) cannot be taken
% for an ego line. A page falls back to all lanes when its slot is empty. Without
% "ego_lanes" (or "ego_lanes": []) P = 1, all lanes in one map.
% info(n): Lanes (decoded points [u v p k], image pixels), Ids (lane slots), Ms.
% Back end: lka_onnx_backend.
    if nargin < 3
        backend = 'auto';
    end
    cfg = jsondecode(fileread(sidecarFile));
    onnxFile = fullfile(fileparts(sidecarFile), cfg.onnx);
    if ~isfile(onnxFile)
        error('lka:vision:modelMissing', ['Lane model "%s" not found. Export it to ONNX as ' ...
            'described in models/perception/README.md.'], onnxFile);
    end
    be = lka_onnx_backend(onnxFile, useGpu, backend);
    inSize = cfg.input_size(:)';
    fullH = floor(inSize(1) / cfg.crop_ratio + 1e-9);
    mean3 = reshape(single(cfg.mean), 1, 1, []);
    std3 = reshape(single(cfg.std), 1, 1, []);
    egoLanes = [2 3];
    if isfield(cfg, 'ego_lanes')
        egoLanes = reshape(cfg.ego_lanes, 1, []);
    end
    det.Name = sprintf('%s [%s%s]', cfg.name, be.Name, repmat(' GPU', 1, be.OnGpu));
    det.Backend = be.Name;
    det.OnGpu = be.OnGpu;
    det.Pages = 1 + (numel(egoLanes) == 2);
    det.MaxRange = inf;                    % trusted look-ahead [m from the rear axle]
    if isfield(cfg, 'max_range_m')
        det.MaxRange = cfg.max_range_m;
    end
    det.Detect = @detectOne;
    det.DetectBatch = @detectBatch;

    function [prob, info] = detectOne(rgb)
        [prob, info] = detectBatch(rgb);
    end

    function [probs, info] = detectBatch(rgbs)
        t0 = tic;
        [h, w, ~, n] = size(rgbs);
        im = rgbs;
        if be.OnGpu
            im = gpuArray(im);
        end
        if strcmpi(cfg.channel_order, 'BGR')
            im = im(:, :, [3 2 1], :);
        end
        if be.OnGpu
            x = lka_resize_pages(reshape(im, h, w, []), [fullH inSize(2)], 'bilinear', ...
                fullH - inSize(1) + 1:fullH);           % resize + bottom crop, one GPU call
        else
            x = imresize(reshape(im, h, w, []), [fullH inSize(2)], 'bilinear');
            x = x(end - inSize(1) + 1:end, :, :);
        end
        x = reshape(x, inSize(1), inSize(2), 3, n);
        x = (single(x) / single(cfg.scale) - mean3) ./ std3;
        outs = be.PredictBatch(x);
        probs = zeros(h, w, det.Pages, n, 'single');
        info = repmat(struct('Lanes', {{}}, 'Ids', zeros(1, 0), 'Ms', 0), n, 1);
        for k = 1:n
            o = lka_ufld_outputs(cellfun(@(c) c{k}, outs, 'UniformOutput', false), cfg);
            [lanes, ids] = lka_ufld_decode(o, cfg, [h w]);
            if det.Pages == 1
                probs(:, :, 1, k) = lka_rasterize_lanes(lanes, [h w], cfg.line_width_px);
            else
                % one raster call: channel 1 ego-left, 2 ego-right, 3 every lane (fallback)
                masks = arrayfun(@(i) [i == egoLanes(1), i == egoLanes(2), true], ids, 'UniformOutput', false);
                rgb = lka_rasterize_lanes(lanes, [h w], cfg.line_width_px, masks);
                for p = 1:2
                    if any(ids == egoLanes(p))
                        probs(:, :, p, k) = rgb(:, :, p);
                    else
                        probs(:, :, p, k) = rgb(:, :, 3);
                    end
                end
            end
            info(k).Lanes = lanes;
            info(k).Ids = ids;
        end
        ms = 1000 * toc(t0) / n;
        [info.Ms] = deal(ms);
    end
end
