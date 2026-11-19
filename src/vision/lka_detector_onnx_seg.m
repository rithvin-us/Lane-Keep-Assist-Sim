function det = lka_detector_onnx_seg(sidecarFile, useGpu, backend)
% LKA_DETECTOR_ONNX_SEG Lane detector from an ONNX segmentation model + JSON sidecar.
%
% Syntax:
%   det = lka_detector_onnx_seg(sidecarFile, useGpu)
%   det = lka_detector_onnx_seg(sidecarFile, useGpu, backend)   'auto' | 'matlab' | 'onnxruntime'
%   [prob, info] = det.Detect(rgb)          rgb uint8 H x W x 3 -> single H x W lane probability
%   [probs, info] = det.DetectBatch(rgbs, keepOnGpu)   H x W x 3 x N -> H x W x 1 x N
%
% Contract and sidecar fields: perception/README.md (type "seg").
% With the network on the GPU the whole chain runs there: letterbox, normalisation,
% network, softmax / sigmoid, un-letterbox and resize back to the camera image; only the
% probability map is copied back (or stays on the GPU with keepOnGpu = true, for
% lka_bev_warp). info(n).Ms: detection time per frame [ms]. Back end: lka_onnx_backend.

    if nargin < 3
        backend = 'auto';
    end
    cfg = jsondecode(fileread(sidecarFile));
    onnxFile = fullfile(fileparts(sidecarFile), cfg.onnx);
    if ~isfile(onnxFile)
        error('lka:vision:modelMissing', ['Lane model "%s" not found. Export it to ONNX as ' ...
            'described in perception/README.md (deploy/export_onnx.py).'], onnxFile);
    end
    be = lka_onnx_backend(onnxFile, useGpu, backend);
    inSize = cfg.input_size(:)';
    mean3 = reshape(single(cfg.mean), 1, 1, []);
    std3 = reshape(single(cfg.std), 1, 1, []);
    det.Name = sprintf('%s [%s%s]', cfg.name, be.Name, repmat(' GPU', 1, be.OnGpu));
    det.Backend = be.Name;
    det.OnGpu = be.OnGpu;
    det.Pages = 1;
    det.MaxRange = inf;                    % trusted look-ahead [m from the rear axle]
    if isfield(cfg, 'max_range_m')
        det.MaxRange = cfg.max_range_m;
    end
    det.Detect = @detectOne;
    det.DetectBatch = @detectBatch;

    function [prob, info] = detectOne(rgb)
        [prob, info] = detectBatch(rgb, false);
    end

    function [probs, info] = detectBatch(rgbs, keepOnGpu)
        if nargin < 2
            keepOnGpu = false;
        end
        t0 = tic;
        [h, w, ~, n] = size(rgbs);
        im = rgbs;
        if be.OnGpu
            im = gpuArray(im);
        end
        if strcmpi(cfg.channel_order, 'BGR')
            im = im(:, :, [3 2 1], :);
        end
        im = reshape(im, h, w, []);            % frames as extra channels: one resize call
        if strcmpi(cfg.resize, 'letterbox')
            [x, pad] = lka_letterbox(im, inSize, cfg.pad_value);
        else
            x = imresize(im, inSize, 'bilinear');
            pad = [0 0 inSize];
        end
        x = reshape(x, inSize(1), inSize(2), 3, n);
        x = (single(x) / single(cfg.scale) - mean3) ./ std3;
        outs = be.PredictBatch(x, be.OnGpu);
        P = zeros(pad(3), pad(4), n, 'like', x);
        for k = 1:n
            Y = lka_onnx_output_hwc(outs{1}{k}, inSize);
            switch lower(cfg.output)
                case 'softmax'
                    e = exp(Y - max(Y, [], 3));
                    p = e(:, :, cfg.lane_channel) ./ sum(e, 3);
                case 'sigmoid'
                    p = 1 ./ (1 + exp(-Y(:, :, cfg.lane_channel)));
                otherwise
                    p = Y(:, :, cfg.lane_channel);
            end
            P(:, :, k) = p(pad(1) + 1:pad(1) + pad(3), pad(2) + 1:pad(2) + pad(4));
        end
        if isgpuarray(P)
            P = lka_resize_pages(P, [h w], 'bilinear');
        else
            P = imresize(P, [h w], 'bilinear');
        end
        probs = reshape(single(min(max(P, 0), 1)), h, w, 1, n);
        if ~keepOnGpu
            probs = gather(probs);
        end
        info = repmat(struct('Lanes', {{}}, 'Ids', zeros(1, 0), 'Ms', 1000 * toc(t0) / n), n, 1);
    end
end
