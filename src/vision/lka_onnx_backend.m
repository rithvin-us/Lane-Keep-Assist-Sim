function be = lka_onnx_backend(onnxFile, useGpu, backend)
% LKA_ONNX_BACKEND Runs an ONNX network on normalised H x W x C (x N) image tensors.
%
% Syntax:
%   be = lka_onnx_backend(onnxFile, useGpu)
%   be = lka_onnx_backend(onnxFile, useGpu, backend)    backend 'auto' | 'matlab' | 'onnxruntime'
%   outs = be.Predict(x)        x H x W x C           -> cell array of network outputs (CPU)
%   outs = be.PredictBatch(x)   x H x W x C x N       -> outs{output}{frame}, each frame in the
%                                                       same layout as be.Predict returns
%   be.OnGpu                    true when the network runs on the GPU
%
% 'matlab'      Deep Learning Toolbox importer add-on (dlnetwork, lka_import_onnx_detector).
%               On the GPU when useGpu and a supported GPU exist: the weights are moved to
%               the GPU once, the input may already be a gpuArray, predict runs with
%               Acceleration "auto" (traced and cached), and the network is warmed up here
%               so the first camera frame does not pay the tracing time.
%               With be.PredictBatch(x, true) the outputs stay on the GPU.
% 'onnxruntime' Python onnxruntime in-process (tools/lka_ort.py, CPU); works without the
%               add-on as long as MATLAB's pyenv Python has onnxruntime installed.
% 'auto'        'matlab' when the add-on is installed, else 'onnxruntime'.
% The output layout differs between back ends (dlarray SSCB vs ONNX NCHW); callers
% reorder by size (lka_onnx_output_hwc).
    if nargin < 3 || isempty(backend)
        backend = 'auto';
    end
    if strcmpi(backend, 'auto')
        if lka_onnx_runtime_available('matlab')
            backend = 'matlab';
        else
            backend = 'onnxruntime';
        end
    end
    switch lower(backend)
        case 'matlab'
            net = lka_import_onnx_detector(onnxFile);
            useGpu = useGpu && canUseGPU();
            if useGpu
                net = dlupdate(@gpuArray, net);      % weights to the GPU once
            end
            nOut = numel(net.OutputNames);
            be.Name = 'matlab';
            be.OnGpu = useGpu;
            be.Predict = @(x) firstFrame(predictMatlab(x, false));
            be.PredictBatch = @predictMatlab;
            if isa(net.Layers(1), 'nnet.cnn.layer.ImageInputLayer')
                x0 = zeros(net.Layers(1).InputSize, 'single');
                for iw = 1:3
                    be.Predict(x0);                  % trace + cache the batch-1 call
                end
            end
        case 'onnxruntime'
            if ~lka_onnx_runtime_available('onnxruntime')
                error('lka:vision:noOnnxBackend', ['No ONNX back end: install the add-on ' ...
                    '"Deep Learning Toolbox Converter for ONNX Model Format", or onnxruntime ' ...
                    'for the Python used by pyenv (pip install onnxruntime).']);
            end
            tools = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'tools');
            if count(py.sys.path, tools) == 0
                insert(py.sys.path, int32(0), tools);
            end
            mod = py.importlib.import_module('lka_ort');
            mod.load(onnxFile);
            be.Name = 'onnxruntime';
            be.OnGpu = false;
            be.Predict = @(x) firstFrame(predictOrt(x));
            be.PredictBatch = @(x, ~) predictOrt(x);
        otherwise
            error('lka:vision:unknownBackend', 'Unknown ONNX back end "%s".', backend);
    end

    function outs = predictMatlab(x, keepOnGpu)
        if nargin < 2
            keepOnGpu = false;
        end
        X = dlarray(single(x), 'SSCB');
        if useGpu && ~isgpuarray(X)
            X = gpuArray(X);
        end
        n = size(x, 4);
        y = cell(1, nOut);
        [y{:}] = predict(net, X, Acceleration="auto");
        outs = cell(1, nOut);
        for o = 1:nOut
            fmt = dims(y{o});
            Y = extractdata(y{o});
            if ~keepOnGpu
                Y = gather(Y);
            end
            bDim = find(fmt == 'B', 1);
            if isempty(bDim)
                bDim = 1;                            % unformatted outputs keep the ONNX order (N first)
            end
            outs{o} = splitBatch(Y, bDim, n);
        end
    end

    function outs = predictOrt(x)
        % frame by frame: the exported models fix the batch size to 1
        x = single(gather(x));
        n = size(x, 4);
        outs = {};
        for fk = 1:n
            raw = cellfun(@single, cell(mod.run(onnxFile, x(:, :, :, fk))), 'UniformOutput', false);
            if fk == 1
                outs = cell(1, numel(raw));
                outs(:) = {cell(1, n)};
            end
            for o = 1:numel(raw)
                outs{o}{fk} = raw{o};
            end
        end
    end
end

function frames = splitBatch(Y, bDim, n)
% One array per frame, the batch dimension kept as a singleton (layout of a batch-1 call).
    frames = cell(1, n);
    idx = repmat({':'}, 1, max(ndims(Y), bDim));
    for k = 1:n
        idx{bDim} = k;
        frames{k} = Y(idx{:});
    end
end

function outs = firstFrame(outsBatch)
    outs = cellfun(@(c) c{1}, outsBatch, 'UniformOutput', false);
end
