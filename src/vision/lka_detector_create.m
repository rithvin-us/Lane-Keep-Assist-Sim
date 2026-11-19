function det = lka_detector_create(V)
% LKA_DETECTOR_CREATE Lane detector named by V.Detector (default 'mobilenet').
%
% Syntax:
%   det = lka_detector_create(V)          V = LKA.Vision
%   prob = det.Detect(rgb)                rgb uint8 H x W x 3 -> single H x W lane probability
%
% Every detector is one approach folder under perception/ (perception/README.md):
%   perception/<Approach>/deploy/<V.Detector>.json   sidecar ("type" picks the adapter)
%   perception/<Approach>/deploy/<model>.onnx        the network it names
% V.DetectorDir is the perception/ root (lka_build_params). Trying a new model therefore
% means adding a folder and setting V.Detector; nothing in the lane fit or the controller
% changes. lka_detector_create('list') prints the detectors that are installed.
%
% Archived detectors (classic, laneassist, yolop, UFLDv2 'cnn'): backup_lane_detection/.
    if ischar(V) || isstring(V)
        names = listDetectors(defaultRoot());
        fprintf('%s\n', names{:});
        det = names;
        return
    end
    root = V.DetectorDir;
    if isempty(root)
        root = defaultRoot();
    end
    sidecar = findSidecar(root, lower(V.Detector));
    cfg = jsondecode(fileread(sidecar));
    backend = 'auto';
    if isfield(V, 'OnnxBackend')
        backend = V.OnnxBackend;
    end
    switch cfg.type
        case 'seg'
            det = lka_detector_onnx_seg(sidecar, V.UseGpu, backend);
        otherwise
            error('lka:vision:unknownModelType', 'Unknown model type "%s" in %s.', cfg.type, sidecar);
    end
end

function root = defaultRoot()
    root = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'perception');
end

function f = findSidecar(root, name)
    hits = dir(fullfile(root, '*', 'deploy', [name '.json']));
    if isempty(hits)
        error('lka:vision:unknownDetector', ...
            'Unknown lane detector "%s". Installed: %s (perception/<Approach>/deploy/<name>.json).', ...
            name, strjoin(listDetectors(root), ', '));
    elseif numel(hits) > 1
        error('lka:vision:ambiguousDetector', 'Detector "%s" is defined in %d approach folders: %s.', ...
            name, numel(hits), strjoin({hits.folder}, ', '));
    end
    f = fullfile(hits.folder, hits.name);
end

function names = listDetectors(root)
    hits = dir(fullfile(root, '*', 'deploy', '*.json'));
    names = cellfun(@(n) n(1:end - 5), {hits.name}, 'UniformOutput', false);
end
