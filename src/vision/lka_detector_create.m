function det = lka_detector_create(V)
% LKA_DETECTOR_CREATE Lane detector selected by V.Detector.
%   'yolop' | 'cnn' | 'classic' | 'laneassist'
%
% Syntax:
%   det = lka_detector_create(V)          V = LKA.Vision
%   prob = det.Detect(rgb)
%
% 'yolop' and 'cnn' read models/perception/<name>.json; its "type" picks the adapter:
% 'seg' (pixel mask, lka_detector_onnx_seg) or 'ufld_v2' (anchors, lka_detector_ufld).
% 'laneassist' is the ported tsiggi/LaneAssist detector (no model file, V.LaneAssist
% parameters, docs/laneassist_comparison.md).
    switch lower(V.Detector)
        case {'yolop', 'cnn'}
            sidecar = fullfile(V.DetectorDir, [lower(V.Detector) '.json']);
            cfg = jsondecode(fileread(sidecar));
            backend = 'auto';
            if isfield(V, 'OnnxBackend')
                backend = V.OnnxBackend;
            end
            switch cfg.type
                case 'seg'
                    det = lka_detector_onnx_seg(sidecar, V.UseGpu, backend);
                case 'ufld_v2'
                    det = lka_detector_ufld(sidecar, V.UseGpu, backend);
                otherwise
                    error('lka:vision:unknownModelType', 'Unknown model type "%s" in %s.', cfg.type, sidecar);
            end
        case 'classic'
            if isfield(V, 'Classic') && V.Classic.RoiFromBev && isfield(V, 'Camera')
                % rows below the horizon: everything above it leaves the bird's-eye view
                C = V.Camera;
                horizon = C.PrincipalPoint(2) - C.FocalLength(2) * tan(deg2rad(C.PitchDeg));
                % the margin has to cover the box filter's half window, or the rows at
                % the horizon would see a truncated window and change value
                margin = max(V.Classic.RoiMargin, floor(V.Classic.BoxSize / 2));
                V.Classic.RowRange = [max(1, floor(horizon - margin)), C.ImageSize(1)];
            end
            det = lka_detector_classic(V);
        case 'laneassist'
            det = lka_detector_laneassist(V);
        otherwise
            error('lka:vision:unknownDetector', ...
                'Unknown lane detector "%s" (yolop | cnn | classic | laneassist).', V.Detector);
    end
end
