classdef LkaVisionLaneDetector < matlab.System
% LKAVISIONLANEDETECTOR Camera image -> LaneSensorBus (MATLAB System block, interpreted).
%
% Detector (LKA.Vision.Detector: YOLOP | team CNN | classic) -> lane probability map ->
% metric bird's-eye view (lka_camera_model) -> lka_vision_lane_step.
% Outputs: 1 LaneSensorBus, 2 bird's-eye probability preview (single, PreviewSize),
%          3 [nPtsL nPtsR foundL foundR width msDetector msTotal].

    properties (Nontunable)
        Vision = struct()     % LKA.Vision
        Ts = 0.05             % frame period [s]
    end

    properties (Access = private)
        Cam
        Det
        State
    end

    methods
        function obj = LkaVisionLaneDetector(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Cam = lka_camera_model(obj.Vision.Camera, obj.Vision.Bev);
            obj.Det = lka_detector_create(obj.Vision);
            obj.Cam.XFar = min(obj.Cam.XFar, obj.Det.MaxRange);   % search/fit only the trusted range
            obj.State = lka_vision_state_init(obj.Vision);
        end

        function resetImpl(obj)
            obj.State = lka_vision_state_init(obj.Vision);
        end

        function [sensor, preview, info] = stepImpl(obj, img)
            t0 = tic;
            prob = obj.Det.Detect(img);
            msDet = 1000 * toc(t0);
            bev = lka_bev_warp(obj.Cam, prob);
            msWarp = 1000 * toc(t0) - msDet;
            [sensor, obj.State, info0] = lka_vision_lane_step(bev, obj.Cam, obj.Vision, obj.State, obj.Ts);
            preview = single(imresize(max(bev, [], 3), obj.Vision.PreviewSize, 'bilinear'));
            info = [info0, msDet, 1000 * toc(t0)];
            if isfield(obj.Vision, 'Monitor') && obj.Vision.Monitor     % dashboard latency chart
                lka_monitor_publish('extra', struct('DetectMs', msDet, 'WarpMs', msWarp, ...
                    'FitMs', info(end) - msDet - msWarp));
            end
        end

        function n = getNumInputsImpl(~)
            n = 1;
        end

        function n = getNumOutputsImpl(~)
            n = 3;
        end

        function [s1, s2, s3] = getOutputSizeImpl(obj)
            s1 = [1 1];
            s2 = obj.Vision.PreviewSize;
            s3 = [1 7];
        end

        function [d1, d2, d3] = getOutputDataTypeImpl(~)
            d1 = 'LaneSensorBus';
            d2 = 'single';
            d3 = 'double';
        end

        function [c1, c2, c3] = isOutputComplexImpl(~)
            c1 = false; c2 = false; c3 = false;
        end

        function [f1, f2, f3] = isOutputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true;
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end
end
