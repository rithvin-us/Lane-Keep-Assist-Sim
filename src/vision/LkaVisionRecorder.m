classdef LkaVisionRecorder < matlab.System
% LKAVISIONRECORDER Writes the annotated LKA camera video (lka_annotate_frame).
%
% Inputs: camera image, bird's-eye preview, LaneSensorBus, steering command [deg, + left],
% LKA state (LkaInfoBus.State), measured speed [m/s], commanded turning radius [m]. The video path is FileName, or Vision.Bench.VideoFile when
% FileName is empty (MATLAB System blocks take char properties literally, so Simulink
% models leave FileName empty and set LKA.Vision.Bench.VideoFile). No path: disabled.
% Vision.Bench.RawVideoFile (optional) also records the unannotated camera frames, e.g. as
% a test input for run_lka_on_video with known ground truth.
% Vision.Monitor (default true) publishes every frame to the live dashboard
% (lka_monitor_publish; the annotated frame is only drawn when it is actually sent).

    properties (Nontunable)
        Vision = struct()     % LKA.Vision
        Ts = 0.05             % frame period [s]
        FileName = ''         % .mp4 path (MATLAB use); empty = Vision.Bench.VideoFile
    end

    properties (Access = private)
        Writer
        RawWriter
        Cam
        Monitor = false
        Count = 0
    end

    methods
        function obj = LkaVisionRecorder(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Writer = [];
            obj.RawWriter = [];
            obj.Count = 0;
            obj.Monitor = ~isfield(obj.Vision, 'Monitor') || obj.Vision.Monitor;
            if obj.Monitor
                obj.Cam = lka_camera_model(obj.Vision.Camera, obj.Vision.Bev);
                det = '';
                if isfield(obj.Vision, 'Detector'), det = obj.Vision.Detector; end
                lka_monitor_publish('ensure', struct('Name', 'camera bench (Simulink)', 'Kind', 'sim', ...
                    'Detector', det, 'Phase', 'closed-loop simulation (Unreal camera)'));
            end
            if isfield(obj.Vision, 'Bench') && isfield(obj.Vision.Bench, 'RawVideoFile') ...
                    && ~isempty(obj.Vision.Bench.RawVideoFile)
                obj.RawWriter = LkaVideoWriter(obj.Vision.Bench.RawVideoFile, round(1 / obj.Ts), 95);
            end
            file = obj.FileName;
            if isempty(file) && isfield(obj.Vision, 'Bench')
                file = obj.Vision.Bench.VideoFile;
            end
            if isempty(file)
                return
            end
            folder = fileparts(file);
            if ~isempty(folder) && ~isfolder(folder)
                mkdir(folder);
            end
            if isempty(obj.Cam)
                obj.Cam = lka_camera_model(obj.Vision.Camera, obj.Vision.Bev);
            end
            obj.Writer = LkaVideoWriter(file, round(1 / obj.Ts));
        end

        function stepImpl(obj, img, preview, sensor, steerDeg, state, speed, radius)
            if ~isempty(obj.RawWriter)
                obj.RawWriter.write(img);
            end
            obj.Count = obj.Count + 1;
            radius = double(radius);
            hud = struct('SteerDeg', double(steerDeg), 'State', double(state), 'Time', (obj.Count - 1) * obj.Ts, ...
                'SpeedKmh', 3.6 * double(speed), 'TurnRadius', radius, 'CurvatureCmd', 1 / radius);
            if ~isempty(obj.Writer)
                obj.Writer.write(lka_annotate_frame(img, preview, sensor, hud, obj.Cam));
            end
            if obj.Monitor
                s = struct('Time', hud.Time, 'Frame', obj.Count, ...
                    'Left', sensor.Left.LateralOffset * (sensor.Left.Strength > 0), 'LeftOk', sensor.Left.Strength > 0, ...
                    'Right', sensor.Right.LateralOffset * (sensor.Right.Strength > 0), 'RightOk', sensor.Right.Strength > 0, ...
                    'Centre', centre(sensor), 'Heading', heading(sensor), 'Steer', hud.SteerDeg, 'State', hud.State, ...
                    'Speed', hud.SpeedKmh, 'Radius', radius);
                lka_monitor_publish('sample', s, @() lka_annotate_frame(img, preview, sensor, hud, obj.Cam));
            end
        end

        function releaseImpl(obj)
            if ~isempty(obj.Writer)
                obj.Writer.close();
                obj.Writer = [];
            end
            if ~isempty(obj.RawWriter)
                obj.RawWriter.close();
                obj.RawWriter = [];
            end
        end

        function n = getNumInputsImpl(~)
            n = 7;
        end

        function n = getNumOutputsImpl(~)
            n = 0;
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

function c = centre(sensor)
% Lane centre from both lines; NaN when one is missing (half an offset would mislead).
    c = NaN;
    if sensor.Left.Strength > 0 && sensor.Right.Strength > 0
        c = 0.5 * (sensor.Left.LateralOffset + sensor.Right.LateralOffset);
    end
end

function h = heading(sensor)
% Lane heading relative to the vehicle [deg]: mean of the reported lines, NaN without one.
    hs = [sensor.Left.HeadingAngle, sensor.Right.HeadingAngle];
    ok = [sensor.Left.Strength, sensor.Right.Strength] > 0;
    h = NaN;
    if any(ok), h = rad2deg(mean(hs(ok))); end
end
