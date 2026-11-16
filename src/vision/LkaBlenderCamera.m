classdef LkaBlenderCamera < matlab.System
% LKABLENDERCAMERA Camera image rendered by Blender at the simulated vehicle pose.
%
% MATLAB System block of LKA_Blender_TestBench (scripts/build_lka_blender_bench.m):
%   in:  rear-axle pose [x y psi] (track frame, from the Baja plant)
%   out: RGB image uint8 ImageSize(1) x ImageSize(2) x 3
% Each step sends the pose to tools/blender/render_server.py (started by
% run_lka_blender_sim) and reads back the rendered frame, so the camera sees exactly where
% the controller has driven the car. The camera intrinsics and mount are taken from
% Vision.Camera and sent to the server at setup. The pose is also published to the
% dashboard path view (lka_monitor_publish 'pose').

    properties (Nontunable)
        Vision = struct()          % LKA.Vision (Camera: ImageSize, FocalLength, PrincipalPoint, X, Height, PitchDeg)
        Ts = 0.05
        Host = '127.0.0.1'
        Port = 5560
        FrameFile = '/dev/shm/lka_blender_frame.jpg'
    end

    properties (Access = private)
        Client = []
        RenderMs = 0
    end

    methods
        function obj = LkaBlenderCamera(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Client = tcpclient(obj.Host, obj.Port, 'Timeout', 60, 'ConnectTimeout', 30);
            C = obj.Vision.Camera;
            rep = obj.call(struct('cmd', 'camera', 'f', C.FocalLength(1), 'cx', C.PrincipalPoint(1), ...
                'cy', C.PrincipalPoint(2), 'x', C.X, 'h', C.Height, 'pitch', C.PitchDeg));
            if rep.width ~= C.ImageSize(2) || rep.height ~= C.ImageSize(1)
                error('lka:blender:size', 'Render server makes %dx%d, the camera expects %dx%d.', ...
                    rep.width, rep.height, C.ImageSize(2), C.ImageSize(1));
            end
        end

        function img = stepImpl(obj, pose)
            rep = obj.call(struct('cmd', 'render', 'x', pose(1), 'y', pose(2), 'psi', pose(3), ...
                'out', obj.FrameFile));
            obj.RenderMs = rep.ms;
            img = imread(obj.FrameFile);
            lka_monitor_publish('pose', pose(1:2));
        end

        function releaseImpl(obj)
            obj.Client = [];
        end

        function sz = getOutputSizeImpl(obj)
            sz = [obj.Vision.Camera.ImageSize 3];
        end

        function dt = getOutputDataTypeImpl(~)
            dt = 'uint8';
        end

        function c = isOutputComplexImpl(~)
            c = false;
        end

        function f = isOutputFixedSizeImpl(~)
            f = true;
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end
    end

    methods (Access = private)
        function rep = call(obj, req)
            writeline(obj.Client, jsonencode(req));
            rep = jsondecode(readline(obj.Client));
            if ~rep.ok
                error('lka:blender:server', 'Render server: %s', rep.error);
            end
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
