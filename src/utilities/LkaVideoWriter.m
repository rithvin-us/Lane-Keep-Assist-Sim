classdef LkaVideoWriter < handle
% LKAVIDEOWRITER MP4 writer that works where MATLAB has no MPEG-4 profile.
%
% Syntax:
%   w = LkaVideoWriter('out.mp4', 20);     % frame rate [Hz]
%   w.write(frame);                        % uint8 H x W x 3
%   w.close();                             % also on delete
%
% Uses VideoWriter 'MPEG-4' when the platform has it. On Linux MATLAB often has none
% ("The specified profile is not valid"): then it writes Motion JPEG AVI next to the
% target and transcodes to the same .mp4 path with ffmpeg on close (H.264, CRF 20, no
% B-frames: they shift the timestamps and VideoReader then drops the last frame). If
% ffmpeg is missing the AVI is kept and a warning names it.

    properties (SetAccess = private)
        File char = ''
        AviFile char = ''
    end
    properties (Access = private)
        Writer = []
        Frames = 0
    end

    methods
        function obj = LkaVideoWriter(file, frameRate, quality)
            if nargin < 3
                quality = 90;
            end
            obj.File = file;
            try
                obj.Writer = VideoWriter(file, 'MPEG-4');
                obj.Writer.FrameRate = frameRate;
                obj.Writer.Quality = quality;
                open(obj.Writer);
            catch
                [d, n] = fileparts(file);
                obj.AviFile = fullfile(d, [n '.avi']);
                obj.Writer = VideoWriter(obj.AviFile, 'Motion JPEG AVI');
                obj.Writer.FrameRate = frameRate;
                obj.Writer.Quality = quality;
                open(obj.Writer);
            end
        end

        function write(obj, frame)
            writeVideo(obj.Writer, frame);
            obj.Frames = obj.Frames + 1;
        end

        function close(obj)
            if isempty(obj.Writer)
                return
            end
            close(obj.Writer);
            obj.Writer = [];
            if obj.Frames == 0                     % run failed before the first frame: no file
                for f = {obj.AviFile, obj.File}
                    if ~isempty(f{1}) && isfile(f{1}), delete(f{1}); end
                end
                return
            end
            if isempty(obj.AviFile)
                return
            end
            cmd = sprintf('ffmpeg -y -loglevel error -i "%s" -c:v libx264 -bf 0 -pix_fmt yuv420p -crf 20 "%s"', ...
                obj.AviFile, obj.File);
            if system(cmd) == 0
                delete(obj.AviFile);
            else
                warning('lka:video:ffmpeg', 'ffmpeg failed or is missing: video left as %s', obj.AviFile);
            end
        end

        function delete(obj)
            obj.close();
        end
    end
end
