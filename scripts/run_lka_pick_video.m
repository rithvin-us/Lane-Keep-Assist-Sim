function out = run_lka_pick_video(varargin)
% RUN_LKA_PICK_VIDEO Pick a video in a file dialog, run the LKA on it and show the results.
%
% Syntax:
%   out = run_lka_pick_video()                              % college-track defaults
%   out = run_lka_pick_video('TrackProfile', 'official')    % any run_lka_on_video option
%   out = run_lka_pick_video('CameraHeight', 1.05, 'HFovDeg', 66)
%
% For team members new to MATLAB: no paths or options to type. The chosen video is run
% through run_lka_on_video (Camera='auto' unless given) into test_res/<video name>/,
% then the overview figure opens and, on Windows, the annotated video plays. The result
% struct is returned (empty when the dialog is cancelled).

    root = fileparts(fileparts(mfilename('fullpath')));
    [file, folder] = uigetfile({'*.mp4;*.mov;*.MP4;*.MOV;*.avi', 'Videos (*.mp4, *.mov, *.avi)'}, ...
        'Choose a track video', fullfile(root, 'vids', filesep));
    out = [];
    if isequal(file, 0)
        fprintf('[LKA] no video chosen\n');
        return
    end
    [~, name] = fileparts(file);
    outDir = fullfile(root, 'test_res', name);
    args = varargin;
    names = lower(string(args(1:2:end)));
    if ~any(names == "camera"), args = [args, {'Camera', 'auto'}]; end
    if ~any(names == "outdir"), args = [args, {'OutDir', outDir}]; else, outDir = args{2 * find(names == "outdir", 1)}; end
    out = run_lka_on_video(fullfile(folder, file), args{:});

    overview = fullfile(outDir, 'lka_video_overview.png');
    if isfile(overview)
        figure('Name', ['LKA on ' name], 'NumberTitle', 'off');
        imshow(imread(overview));
    end
    annotated = fullfile(outDir, 'lka_video_annotated.mp4');
    if ispc && isfile(annotated)
        winopen(annotated);
    end
    fprintf('[LKA] results in %s\n', outDir);
end
