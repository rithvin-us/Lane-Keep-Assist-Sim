function report = lka_check_requirements(options)
% LKA_CHECK_REQUIREMENTS What this repository needs, and what is missing here.
%
% Syntax:
%   lka_check_requirements
%   report = lka_check_requirements(Quiet=true)
%
% Prints every MathWorks product, support package and file the repository uses, grouped by
% what stops working without it, and returns the same information as a table. The
% documented version of this list is requirements/ (README.md, matlab_toolboxes.md,
% system.md, python-requirements.txt); keep the two in step.
%
% Output: table with Item, Kind, Needed ('required' | 'detectors' | 'gpu' | 'optional'),
% Present, Enables.

    arguments
        options.Quiet (1,1) logical = false
    end

    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));   % src/utilities -> repo root
    installed = {ver().Name};
    rows = {};

    % product, needed, enables
    products = {
        'MATLAB',                            'required',  'everything'
        'Simulink',                          'required',  'every model and test bench'
        'Automated Driving Toolbox',         'required',  'monoCamera / birdsEyeView, Unreal co-simulation'
        'Computer Vision Toolbox',           'required',  'insertShape: lane rasterisation and annotation'
        'Image Processing Toolbox',          'required',  'imboxfilt, strel, bwlabel: classic detector, metrics'
        'Model Predictive Control Toolbox',  'required',  'the MPC controller; without it NO Simulink model compiles'
        'Control System Toolbox',            'required',  'prerequisite of the MPC Toolbox'
        'Deep Learning Toolbox',             'detectors', 'the cnn and yolop detectors'
        'Parallel Computing Toolbox',        'gpu',       'gpuArray paths and the GPU timing table'
        };
    for k = 1:size(products, 1)
        rows(end + 1, :) = {products{k, 1}, 'product', products{k, 2}, ...
            any(strcmp(installed, products{k, 1})), products{k, 3}}; %#ok<AGROW>
    end

    % support packages and files, probed by what they provide
    rows(end + 1, :) = {'Deep Learning Toolbox Converter for ONNX Model Format', 'support package', ...
        'detectors', exist('importNetworkFromONNX') > 0, 'importing yolop.onnx / team_cnn.onnx'};   %#ok<EXIST>
    rows(end + 1, :) = {'models/perception/team_cnn.onnx', 'file', 'detectors', ...
        isfile(fullfile(root, 'models', 'perception', 'team_cnn.onnx')), 'the team CNN detector (team Drive, > 100 MB)'};
    rows(end + 1, :) = {'models/perception/yolop_ll_384x640.onnx', 'file', 'detectors', ...
        isfile(fullfile(root, 'models', 'perception', 'yolop_ll_384x640.onnx')), 'the YOLOP detector'};
    rows(end + 1, :) = {'data/scenes/LKA/LKA.usd', 'file', 'required', ...
        isfile(fullfile(root, 'data', 'scenes', 'LKA', 'LKA.usd')), 'the Unreal camera bench'};
    rows(end + 1, :) = {'python onnxruntime', 'python', 'optional', ...
        lka_onnx_runtime_available('onnxruntime'), 'the CPU ONNX fallback (requirements/python-requirements.txt)'};
    rows(end + 1, :) = {'ffmpeg', 'system', 'optional', ...
        system('ffmpeg -version > /dev/null 2>&1') == 0, 'annotated video from a headless batch job'};
    rows(end + 1, :) = {'GPU device', 'hardware', 'gpu', ...
        exist('canUseGPU') > 0 && canUseGPU(), 'detector, warp and odometry on the GPU'};   %#ok<EXIST>

    report = cell2table(rows, 'VariableNames', {'Item', 'Kind', 'Needed', 'Present', 'Enables'});
    if options.Quiet
        return
    end

    groups = {'required', 'detectors', 'gpu', 'optional'};
    titles = {'REQUIRED', 'NEURAL DETECTORS (cnn, yolop)', 'GPU PATHS', 'OPTIONAL'};
    for g = 1:numel(groups)
        sel = report(strcmp(report.Needed, groups{g}), :);
        fprintf('\n%s\n', titles{g});
        for k = 1:height(sel)
            fprintf('  %s %-52s %s\n', mark(sel.Present(k)), sel.Item{k}, sel.Enables{k});
        end
    end
    missing = report(~report.Present & strcmp(report.Needed, 'required'), :);
    fprintf('\n');
    if isempty(missing)
        fprintf('All requirements for the MIL bench and the camera chain are met.\n');
    else
        fprintf('%d required item(s) missing: see requirements/matlab_toolboxes.md\n', height(missing));
    end
end

function s = mark(tf)
    if tf
        s = '[ok]     ';
    else
        s = '[MISSING]';
    end
end
