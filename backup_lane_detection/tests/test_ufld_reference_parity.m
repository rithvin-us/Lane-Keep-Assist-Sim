% ==============================================================================
% test_ufld_reference_parity.m
% The team CNN in MATLAB (importer add-on, GPU, lka_detector_ufld) against the reference
% inference of the team's Colab / cfzd demo.py (tools/ufld_reference.py: PIL +
% torchvision preprocessing, onnxruntime, original pred2coords). Same lanes, same lane
% slots, same points: the Colab accuracy carries over to MATLAB for the same images.
% Frames: recorded college-track clips (vids/) and a synthetic lane scene.
% Skipped without the model, the importer / onnxruntime, or Python torch + torchvision.
% ==============================================================================

function tests = test_ufld_reference_parity
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    testCase.TestData.root = root;
    run(fullfile(root, 'config', 'startup.m'));
    evalin('base', 'create_lka_buses;');
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    V = LKA.Vision; V.Detector = 'cnn'; V.UseGpu = true;
    model = fullfile(V.DetectorDir, 'team_cnn.onnx');
    testCase.assumeTrue(isfile(model) && lka_onnx_runtime_available(), 'team UFLDv2 or ONNX back end missing');
    py = 'python';
    try
        pe = pyenv;
        if strlength(pe.Executable) > 0, py = char(pe.Executable); end
    catch
    end
    [st, ~] = system(sprintf('"%s" -c "import torch, torchvision, onnxruntime, PIL"', py));
    testCase.assumeEqual(st, 0, 'Python torch / torchvision / onnxruntime / Pillow not available');

    % frames: three clips of the college track and one synthetic scene
    files = {};
    vids = dir(fullfile(root, 'vids', '*.mp4'));
    times = [4 12 20];
    for k = 1:min(3, numel(vids))
        vr = VideoReader(fullfile(vids(k).folder, vids(k).name));
        vr.CurrentTime = min(times(k), vr.Duration - 0.5);
        files{end + 1} = [tempname '.png']; %#ok<AGROW>
        imwrite(readFrame(vr), files{end});
    end
    cam = lka_camera_model(V.Camera, V.Bev);
    s = lka_default_scene(); s.Curvature = 1 / 200; s.Offset = 0.3;
    files{end + 1} = [tempname '.png'];
    imwrite(lka_render_lane_scene(cam, s), files{end});

    out = [tempname '.json'];
    args = strjoin(cellfun(@(f) ['"' f '"'], files, 'UniformOutput', false), ' ');
    [st, msg] = system(sprintf('"%s" "%s" "%s" "%s" %s', py, fullfile(root, 'tools', 'ufld_reference.py'), model, out, args));
    testCase.assertEqual(st, 0, msg);
    testCase.TestData.ref = jsondecode(fileread(out));
    testCase.TestData.files = files;
    testCase.TestData.det = lka_detector_create(V);
end

function testSameLaneSlots(testCase)
    ref = testCase.TestData.ref;
    for k = 1:numel(ref)
        [~, info] = testCase.TestData.det.Detect(imread(testCase.TestData.files{k}));
        refSlots = sort(reshape(arrayfun(@(l) l.slot, lanesOf(ref(k))), 1, []));
        testCase.verifyEqual(sort(info.Ids), refSlots, sprintf('frame %d', k));
    end
end

function testSamePoints(testCase)
    % pred2coords positions are 0-based continuous pixels, MATLAB adds 0.5 (pixel centre).
    % Allowed differences come from the resize only (PIL vs imresize antialiasing).
    ref = testCase.TestData.ref;
    du = [];
    common = 0; total = 0;
    for k = 1:numel(ref)
        [~, info] = testCase.TestData.det.Detect(imread(testCase.TestData.files{k}));
        L = lanesOf(ref(k));
        for j = 1:numel(L)
            i = find(info.Ids == L(j).slot, 1);
            if isempty(i), continue; end
            Pm = info.Lanes{i};
            Pr = reshape(L(j).points, [], 3);
            [c, im, ir] = intersect(Pm(:, 4), Pr(:, 3));
            common = common + numel(c);
            total = total + max(size(Pm, 1), size(Pr, 1));
            if L(j).row
                du = [du; Pm(im, 1) - 0.5 - Pr(ir, 1)]; %#ok<AGROW>
            else
                du = [du; Pm(im, 2) - 0.5 - Pr(ir, 2)]; %#ok<AGROW>
            end
        end
    end
    testCase.verifyGreaterThan(common / max(total, 1), 0.95, 'anchors found by both');
    testCase.verifyLessThan(median(abs(du)), 1.0, 'median point difference [px]');
    testCase.verifyLessThan(prctile(abs(du), 95), 4.0, '95th percentile point difference [px]');
    fprintf('[parity] %d common lane points, |du| median %.2f px, p95 %.2f px, max %.2f px\n', ...
        numel(du), median(abs(du)), prctile(abs(du), 95), max(abs(du)));
end

function L = lanesOf(r)
    L = r.lanes;
    if isempty(L)
        L = struct('slot', {}, 'row', {}, 'points', {});
    elseif iscell(L)
        L = [L{:}];
    end
end
