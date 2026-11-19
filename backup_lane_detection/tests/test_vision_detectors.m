% ==============================================================================
% test_vision_detectors.m
% Lane detectors: letterbox, classic detector, factory errors, UFLDv2 decoding,
% YOLOP / UFLDv2 ONNX parity (skipped until the ONNX importer and models exist).
% ==============================================================================

function tests = test_vision_detectors
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(testCase.TestData.root, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    testCase.TestData.V = LKA.Vision;
    testCase.TestData.cam = lka_camera_model(LKA.Vision.Camera, LKA.Vision.Bev);
end

function testLetterbox720pTo384x640(testCase)
    img = uint8(randi(255, 720, 1280, 3));
    [out, pad, scale] = lka_letterbox(img, [384 640], 114);
    testCase.verifySize(out, [384 640 3]);
    testCase.verifyEqual(pad, [12 0 360 640]);
    testCase.verifyEqual(scale, 0.5);
    testCase.verifyTrue(all(out(1:12, :, :) == 114, 'all'));
    testCase.verifyTrue(all(out(373:384, :, :) == 114, 'all'));
end

function testClassicDetectorOnSyntheticFrame(testCase)
    V = testCase.TestData.V; V.Detector = 'classic'; cam = testCase.TestData.cam;
    det = lka_detector_create(V);
    [img, ~, gt] = lka_render_lane_scene(cam, lka_default_scene());
    prob = det.Detect(img);
    uv = round(vehicleToImage(cam.Sensor, [10 1.75; 10 0]));
    testCase.verifyGreaterThan(prob(uv(1, 2), uv(1, 1)), 0.8);
    testCase.verifyLessThan(prob(uv(2, 2), uv(2, 1)), 0.1);
    sensor = lka_vision_lane_step(double(transformImage(cam.Bev, prob)), cam, V, lka_vision_state_init(V), 0.05);
    testCase.verifyEqual(sensor.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.08);
    testCase.verifyEqual(sensor.Right.LateralOffset, gt.Right.LateralOffset, 'AbsTol', 0.08);
end

function testUnknownDetectorErrors(testCase)
    V = testCase.TestData.V; V.Detector = 'magic';
    testCase.verifyError(@() lka_detector_create(V), 'lka:vision:unknownDetector');
end

function testMissingCnnModelErrors(testCase)
    V = testCase.TestData.V; V.Detector = 'cnn';
    testCase.assumeFalse(isfile(fullfile(V.DetectorDir, 'team_cnn.onnx')), 'team CNN already delivered');
    testCase.verifyError(@() lka_detector_create(V), 'lka:vision:modelMissing');
end

function testOutputReorder(testCase)
    Y = rand(1, 2, 384, 640);
    Z = lka_onnx_output_hwc(Y, [384 640]);
    testCase.verifySize(Z, [384 640 2]);
    testCase.verifyEqual(Z(5, 7, 2), Y(1, 2, 5, 7));
end

function testYolopImportMatchesOnnxRuntime(testCase)
    testCase.assumeEqual(exist('importNetworkFromONNX', 'file'), 2, 'ONNX importer add-on not installed');
    onnx = fullfile(testCase.TestData.root, 'models', 'perception', 'yolop_ll_384x640.onnx');
    rng(7);
    x = single(0.5 * randn(384, 640, 3));
    prefix = onnxReference(testCase, onnx, x);
    yRef = lka_onnx_output_hwc(readOnnxOutput(prefix, 1), [384 640]);
    net = lka_import_onnx_detector(onnx);
    y = lka_onnx_output_hwc(extractdata(predict(net, dlarray(x, 'SSCB'))), [384 640]);
    testCase.verifyLessThan(max(abs(y(:) - yRef(:))), 1e-3);
end

function testYolopDetectorRuns(testCase)
    testCase.assumeTrue(lka_onnx_runtime_available(), 'no ONNX back end (importer add-on or Python onnxruntime)');
    V = testCase.TestData.V; V.Detector = 'yolop';
    det = lka_detector_create(V);
    img = lka_render_lane_scene(testCase.TestData.cam, lka_default_scene());
    det.Detect(img);                                   % warm-up (import cache, GPU)
    t0 = tic; prob = det.Detect(img); ms = 1000 * toc(t0);
    testCase.verifySize(prob, [720 1280]);
    testCase.verifyGreaterThanOrEqual(min(prob(:)), 0);
    testCase.verifyLessThanOrEqual(max(prob(:)), 1);
    fprintf('[vision] YOLOP %.1f ms/frame (GPU %d)\n', ms, V.UseGpu && canUseGPU());
end

function testOnnxRuntimeBackendMatchesReference(testCase)
    % MATLAB -> Python (in-process onnxruntime) must give the same tensors as the
    % stand-alone reference: checks the H x W x C -> NCHW marshaling.
    testCase.assumeTrue(lka_onnx_runtime_available('onnxruntime'), 'Python onnxruntime not available');
    onnx = fullfile(testCase.TestData.root, 'models', 'perception', 'yolop_ll_384x640.onnx');
    rng(9);
    x = single(0.5 * randn(384, 640, 3));
    prefix = onnxReference(testCase, onnx, x);
    be = lka_onnx_backend(onnx, false, 'onnxruntime');
    outs = be.Predict(x);
    testCase.verifyEqual(be.Name, 'onnxruntime');
    testCase.verifyEqual(size(outs{1}), [1 2 384 640]);
    testCase.verifyLessThan(max(abs(outs{1}(:) - reshape(readOnnxOutput(prefix, 1), [], 1))), 1e-4);
end

% ------------------------------------------------------------------ UFLDv2 (team CNN)
function testUfldDecodeSynthetic(testCase)
    cfg = jsondecode(fileread(fullfile(testCase.TestData.V.DetectorDir, 'cnn.json')));
    o = emptyUfld(cfg);
    o.ExistRow(:, :, 2) = repmat([0; 1], 1, cfg.num_cls_row);   % lane 2 exists at every row anchor
    o.LocRow(41, :, 2) = 10;                                    % arg-max cell 41 (0-based 40)
    lanes = lka_ufld_decode(o, cfg, [720 1280]);
    testCase.verifyNumElements(lanes, 1);
    P = lanes{1};
    testCase.verifySize(P, [cfg.num_cls_row 4]);
    e = exp([0 10 0] - 10);
    uExp = (sum(e .* [39 40 41]) / sum(e) + 0.5) / (cfg.num_grid_row - 1) * 1280 + 0.5;
    testCase.verifyEqual(P(:, 1), repmat(uExp, cfg.num_cls_row, 1), 'AbsTol', 1e-9);
    testCase.verifyEqual(P(1, 2), 160.5, 'AbsTol', 1e-9);
    testCase.verifyEqual(P(end, 2), 710.5, 'AbsTol', 1e-9);
    prob = lka_rasterize_lanes(lanes, [720 1280], cfg.line_width_px);
    testCase.verifyGreaterThan(prob(400, round(uExp)), 0.5);
    testCase.verifyEqual(prob(400, 100), single(0));
end

function testUfldShortLaneRejected(testCase)
    cfg = jsondecode(fileread(fullfile(testCase.TestData.V.DetectorDir, 'cnn.json')));
    o = emptyUfld(cfg);
    o.ExistRow(:, 1:20, 2) = repmat([0; 1], 1, 20);             % only 20 of 56 anchors (< half)
    testCase.verifyEmpty(lka_ufld_decode(o, cfg, [720 1280]));
end

function testUfldOutputsIdentifiedByShape(testCase)
    cfg = jsondecode(fileread(fullfile(testCase.TestData.V.DetectorDir, 'cnn.json')));
    outs = {rand(2, 41, 4, 'single'), rand(56, 4, 100, 'single'), ...     % mixed order and layouts
            rand(1, 100, 41, 4, 'single'), rand(1, 2, 56, 4, 'single')};
    o = lka_ufld_outputs(outs, cfg);
    testCase.verifySize(o.LocRow, [100 56 4]);
    testCase.verifySize(o.LocCol, [100 41 4]);
    testCase.verifySize(o.ExistRow, [2 56 4]);
    testCase.verifySize(o.ExistCol, [2 41 4]);
    testCase.verifyEqual(o.LocRow(7, 3, 2), outs{2}(3, 2, 7));
end

function testUfldImportMatchesOnnxRuntime(testCase)
    testCase.assumeEqual(exist('importNetworkFromONNX', 'file'), 2, 'ONNX importer add-on not installed');
    d = testCase.TestData.V.DetectorDir;
    onnx = fullfile(d, 'team_cnn.onnx');
    testCase.assumeTrue(isfile(onnx), 'team UFLDv2 ONNX not delivered yet');
    cfg = jsondecode(fileread(fullfile(d, 'cnn.json')));
    rng(8);
    x = single(0.5 * randn(320, 800, 3));
    prefix = onnxReference(testCase, onnx, x);
    refs = cell(1, 4);
    for k = 1:4
        refs{k} = readOnnxOutput(prefix, k);
    end
    oRef = lka_ufld_outputs(refs, cfg);
    net = lka_import_onnx_detector(onnx);
    outs = cell(1, numel(net.OutputNames));
    [outs{:}] = predict(net, dlarray(x, 'SSCB'));
    o = lka_ufld_outputs(cellfun(@extractdata, outs, 'UniformOutput', false), cfg);
    for f = ["LocRow", "LocCol", "ExistRow", "ExistCol"]
        testCase.verifyLessThan(max(abs(o.(f)(:) - oRef.(f)(:))), 1e-3, f);
    end
end

function testUfldDetectorRuns(testCase)
    testCase.assumeTrue(lka_onnx_runtime_available(), 'no ONNX back end (importer add-on or Python onnxruntime)');
    V = testCase.TestData.V; V.Detector = 'cnn';
    testCase.assumeTrue(isfile(fullfile(V.DetectorDir, 'team_cnn.onnx')), 'team UFLDv2 ONNX not delivered yet');
    det = lka_detector_create(V);
    img = lka_render_lane_scene(testCase.TestData.cam, lka_default_scene());
    det.Detect(img);
    t0 = tic; prob = det.Detect(img); ms = 1000 * toc(t0);
    testCase.verifyEqual(det.Pages, 2);                % ego-left / ego-right lane slots
    testCase.verifySize(prob, [720 1280 2]);
    testCase.verifyGreaterThan(nnz(prob(:, :, 1) > 0.5), 100);   % both ego lines of the scene
    testCase.verifyGreaterThan(nnz(prob(:, :, 2) > 0.5), 100);
    fprintf('[vision] UFLDv2 %.1f ms/frame (GPU %d)\n', ms, V.UseGpu && canUseGPU());
end

function testDetectorMaxRange(testCase)
    % Each detector says how far ahead its lane points can be trusted; the team UFLDv2
    % bends the far points of tight curves the wrong way, so its fit stops at 18 m.
    V = testCase.TestData.V; V.Detector = 'classic';
    testCase.verifyEqual(lka_detector_create(V).MaxRange, inf);
    cfg = jsondecode(fileread(fullfile(V.DetectorDir, 'cnn.json')));
    testCase.verifyEqual(cfg.max_range_m, 18);
    testCase.assumeTrue(lka_onnx_runtime_available() && isfile(fullfile(V.DetectorDir, 'team_cnn.onnx')), ...
        'team UFLDv2 not available');
    V.Detector = 'cnn';
    testCase.verifyEqual(lka_detector_create(V).MaxRange, 18);
end

% ------------------------------------------------------------------ helpers
function o = emptyUfld(cfg)
    L = cfg.num_lanes;
    o.LocRow = zeros(cfg.num_grid_row, cfg.num_cls_row, L);
    o.LocCol = zeros(cfg.num_grid_col, cfg.num_cls_col, L);
    o.ExistRow = repmat([1; 0], 1, cfg.num_cls_row, L);         % channel 1 wins: not present
    o.ExistCol = repmat([1; 0], 1, cfg.num_cls_col, L);
end

function prefix = onnxReference(testCase, onnx, x)
    root = testCase.TestData.root;
    inBin = [tempname '.bin'];
    prefix = tempname;
    fid = fopen(inBin, 'w'); fwrite(fid, permute(x, [2 1 3]), 'single'); fclose(fid);   % NCHW, C order
    [st, msg] = system(sprintf('python "%s" "%s" "%s" "%s"', ...
        fullfile(root, 'tools', 'onnx_reference.py'), onnx, inBin, prefix));
    testCase.assumeEqual(st, 0, ['onnxruntime reference unavailable: ' msg]);
end

function Y = readOnnxOutput(prefix, k)
    shapes = jsondecode(fileread([prefix '.json']));
    if iscell(shapes), s = shapes{k}(:)'; else, s = shapes(k, :); end
    fid = fopen(sprintf('%s_%d.bin', prefix, k), 'r');
    y = fread(fid, inf, 'single=>single');
    fclose(fid);
    Y = permute(reshape(y, fliplr(s)), numel(s):-1:1);     % C order -> ONNX dimension order
end
