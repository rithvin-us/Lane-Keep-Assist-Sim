% ==============================================================================
% test_vision_gpu_pipeline.m
% GPU / batched camera pipeline: bird's-eye sampler (lka_bev_warp) against
% transformImage, matrix resize (lka_resize_pages) against imresize, batched against
% single-frame detection, UFLDv2 lane slots, per-side probability pages in the lane step,
% odometry next to the camera field-of-view border, live telemetry.
% GPU variants run only when a supported GPU is present.
% ==============================================================================

function tests = test_vision_gpu_pipeline
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(testCase.TestData.root, 'config', 'startup.m'));
    evalin('base', 'create_lka_buses;');
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    V = LKA.Vision; V.Detector = 'classic'; V.UseGpu = false;
    testCase.TestData.V = V;
    testCase.TestData.cam = lka_camera_model(V.Camera, V.Bev);
end

% ------------------------------------------------------------------ bird's-eye warp
function testBevWarpMatchesTransformImage(testCase)
    cam = testCase.TestData.cam;
    rng(3);
    I = single(imgaussfilt(rand(cam.Params.ImageSize), 2));
    A = transformImage(cam.Bev, I);
    B = lka_bev_warp(cam, I);
    testCase.verifySize(B, size(A));
    testCase.verifyEqual(double(B), double(A), 'AbsTol', 1e-5);
    if canUseGPU()
        G = lka_bev_warp(cam, gpuArray(I));
        testCase.verifyTrue(isgpuarray(G));
        testCase.verifyEqual(double(gather(G)), double(B), 'AbsTol', 1e-6);
    end
end

function testBevWarpPagesAndFrames(testCase)
    cam = testCase.TestData.cam;
    rng(4);
    I = single(rand([cam.Params.ImageSize 2 3]));
    B = lka_bev_warp(cam, I);
    testCase.verifySize(B, [cam.Sampler.Size 2 3]);
    testCase.verifyEqual(double(B(:, :, 2, 3)), double(lka_bev_warp(cam, I(:, :, 2, 3))), 'AbsTol', 1e-6);
    testCase.verifyError(@() lka_bev_warp(cam, I(1:10, :, 1)), 'lka:vision:imageSize');
end

% ------------------------------------------------------------------ matrix resize
function testResizePagesMatchesImresize(testCase)
    rng(5);
    X = randi(255, 464, 832, 3, 'uint8');
    for method = {'bilinear', 'box'}
        ref = imresize(X, [400 800], method{1});
        Y = lka_resize_pages(X, [400 800], method{1});
        % imresize rounds its uint8 result in its own fixed-point way: at most 1 grey level apart
        testCase.verifyLessThanOrEqual(max(abs(double(Y(:)) - double(ref(:)))), 1, method{1});
        testCase.verifyLessThan(mean(abs(double(Y(:)) - double(ref(:)))), 0.25, method{1});
    end
    Xs = single(X) / 255;
    testCase.verifyEqual(double(lka_resize_pages(Xs, [300 500], 'bilinear')), double(imresize(Xs, [300 500], 'bilinear')), 'AbsTol', 1e-4);
    crop = lka_resize_pages(X, [400 800], 'bilinear', 81:400);
    full = lka_resize_pages(X, [400 800], 'bilinear');
    testCase.verifyEqual(crop, full(81:400, :, :));
    if canUseGPU()
        G = gather(lka_resize_pages(gpuArray(X), [400 800], 'bilinear'));
        testCase.verifyLessThanOrEqual(max(abs(G(:) - full(:))), 1);
    end
end

% ------------------------------------------------------------------ detectors
function testClassicBatchMatchesSingle(testCase)
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    det = lka_detector_create(V);
    s1 = lka_default_scene(); s2 = s1; s2.Offset = 0.4;
    imgs = cat(4, lka_render_lane_scene(cam, s1), lka_render_lane_scene(cam, s2));
    [P, info] = det.DetectBatch(imgs);
    testCase.verifySize(P, [cam.Params.ImageSize 1 2]);
    testCase.verifyNumElements(info, 2);
    testCase.verifyEqual(P(:, :, 1, 2), det.Detect(imgs(:, :, :, 2)));
end

function testCnnBatchMatchesSingle(testCase)
    V = testCase.TestData.V; V.Detector = 'cnn'; V.UseGpu = true;
    testCase.assumeTrue(lka_onnx_runtime_available() && isfile(fullfile(V.DetectorDir, 'team_cnn.onnx')), ...
        'team UFLDv2 not available');
    cam = testCase.TestData.cam;
    det = lka_detector_create(V);
    s1 = lka_default_scene(); s2 = s1; s2.Offset = 0.4; s2.Curvature = 1 / 150;
    imgs = cat(4, lka_render_lane_scene(cam, s1), lka_render_lane_scene(cam, s2));
    [P, info] = det.DetectBatch(imgs);
    testCase.verifySize(P, [cam.Params.ImageSize 2 2]);
    for k = 1:2
        [p1, i1] = det.Detect(imgs(:, :, :, k));
        testCase.verifyLessThan(mean(abs(P(:, :, :, k) - p1), 'all'), 1e-3);
        testCase.verifyEqual(sort(info(k).Ids), sort(i1.Ids));
    end
    testCase.verifyTrue(all(ismember([2 3], info(1).Ids)), 'ego-left / ego-right slots on a centred straight');
end

function testUfldDecodeReturnsLaneSlots(testCase)
    cfg = jsondecode(fileread(fullfile(testCase.TestData.V.DetectorDir, 'cnn.json')));
    L = cfg.num_lanes;
    o.LocRow = zeros(cfg.num_grid_row, cfg.num_cls_row, L);
    o.LocCol = zeros(cfg.num_grid_col, cfg.num_cls_col, L);
    o.ExistRow = repmat([1; 0], 1, cfg.num_cls_row, L);
    o.ExistCol = repmat([1; 0], 1, cfg.num_cls_col, L);
    o.ExistRow(:, :, 3) = repmat([0; 1], 1, cfg.num_cls_row);   % only the ego-right slot
    o.LocRow(70, :, 3) = 10;
    o.ExistCol(:, :, 4) = repmat([0; 1], 1, cfg.num_cls_col);   % and the right-right column lane
    o.LocCol(50, :, 4) = 10;
    [lanes, ids] = lka_ufld_decode(o, cfg, [720 1280]);
    testCase.verifyEqual(ids, [3 4]);
    testCase.verifyNumElements(lanes, 2);
    rgb = lka_rasterize_lanes(lanes, [720 1280], 5, {[false true true], [false false true]});
    testCase.verifySize(rgb, [720 1280 3]);
    testCase.verifyEqual(nnz(rgb(:, :, 1)), 0);                 % nothing drawn into channel 1
    testCase.verifyGreaterThan(nnz(rgb(:, :, 2)), 0);
    testCase.verifyGreaterThan(nnz(rgb(:, :, 3)), nnz(rgb(:, :, 2)));
end

% ------------------------------------------------------------------ lane step pages
function testLaneStepReadsOwnPagePerSide(testCase)
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    sL = lka_default_scene(); sL.Missing = [false true];        % left line only
    sR = lka_default_scene(); sR.Missing = [true false];        % right line only
    [~, pL, gt] = lka_render_lane_scene(cam, sL);
    [~, pR] = lka_render_lane_scene(cam, sR);
    bev = lka_bev_warp(cam, cat(3, pL, pR));
    s = lka_vision_lane_step(bev, cam, V, lka_vision_state_init(V), 0.05);
    testCase.verifyGreaterThan([s.Left.Strength s.Right.Strength], [0 0]);
    testCase.verifyEqual(s.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.05);
    testCase.verifyEqual(s.Right.LateralOffset, gt.Right.LateralOffset, 'AbsTol', 0.05);
    % pages swapped: each side only sees the other side's line -> rejected (wrong side)
    s = lka_vision_lane_step(bev(:, :, [2 1]), cam, V, lka_vision_state_init(V), 0.05);
    testCase.verifyFalse(s.FrameValid);
end

% ------------------------------------------------------------------ odometry
function testOdometryIgnoresFieldOfViewBorder(testCase)
    % Outside the camera field of view both bird's-eye images are 0. That static border
    % used to pull the correlation peak to zero shift (speed read too low).
    cam = testCase.TestData.cam;
    rng(6);
    tex = imgaussfilt(rand(1400, 1400), 1.5);
    seen = reshape(sum(cam.Sampler.Weight, 2) > 0, cam.Sampler.Size);
    a = sampleTexture(tex, cam, [5 0 0]) .* seen;
    b = sampleTexture(tex, cam, [5.35 0.02 deg2rad(0.4)]) .* seen;
    m = lka_bev_odometry(a, b, cam);
    testCase.verifyEqual(m.dx, 0.35, 'AbsTol', 0.03);
    testCase.verifyEqual(m.dy, 0.02, 'AbsTol', 0.03);
    testCase.verifyEqual(rad2deg(m.dpsi), 0.4, 'AbsTol', 0.12);
end

function testOdometryBatchMatchesPairs(testCase)
    cam = testCase.TestData.cam;
    rng(7);
    tex = imgaussfilt(rand(1400, 1400), 1.5);
    poses = [5 0 0; 5.3 0 0.002; 5.62 0.01 0.004; 5.9 0.01 0.004];
    G = zeros([cam.Sampler.Size 4]);
    for k = 1:4
        G(:, :, k) = sampleTexture(tex, cam, poses(k, :));
    end
    mb = lka_bev_odometry(G(:, :, 1:3), G(:, :, 2:4), cam);
    for k = 1:3
        mk = lka_bev_odometry(G(:, :, k), G(:, :, k + 1), cam);
        testCase.verifyEqual([mb.dx(k) mb.dy(k) mb.dpsi(k)], [mk.dx mk.dy mk.dpsi], 'AbsTol', 1e-9);
    end
    testCase.verifyEqual(mb.dx, diff(poses(:, 1))', 'AbsTol', 0.03);
    if canUseGPU()
        mg = lka_bev_odometry(gpuArray(single(G(:, :, 1:3))), gpuArray(single(G(:, :, 2:4))), cam);
        testCase.verifyEqual(mg.dx, mb.dx, 'AbsTol', 5e-3);
    end
end

% ------------------------------------------------------------------ telemetry
function testMonitorPublishWritesLiveJson(testCase)
    f = fullfile(testCase.TestData.root, 'results', 'monitor', 'live.json');
    lka_monitor_publish('start', struct('Name', 'unit_test', 'Kind', 'test', 'Frames', 3, 'Phase', 'testing'));
    for k = 1:3
        lka_monitor_publish('sample', struct('Time', 0.05 * k, 'Steer', k, 'LeftOk', true), uint8(zeros(8, 8, 3)));
    end
    lka_monitor_publish('done', struct('Frames', 3));
    s = jsondecode(fileread(f));
    testCase.verifyEqual(s.Run.Name, 'unit_test');
    testCase.verifyEqual(s.Phase, 'done');
    testCase.verifyEqual(s.Latest.Steer, 3);
    testCase.verifyEqual(s.Series.Steer(:)', [1 2 3]);
    testCase.verifyTrue(isfile(fullfile(fileparts(f), 'live.jpg')));
    % 'ensure' starts a run only when none is active
    lka_monitor_publish('ensure', struct('Name', 'second', 'Kind', 'test'));
    testCase.verifyEqual(jsondecode(fileread(f)).Run.Name, 'second');      % previous one was done
    lka_monitor_publish('ensure', struct('Name', 'third', 'Kind', 'test'));
    testCase.verifyEqual(jsondecode(fileread(f)).Run.Name, 'second');      % 'second' still running
    lka_monitor_publish('done', struct());
end

% ------------------------------------------------------------------ helpers
function bev = sampleTexture(tex, cam, pose)
    [Y, X] = meshgrid(cam.ColY, cam.RowX);
    c = cos(pose(3)); s = sin(pose(3));
    xw = pose(1) + c * X - s * Y;
    yw = pose(2) + s * X + c * Y;
    bev = interp2(tex, 1 + (yw + 35) / 0.05, 1 + xw / 0.05, 'linear', 0);
end
