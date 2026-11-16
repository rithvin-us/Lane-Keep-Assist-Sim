% ==============================================================================
% test_vision_system_objects.m
% LkaVisionLaneDetector / LkaVisionRecorder / lka_annotate_frame outside Simulink.
% ==============================================================================

function tests = test_vision_system_objects
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    V = LKA.Vision; V.Detector = 'mobilenet'; V.UseGpu = false;
    testCase.TestData.V = V;
    testCase.TestData.cam = lka_camera_model(V.Camera, V.Bev);
end

function testDetectorObjectStep(testCase)
    V = testCase.TestData.V;
    det = LkaVisionLaneDetector('Vision', V, 'Ts', 0.05);
    [img, ~, gt] = lka_render_lane_scene(testCase.TestData.cam, lka_default_scene());
    [sensor, preview, info] = det(img);
    testCase.verifyEqual(sensor.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.08);
    testCase.verifySize(preview, V.PreviewSize);
    testCase.verifyClass(preview, 'single');
    testCase.verifySize(info, [1 7]);
    testCase.verifyGreaterThan(info(7), 0);
    release(det);
end

function testDetectorResetClearsState(testCase)
    det = LkaVisionLaneDetector('Vision', testCase.TestData.V, 'Ts', 0.05);
    img = lka_render_lane_scene(testCase.TestData.cam, lka_default_scene());
    det(img);
    reset(det);
    [~, ~, info] = det(uint8(70 * ones(720, 1280, 3)));
    testCase.verifyEqual(info(3:4), [0 0]);
    release(det);
end

function testAnnotateDrawsLeftBoundaryGreen(testCase)
    cam = testCase.TestData.cam;
    [img, prob, gt] = lka_render_lane_scene(cam, lka_default_scene());
    b = gt.Left; b.Strength = 1; b.ViewRange = 30;
    r = gt.Right; r.Strength = 1; r.ViewRange = 30;
    sensor = struct('Left', b, 'Right', r, 'FrameValid', true);
    preview = imresize(transformImage(cam.Bev, prob), testCase.TestData.V.PreviewSize);
    frame = lka_annotate_frame(img, preview, sensor, struct('SteerDeg', 1.5, 'State', 2), cam);
    testCase.verifySize(frame, size(img));
    uv = round(vehicleToImage(cam.Sensor, [12 gt.Left.LateralOffset + tan(gt.Left.HeadingAngle) * 12]));
    px = squeeze(frame(uv(2), uv(1), :));
    testCase.verifyGreaterThan(double(px(2)), 200);
    testCase.verifyLessThan(double(px(1)), 80);
end

function testRecorderWritesVideo(testCase)
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    f = [tempname '.mp4'];
    rec = LkaVisionRecorder('Vision', V, 'Ts', 0.05, 'FileName', f);
    det = LkaVisionLaneDetector('Vision', V, 'Ts', 0.05);
    img = lka_render_lane_scene(cam, lka_default_scene());
    for k = 1:3
        [sensor, preview] = det(img);
        rec(img, preview, sensor, 0.5, uint8(2), 8.3, 120);
    end
    release(rec); release(det);
    testCase.verifyTrue(isfile(f));
    vr = VideoReader(f);
    testCase.verifyEqual(vr.NumFrames, 3);
end
