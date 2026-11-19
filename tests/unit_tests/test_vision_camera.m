% ==============================================================================
% test_vision_camera.m
% Camera model of the lane detector: pixel <-> vehicle frame (ISO 8855, rear axle).
% ==============================================================================

function tests = test_vision_camera
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    testCase.TestData.V = LKA.Vision;
end

function testPitchZeroRowToDistance(testCase)
    C = struct('FocalLength', [1109 1109], 'PrincipalPoint', [640 360], 'ImageSize', [720 1280], ...
        'X', 2.6, 'Y', 0, 'Height', 1.1, 'PitchDeg', 0);
    cam = lka_camera_model(C, testCase.TestData.V.Bev);
    xy = imageToVehicle(cam.Sensor, [640 560]);
    testCase.verifyEqual(xy(1), 2.6 + 1.1 * 1109 / 200, 'AbsTol', 1e-6);
    testCase.verifyEqual(xy(2), 0, 'AbsTol', 1e-9);
end

function testImageLeftIsVehicleLeft(testCase)
    cam = lka_camera_model(testCase.TestData.V.Camera, testCase.TestData.V.Bev);
    xy = imageToVehicle(cam.Sensor, [400 600]);
    testCase.verifyGreaterThan(xy(2), 0);
end

function testPitchDownMovesHorizonUp(testCase)
    cam = lka_camera_model(testCase.TestData.V.Camera, testCase.TestData.V.Bev);   % pitch 8 deg
    xy = imageToVehicle(cam.Sensor, [640 210]);    % 0.3 deg below the horizon (row 204.1)
    testCase.verifyGreaterThan(xy(1), 100);
end

function testBevAxes(testCase)
    cam = lka_camera_model(testCase.TestData.V.Camera, testCase.TestData.V.Bev);
    testCase.verifyLessThan(cam.RowX(2) - cam.RowX(1), 0);       % row 1 is the far end
    testCase.verifyLessThan(cam.ColY(2) - cam.ColY(1), 0);       % column 1 is the left edge
    testCase.verifyEqual(cam.RowX(end), cam.XNear, 'AbsTol', 0.06);
    testCase.verifyEqual(cam.ColY(1), testCase.TestData.V.Bev.HalfWidth, 'AbsTol', 0.06);
end

function testNearLimitIsInsideImage(testCase)
    cam = lka_camera_model(testCase.TestData.V.Camera, testCase.TestData.V.Bev);
    uv = vehicleToImage(cam.Sensor, [cam.XNear 0]);
    testCase.verifyLessThanOrEqual(uv(2), 720);
end

function testParamsInLkaStruct(testCase)
    V = testCase.TestData.V;
    testCase.verifyTrue(isfolder(V.DetectorDir));
    testCase.verifyTrue(isfile(V.Bench.ScenePath));
    testCase.verifyEqual(V.Ts, 0.05);
    testCase.verifyTrue(ismember(V.Detector, lka_detector_create('list')));
end
