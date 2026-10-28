% ==============================================================================
% test_lka_video_tools.m
% Tools for running the LKA on a recorded lane video: bird's-eye visual odometry
% (speed / yaw rate), camera auto-calibration, open-loop controller replay and the
% run_lka_on_video pipeline on a short synthetic video.
% ==============================================================================

function tests = test_lka_video_tools
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(testCase.TestData.root, 'config', 'startup.m'));
    evalin('base', 'create_lka_buses;');
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    V = LKA.Vision; V.Detector = 'classic'; V.UseGpu = false;
    testCase.TestData.LKA = LKA;
    testCase.TestData.V = V;
    testCase.TestData.cam = lka_camera_model(V.Camera, V.Bev);
end

% ------------------------------------------------------------------ odometry
function testOdometryRecoversMotion(testCase)
    cam = testCase.TestData.cam;
    rng(1);
    tex = imgaussfilt(rand(1400, 1400), 1.5);                 % road texture, 0.05 m/px
    poseA = [5 0 0];
    poseB = [5.42 0.03 deg2rad(0.6)];                        % 0.42 m forward, 3 cm left, 0.6 deg left
    a = sampleTexture(tex, cam, poseA);
    b = sampleTexture(tex, cam, poseB);
    m = lka_bev_odometry(a, b, cam);
    testCase.verifyEqual(m.dx, 0.42, 'AbsTol', 0.03);
    testCase.verifyEqual(m.dy, 0.03, 'AbsTol', 0.03);
    testCase.verifyEqual(rad2deg(m.dpsi), 0.6, 'AbsTol', 0.1);
end

function testOdometryStandstill(testCase)
    cam = testCase.TestData.cam;
    rng(2);
    tex = imgaussfilt(rand(1400, 1400), 1.5);
    a = sampleTexture(tex, cam, [5 0 0]);
    m = lka_bev_odometry(a, a, cam);
    testCase.verifyEqual([m.dx m.dy m.dpsi], [0 0 0], 'AbsTol', 1e-3);
end

% ------------------------------------------------------------------ auto-calibration
function testAutoCalibrationRecoversPitchAndHeight(testCase)
    V = testCase.TestData.V;
    Ctrue = V.Camera; Ctrue.PitchDeg = 6; Ctrue.Height = 1.35;
    camTrue = lka_camera_model(Ctrue, V.Bev);
    probs = cell(1, 4);
    offsets = [0 0.2 -0.2 0.1];
    for k = 1:4
        s = lka_default_scene(); s.Offset = offsets(k);
        [~, probs{k}] = lka_render_lane_scene(camTrue, s);
    end
    C0 = V.Camera;                                            % wrong guess: 8 deg, 1.20 m
    C = lka_autocalibrate_camera(probs, C0, V, LaneWidth=3.5);
    testCase.verifyEqual(C.PitchDeg, 6, 'AbsTol', 0.5);
    testCase.verifyEqual(C.Height, 1.35, 'AbsTol', 0.05);
end

% ------------------------------------------------------------------ controller replay
function testReplaySteersTowardsLaneCentre(testCase)
    LKA = testCase.TestData.LKA;
    t = (0:0.05:4)';
    n = numel(t);
    b = @(off) struct('LateralOffset', off, 'HeadingAngle', 0, 'Curvature', 0, ...
        'CurvatureDerivative', 0, 'Strength', 0.9, 'ViewRange', 25);
    sensors = repmat(struct('Left', b(1.75 + 0.3), 'Right', b(-1.75 + 0.3), 'FrameValid', true), n, 1);
    out = lka_replay_controller(t, sensors, 8 * ones(n, 1), zeros(n, 1), LKA);
    testCase.verifySize(out.SteerCmd, [n 1]);
    testCase.verifyEqual(out.State(end), 2);                % ACTIVE
    testCase.verifyGreaterThan(out.SteerCmd(end), 0);       % lane centre 0.3 m left -> steer left
    testCase.verifyGreaterThan(out.TurnRadius(end), 0);
end

% ------------------------------------------------------------------ full pipeline
function testRunOnSyntheticVideo(testCase)
    V = testCase.TestData.V; cam = testCase.TestData.cam;
    f = [tempname '.mp4'];
    vw = VideoWriter(f, 'MPEG-4'); vw.FrameRate = 20; open(vw);
    s = lka_default_scene(); s.Offset = 0.3;
    [img, ~, gt] = lka_render_lane_scene(cam, s);
    for k = 1:30
        writeVideo(vw, img);
    end
    close(vw);
    outDir = tempname;
    out = run_lka_on_video(f, Detector='classic', Camera=V.Camera, Speed=8, OutDir=outDir, Video=false);
    testCase.verifyTrue(isfile(fullfile(outDir, 'lka_video_results.csv')));
    testCase.verifyTrue(isfile(fullfile(outDir, 'lka_video_overview.png')));
    T = out.Table;
    testCase.verifyEqual(height(T), 30);
    testCase.verifyEqual(median(T.LeftOffset_m), gt.Left.LateralOffset, 'AbsTol', 0.08);
    % car 0.3 m left of the centre -> lane centre 0.3 m to its right -> steer right (< 0)
    testCase.verifyEqual(T.LaneCentreOffset_m(end), -0.3, 'AbsTol', 0.08);
    testCase.verifyLessThan(T.SteerCmd_deg(end), 0);
    % straight-lane alarm columns: the fitted lines lie on the painted lines
    testCase.verifyLessThan(median([T.LeftShapeError_m; T.RightShapeError_m], 'omitnan'), 0.05);
    testCase.verifyGreaterThan(median(T.LeftViewRange_m), 10);
end

% ------------------------------------------------------------------ helpers
function bev = sampleTexture(tex, cam, pose)
    [Y, X] = meshgrid(cam.ColY, cam.RowX);
    c = cos(pose(3)); s = sin(pose(3));
    xw = pose(1) + c * X - s * Y;
    yw = pose(2) + s * X + c * Y;
    bev = interp2(tex, 1 + (yw + 35) / 0.05, 1 + xw / 0.05, 'linear', 0);
end
