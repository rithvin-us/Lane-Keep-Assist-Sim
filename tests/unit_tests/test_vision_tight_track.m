% ==============================================================================
% test_vision_tight_track.m
% Tight test lanes (college snake track: 2 m wide, bends of R 3-8 m). The default
% (official-track) settings fit such a lane almost straight; lka_set_track_profile
% 'tight' must follow it, and lka_lane_shape_error must flag the straight fit.
% Lanes are drawn directly in the bird's-eye grid (lka_synthetic_bev): no camera
% toolbox or Unreal needed.
% ==============================================================================

function tests = test_vision_tight_track
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    LKA = lka_set_lane_width(LKA, 2.0, MinFactor=0.5, MaxFactor=2);
    testCase.TestData.LKA = LKA;
    testCase.TestData.Tight = lka_set_track_profile(LKA, 'tight', MinRadius=3);
    % S-bend: lane centre 0.8 m to each side, 12 m wavelength (sharpest bend R 4.6 m)
    testCase.TestData.SBend = @(x) 0.8 * sin(2 * pi * (x - 4.5) / 12);
end

% ------------------------------------------------------------------ the problem
function testOfficialSettingsFlattenSBend(testCase)
    % Documents the straight-lane problem: with the official-track settings the fitted
    % lines miss the S-bend by more than half a metre.
    [bev, cam, lineY] = lka_synthetic_bev(testCase.TestData.SBend, 2.0);
    V = testCase.TestData.LKA.Vision;
    sensor = lka_vision_lane_step(bev, cam, V, lka_vision_state_init(V), 0.05);
    testCase.verifyGreaterThan(maxFitError(sensor.Right, lineY, -1, cam), 0.5);
end

function testShapeErrorFlagsStraightFit(testCase)
    [bev, cam] = lka_synthetic_bev(testCase.TestData.SBend, 2.0);
    V = testCase.TestData.LKA.Vision;
    sensor = lka_vision_lane_step(bev, cam, V, lka_vision_state_init(V), 0.05);
    err = lka_lane_shape_error(bev, cam, sensor);
    testCase.verifyGreaterThan(max(err), 0.5);
end

% ------------------------------------------------------------------ the fix
function testTightProfileFollowsSBend(testCase)
    [bev, cam, lineY] = lka_synthetic_bev(testCase.TestData.SBend, 2.0);
    V = testCase.TestData.Tight.Vision;
    sensor = lka_vision_lane_step(bev, cam, V, lka_vision_state_init(V), 0.05);
    sides = [1 -1];
    names = {'Left', 'Right'};
    for k = 1:2
        b = sensor.(names{k});
        testCase.verifyGreaterThan(b.Strength, 0.3, names{k});
        testCase.verifyGreaterThanOrEqual(b.ViewRange, cam.XNear + 5, names{k});
        testCase.verifyLessThan(maxFitError(b, lineY, sides(k), cam), 0.12, names{k});
    end
    err = lka_lane_shape_error(bev, cam, sensor);
    testCase.verifyLessThan(max(err), 0.1);
end

function testTightProfileFollowsStraightIntoTightBend(testCase)
    % Straight up to 8 m, then an R 8 m left-hand bend (heading 49 deg at 14 m). The near
    % part, which the controller steers on, must be fitted accurately.
    R = 8;
    centre = @(x) (x > 8) .* (R - sqrt(max(R^2 - (x - 8).^2, 0)));
    [bev, cam, lineY] = lka_synthetic_bev(centre, 2.0, 4.5, 14);
    V = testCase.TestData.Tight.Vision;
    sensor = lka_vision_lane_step(bev, cam, V, lka_vision_state_init(V), 0.05);
    sides = [1 -1];
    names = {'Left', 'Right'};
    for k = 1:2
        b = sensor.(names{k});
        testCase.verifyGreaterThan(b.Strength, 0.3, names{k});
        testCase.verifyLessThan(maxFitError(b, lineY, sides(k), cam), 0.12, names{k});
    end
end

function testTightProfileKeepsStraightAndGentleLanes(testCase)
    % On lanes the polynomial can follow, the tight settings change nothing that matters:
    % same accuracy and the full range.
    shapes = {@(x) 0 * x, @(x) (x - 4.5).^2 / (2 * 50)};
    V = testCase.TestData.Tight.Vision;
    for i = 1:numel(shapes)
        [bev, cam, lineY] = lka_synthetic_bev(shapes{i}, 2.0);
        sensor = lka_vision_lane_step(bev, cam, V, lka_vision_state_init(V), 0.05);
        testCase.verifyLessThan(maxFitError(sensor.Left, lineY, 1, cam), 0.03);
        testCase.verifyGreaterThan(sensor.Left.ViewRange, cam.XFar - 1.5);
    end
end

function testAdaptiveRangeStopsAtMinRange(testCase)
    % A zigzag no 5 m cubic can follow, starting at 3.05 m: (3.05 + 5) - 3.05 > 5 in
    % double precision, which kept the first version of the range loop running forever.
    F = testCase.TestData.Tight.Vision.Fit;
    x = (3.05:0.05:15)';
    y = 1 + 0.5 * sin(2 * pi * x / 3);
    t0 = tic;
    [b, ok] = lka_fit_boundary([x y 0.9 * ones(size(x))], F);
    testCase.verifyLessThan(toc(t0), 5);
    testCase.verifyTrue(ok);
    testCase.verifyEqual(b.ViewRange, 3.05 + F.MinRange, 'AbsTol', 0.06);
end

function testTraceLineFollowsPaint(testCase)
    [bev, cam, lineY] = lka_synthetic_bev(testCase.TestData.SBend, 2.0);
    [xs, ys] = lka_trace_line(bev, cam, lineY(cam.XNear, 1));
    testCase.verifyGreaterThan(max(xs), 15);
    testCase.verifyLessThan(max(abs(ys - lineY(xs, 1))), 0.06);
end

% ------------------------------------------------------------------ settings
function testDefaultsKeepOfficialBehaviour(testCase)
    LKA = testCase.TestData.LKA;
    testCase.verifyFalse(LKA.Vision.Search.PredictQuadratic);
    testCase.verifyEqual(LKA.Vision.Search.ReacquireGrow, 0);
    testCase.verifyFalse(LKA.Vision.Fit.AdaptiveRange);
    testCase.verifyFalse(LKA.Vision.Track.CheckAtNear);
    testCase.verifyEqual(LKA.Val.CheckX, 0);
    testCase.verifyEqual(LKA.Val.CurvatureMax, 0.1);
    testCase.verifyEqual(lka_set_track_profile(LKA, 'official'), LKA);
end

function testTightProfileOpensBothCurvatureGates(testCase)
    T = testCase.TestData.Tight;
    testCase.verifyEqual(T.Vision.Track.CurvatureMax, 1 / (0.8 * 3), 'AbsTol', 1e-12);
    testCase.verifyEqual(T.Val.CurvatureMax, T.Vision.Track.CurvatureMax);
    testCase.verifyGreaterThan(T.Val.CheckX, 0);
    testCase.verifyTrue(T.Vision.Fit.AdaptiveRange);
end

function testValidationAtCheckX(testCase)
    % Left line of a tight left bend seen from 4.5 m: extrapolated back to the rear axle
    % it lies right of the vehicle axis (rejected at x = 0), but at 4 m ahead it is where
    % the left line belongs.
    P = testCase.TestData.Tight;
    left = struct('LateralOffset', -0.7, 'HeadingAngle', deg2rad(10), 'Curvature', 0.15, ...
        'CurvatureDerivative', 0, 'Strength', 0.9, 'ViewRange', 12);
    right = struct('LateralOffset', -2.7, 'HeadingAngle', deg2rad(10), 'Curvature', 0.12, ...
        'CurvatureDerivative', 0, 'Strength', 0.9, 'ViewRange', 12);
    Val0 = P.Val; Val0.CheckX = 0;
    vL0 = lka_validate_boundaries(left, right, 0, 0, false, false, Val0, P.Lane);
    testCase.verifyFalse(vL0);
    [vL, vR] = lka_validate_boundaries(left, right, 0, 0, false, false, P.Val, P.Lane);
    testCase.verifyTrue(vL);
    testCase.verifyTrue(vR);
end

% ------------------------------------------------------------------ helpers
function e = maxFitError(b, lineY, side, cam)
% Largest distance between the fitted boundary and the true line over the fitted range.
    x = (cam.XNear:0.25:min(b.ViewRange, cam.XFar))';
    if b.Strength <= 0 || numel(x) < 2
        e = inf;
        return
    end
    y = b.LateralOffset + tan(b.HeadingAngle) * x + b.Curvature / 2 * x.^2 + b.CurvatureDerivative / 6 * x.^3;
    e = max(abs(y - lineY(x, side)));
end
