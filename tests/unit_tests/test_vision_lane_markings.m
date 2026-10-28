% ==============================================================================
% test_vision_lane_markings.m
% Dashed-line and horizontal-marking (stop line) detection on a synthetic metric
% bird's-eye view. Both run on run-length encoding only, so no toolbox is needed and
% every length in the test is in metres.
% ==============================================================================

function tests = test_vision_lane_markings
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(testCase.TestData.root, 'config', 'startup.m'));
    V = lka_vision_params('baja');
    testCase.TestData.D = V.Marking;
    testCase.TestData.cam = fakeCam();          % no Computer Vision Toolbox needed
end

function testSolidLineIsNotDashed(testCase)
    cam = testCase.TestData.cam;
    [bev, b] = laneBev(cam, inf, 0);            % painted without interruption
    [dashed, info] = lka_dashed_detect(bev, cam, b, testCase.TestData.D);
    testCase.verifyFalse(dashed);
    testCase.verifyEqual(info.Coverage, 1, 'AbsTol', 0.02);
    testCase.verifyEqual(info.Cycles, 0);
end

function testDashedLineIsDetectedWithMetricLengths(testCase)
    cam = testCase.TestData.cam;
    [bev, b] = laneBev(cam, 3, 6);              % 3 m dash, 6 m gap
    [dashed, info] = lka_dashed_detect(bev, cam, b, testCase.TestData.D);
    testCase.verifyTrue(dashed);
    testCase.verifyGreaterThanOrEqual(info.Cycles, 2);
    inner = info.DashLengths(info.DashLengths > 1);
    testCase.verifyEqual(median(inner), 3, 'AbsTol', 0.3);
    testCase.verifyEqual(median(info.GapLengths), 6, 'AbsTol', 0.3);
end

function testDashedNeedsAFoundBoundary(testCase)
    cam = testCase.TestData.cam;
    [bev, b] = laneBev(cam, 3, 6);
    b.Strength = 0;
    testCase.verifyFalse(lka_dashed_detect(bev, cam, b, testCase.TestData.D));
end

function testStopLineDistanceAndLength(testCase)
    cam = testCase.TestData.cam;
    D = testCase.TestData.D;
    bev = zeros(numel(cam.RowX), numel(cam.ColY), 'single');
    band = cam.RowX >= 12 & cam.RowX <= 12.4;   % 0.4 m stop line at 12 m
    bev(band, abs(cam.ColY) <= 1.6) = 1;
    [found, info] = lka_horizontal_detect(bev, cam, 3.5, D);
    testCase.verifyTrue(found);
    testCase.verifyEqual(info.Distance, 12.2, 'AbsTol', 0.2);
    testCase.verifyEqual(info.Length, 0.4, 'AbsTol', 0.1);
    testCase.verifyGreaterThan(info.Coverage, D.HorCoverage);
end

function testLaneLinesAreNotAStopLine(testCase)
    % Two longitudinal lines cover 6 of 71 columns per row: far below HorCoverage.
    cam = testCase.TestData.cam;
    [bev, ~] = laneBev(cam, inf, 0);
    testCase.verifyFalse(lka_horizontal_detect(bev, cam, 3.5, testCase.TestData.D));
end

function testStopLineTooLongIsRejected(testCase)
    % A painted junction area is wider than HorMax and must not be called a stop line.
    cam = testCase.TestData.cam;
    bev = zeros(numel(cam.RowX), numel(cam.ColY), 'single');
    bev(cam.RowX >= 12 & cam.RowX <= 16, abs(cam.ColY) <= 1.6) = 1;
    testCase.verifyFalse(lka_horizontal_detect(bev, cam, 3.5, testCase.TestData.D));
end

% ------------------------------------------------------------------------------------ %

function cam = fakeCam()
% The fields lka_dashed_detect / lka_horizontal_detect use from lka_camera_model:
% a 0.05 m/px bird's-eye grid from 4 m to 30 m, +-1.75 m.
    cam.XNear = 4;
    cam.XFar = 30;
    cam.RowX = (cam.XNear:0.05:cam.XFar)';
    cam.ColY = -1.75:0.05:1.75;
end

function [bev, b] = laneBev(cam, dashLen, gapLen)
% Straight lines at +-1.5 m, 3 columns (0.15 m) wide like 0.125 m paint. dashLen = inf
% paints them solid, otherwise the left line is broken into dashLen dashes separated by
% gapLen, starting at XNear.
    bev = zeros(numel(cam.RowX), numel(cam.ColY), 'single');
    [~, cRight] = min(abs(cam.ColY + 1.5));
    [~, cLeft] = min(abs(cam.ColY - 1.5));
    bev(:, cRight - 1:cRight + 1) = 1;
    if isinf(dashLen)
        bev(:, cLeft - 1:cLeft + 1) = 1;
    else
        phase = mod(cam.RowX - cam.XNear, dashLen + gapLen);
        bev(phase < dashLen, cLeft - 1:cLeft + 1) = 1;
    end
    b = struct('LateralOffset', 1.5, 'HeadingAngle', 0, 'Curvature', 0, ...
        'CurvatureDerivative', 0, 'Strength', 1, 'ViewRange', cam.XFar);
end
