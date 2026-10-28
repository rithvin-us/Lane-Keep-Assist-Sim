% ==============================================================================
% test_detector_laneassist.m
% Ported LaneAssist detector: square-pulse row scan, clustering and lane choice,
% certainty and its hysteresis, and (where the Computer Vision Toolbox is available)
% the probability map, the metric full-stack sensor and the closed detector contract.
% The pulse scan and the clustering are pure MATLAB, so those tests run anywhere.
% ==============================================================================

function tests = test_detector_laneassist
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(testCase.TestData.root, 'config', 'startup.m'));
    testCase.TestData.A = lka_vision_params('baja').LaneAssist;
    [testCase.TestData.gray, testCase.TestData.truth] = perspectiveLanes();
end

function testPulseScanFindsLineCentres(testCase)
    A = testCase.TestData.A;
    [pts, slice, rows] = lka_laneassist_peaks(testCase.TestData.gray, A);
    testCase.verifyEqual(numel(rows), 20);
    testCase.verifyEqual(rows(1), 720 - A.BottomOffset);
    testCase.verifyTrue(all(diff(rows) < 0));                  % bottom to top
    testCase.verifyEqual(size(pts, 1), 40);                    % two lines per row
    testCase.verifyEqual(numel(slice), size(pts, 1));
    testCase.verifyLessThan(max(abs(pts(:, 1) - testCase.TestData.truth(pts))), 2);
end

function testPulseScanIgnoresWideAndFlatRegions(testCase)
    A = testCase.TestData.A;
    gray = 70 * ones(720, 1280);
    gray(:, 300:700) = 235;                                    % far too wide to be paint
    testCase.verifyEmpty(lka_laneassist_peaks(gray, A));
    gray = 200 * ones(720, 1280);                              % above MinHeight, no edges
    testCase.verifyEmpty(lka_laneassist_peaks(gray, A));
end

function testPulseScanSurvivesGlobalBrightnessShift(testCase)
    % The pulse test is a local contrast test, so a dimmer frame keeps the same columns
    % as long as the paint stays above MinHeight: the property our fixed classic
    % thresholds lack (finding L2 of docs/laneassist_comparison.md).
    A = testCase.TestData.A;
    pts = lka_laneassist_peaks(testCase.TestData.gray, A);
    dim = lka_laneassist_peaks(max(0, testCase.TestData.gray - 50), A);
    testCase.verifyEqual(size(dim, 1), size(pts, 1));
    testCase.verifyEqual(dim(:, 1), pts(:, 1));
end

function testClusteringSplitsTheTwoLines(testCase)
    A = testCase.TestData.A;
    [pts, slice, rows] = lka_laneassist_peaks(testCase.TestData.gray, A);
    [sel, lanes, out] = lka_laneassist_cluster(pts, slice, rows, [720 1280], A, emptyPrev());
    testCase.verifyEqual(numel(lanes), 2);
    testCase.verifyEqual(size(sel.Left, 1), 20);
    testCase.verifyEqual(size(sel.Right, 1), 20);
    testCase.verifyTrue(all(sel.Left(:, 1) < 640) && all(sel.Right(:, 1) > 640));
    testCase.verifySize(out.LeftCoef, [1 3]);
    testCase.verifySize(out.RightCoef, [1 3]);
    % x = a*y^2 + b*y + c back on the painted columns
    testCase.verifyLessThan(max(abs(polyval(out.LeftCoef, sel.Left(:, 2)) - sel.Left(:, 1))), 2);
    testCase.verifyGreaterThan(out.BottomWidth, out.TopWidth);  % perspective
end

function testCertaintyNeedsAPreviousFrameAndTrustHasHysteresis(testCase)
    A = testCase.TestData.A;
    [pts, slice, rows] = lka_laneassist_peaks(testCase.TestData.gray, A);
    args = {pts, slice, rows, [720 1280], A};
    [~, ~, f1] = lka_laneassist_cluster(args{:}, emptyPrev());
    testCase.verifyEqual([f1.LeftCert f1.RightCert], [0 0]);    % no previous coefficients
    testCase.verifyFalse(f1.TrustLk);

    prev = struct('LeftCoef', f1.LeftCoef, 'RightCoef', f1.RightCoef, 'PrevTrustLk', f1.OkLk);
    [~, ~, f2] = lka_laneassist_cluster(args{:}, prev);
    testCase.verifyGreaterThan(f2.LeftCert, A.MinSingleCertainty);
    testCase.verifyTrue(f2.OkLk);
    testCase.verifyFalse(f2.TrustLk);                           % one frame of hysteresis
    prev.PrevTrustLk = f2.OkLk;
    [~, ~, f3] = lka_laneassist_cluster(args{:}, prev);
    testCase.verifyTrue(f3.TrustLk);
end

function testCertaintyGapDistrustsTheWeakerLane(testCase)
    % The right line painted over the nearest 10 of the 20 slices: half the points, so
    % its certainty drops by 0.5 * 52 points and the gap crosses AllowedCertaintyDif.
    % 16 of 20 slices would give a gap of only 10.5 and both lanes would stay trusted.
    A = testCase.TestData.A;
    gray = perspectiveLanes(540:720);
    [pts, slice, rows] = lka_laneassist_peaks(gray, A);
    args = {pts, slice, rows, [720 1280], A};
    [~, ~, f1] = lka_laneassist_cluster(args{:}, emptyPrev());
    prev = struct('LeftCoef', f1.LeftCoef, 'RightCoef', f1.RightCoef, 'PrevTrustLk', true);
    [~, ~, f2] = lka_laneassist_cluster(args{:}, prev);
    testCase.verifyGreaterThan(f2.LeftCert - f2.RightCert, A.AllowedCertaintyDif);
    testCase.verifyTrue(f2.TrustLeft);
    testCase.verifyFalse(f2.TrustRight);
end

function testExtremeCoefficientRejection(testCase)
    A = testCase.TestData.A;
    A.ExtremeCoef2 = 1e-9;                                      % reject every curved fit
    A.ExtremeCoef1 = 1e-9;
    [pts, slice, rows] = lka_laneassist_peaks(testCase.TestData.gray, A);
    [~, ~, out] = lka_laneassist_cluster(pts, slice, rows, [720 1280], A, emptyPrev());
    testCase.verifyEmpty(out.LeftCoef);
    testCase.verifyEmpty(out.RightCoef);
    testCase.verifyFalse(out.TrustLeft);
end

function testDetectorContractAndProbabilityMap(testCase)
    assumeToolbox(testCase);
    V = lka_vision_params('baja'); V.Detector = 'laneassist'; V.UseGpu = false;
    det = lka_detector_create(V);
    testCase.verifyEqual(det.Pages, 2);
    testCase.verifyFalse(det.OnGpu);
    rgb = uint8(repmat(testCase.TestData.gray, 1, 1, 3));
    [prob, info] = det.Detect(rgb);
    testCase.verifySize(prob, [720 1280 2]);
    testCase.verifyClass(prob, 'single');
    testCase.verifyTrue(all(prob(:) >= 0 & prob(:) <= 1));
    testCase.verifyGreaterThan(max(prob(:, :, 1), [], 'all'), V.Search.ProbThreshold);
    testCase.verifyGreaterThan(max(prob(:, :, 2), [], 'all'), V.Search.ProbThreshold);
    % page 1 carries the left line only, page 2 the right one
    testCase.verifyEqual(max(prob(:, 700:end, 1), [], 'all'), single(0));
    testCase.verifyEqual(max(prob(:, 1:580, 2), [], 'all'), single(0));
    testCase.verifyGreaterThan(info.Ms, 0);
    testCase.verifyEqual(numel(info.Lanes), 2);
end

function testDetectorResetClearsState(testCase)
    assumeToolbox(testCase);
    V = lka_vision_params('baja'); V.Detector = 'laneassist'; V.UseGpu = false;
    det = lka_detector_create(V);
    rgb = uint8(repmat(testCase.TestData.gray, 1, 1, 3));
    det.Detect(rgb);
    [~, i2] = det.Detect(rgb);
    testCase.verifyGreaterThan(i2.Certainty(1), 0);             % previous frame remembered
    det.Reset();
    [~, i3] = det.Detect(rgb);
    testCase.verifyEqual(i3.Certainty, [0 0]);
    testCase.verifyEqual(i3.MinHeight, V.LaneAssist.MinHeight);
end

function testFullStackSensorMatchesRenderedGroundTruth(testCase)
    assumeToolbox(testCase);
    V = lka_vision_params('baja'); V.Detector = 'laneassist'; V.UseGpu = false;
    cam = lka_camera_model(V.Camera, V.Bev);
    det = lka_detector_create(V);
    [img, ~, gt] = lka_render_lane_scene(cam, lka_default_scene());
    det.Detect(img);                                            % first frame: certainty 0
    [~, info] = det.Detect(img);
    [~, info] = det.Detect(img);                                %#ok<ASGLU> trust_lk needs 3
    [sensor, counts] = lka_laneassist_sensor(info, cam, V);
    testCase.verifyTrue(sensor.FrameValid);
    testCase.verifyTrue(all(counts(3:4) == 1));
    testCase.verifyEqual(sensor.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.15);
    testCase.verifyEqual(sensor.Right.LateralOffset, gt.Right.LateralOffset, 'AbsTol', 0.15);
end

function testTheirDefaultsCannotSpanOurBev(testCase)
    % Their bottom_perc = 0.5 covers the whole useful track of a 1:10 car; on our 1.2 m
    % mount it is 4 to 7.5 m of road, so the support spans ~3.45 m, under
    % V.Fit.MinSpan = 4 m, and no boundary is produced. Finding L11 of
    % docs/laneassist_comparison.md, pinned here so the port is never blamed for it.
    assumeToolbox(testCase);
    V = lka_vision_params('baja'); V.Detector = 'laneassist'; V.UseGpu = false;
    cam = lka_camera_model(V.Camera, V.Bev);
    det = lka_detector_create(V);
    img = lka_render_lane_scene(cam, lka_default_scene());
    prob = det.Detect(img);
    prior = struct('Fun', @(x) 1.75 + 0 * x, 'HalfWidth', V.Search.ColdHalfWidth, ...
        'Cold', true, 'Side', 1);
    pts = lka_search_boundary(lka_bev_warp(cam, prob(:, :, 1)), cam, prior, V.Search);
    span = max(pts(:, 1)) - min(pts(:, 1));
    testCase.verifyLessThan(span, V.Fit.MinSpan);
    sensor = lka_vision_lane_step(lka_bev_warp(cam, prob), cam, V, lka_vision_state_init(V), 0.05);
    testCase.verifyFalse(sensor.FrameValid);
end

function testRetunedPipelineFindsBothLines(testCase)
    % The re-tune their README asks for (BottomPerc to our horizon, more slices) and the
    % same chain our own detectors use.
    assumeToolbox(testCase);
    V = lka_vision_params('baja'); V.Detector = 'laneassist'; V.UseGpu = false;
    V.LaneAssist.BottomPerc = 0.70;
    V.LaneAssist.Slices = 40;
    cam = lka_camera_model(V.Camera, V.Bev);
    det = lka_detector_create(V);
    [img, ~, gt] = lka_render_lane_scene(cam, lka_default_scene());
    prob = det.Detect(img);
    sensor = lka_vision_lane_step(lka_bev_warp(cam, prob), cam, V, lka_vision_state_init(V), 0.05);
    testCase.verifyTrue(sensor.FrameValid);
    testCase.verifyEqual(sensor.Left.LateralOffset, gt.Left.LateralOffset, 'AbsTol', 0.25);
    testCase.verifyEqual(sensor.Right.LateralOffset, gt.Right.LateralOffset, 'AbsTol', 0.25);
end

% ------------------------------------------------------------------------------------ %

function assumeToolbox(testCase)
    testCase.assumeTrue(exist('insertShape', 'file') == 2 && exist('monoCamera', 'file') == 2, ...
        'Computer Vision Toolbox and Automated Driving Toolbox required.');
end

function p = emptyPrev()
    p = struct('LeftCoef', [], 'RightCoef', [], 'PrevTrustLk', true);
end

function [gray, truth] = perspectiveLanes(rightRows)
% Two straight lane lines in perspective, 720 x 1280, asphalt 70, paint 235, painted
% width 6 px at the vanishing row and 30 px at the bottom. rightRows limits the rows the
% right line is painted on. truth(pts) gives the exact centre column of the line each
% point belongs to.
    if nargin < 1
        rightRows = 301:720;
    end
    gray = 70 * ones(720, 1280);
    centre = @(r) 640 + [-1 1] * 430 * (r - 300) / 420;
    for r = 301:720
        half = 6 + 24 * (r - 300) / 420;
        cols = centre(r);
        if ~ismember(r, rightRows)
            cols = cols(1);
        end
        for c = cols
            gray(r, max(1, round(c - half / 2)):min(1280, round(c + half / 2))) = 235;
        end
    end
    truth = @(pts) 640 + sign(pts(:, 1) - 640) .* 430 .* (pts(:, 2) - 300) / 420;
end
