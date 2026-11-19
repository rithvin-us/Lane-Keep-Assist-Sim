% ==============================================================================
% test_vision_benchmark.m
% Segmentation metrics (incl. the pixel-accuracy trap) and the detector benchmark
% on a synthetic dataset.
% ==============================================================================

function tests = test_vision_benchmark
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    V = LKA.Vision; V.UseGpu = false;
    testCase.TestData.V = V;
end

function testPixelAccuracyTrap(testCase)
    gt = false(720, 1280); gt(:, 300:330) = true; gt(:, 950:980) = true;    % ~5 % lane pixels
    m = lka_seg_metrics(zeros(720, 1280, 'single'), gt, 0.5, 3);
    testCase.verifyGreaterThan(m.PixelAccuracy, 0.94);     % "accurate" while seeing nothing
    testCase.verifyEqual(m.F1, 0);
    testCase.verifyEqual(m.Recall, 0);
end

function testPerfectPrediction(testCase)
    gt = false(100, 100); gt(:, 40:42) = true;
    m = lka_seg_metrics(single(gt), gt, 0.5, 3);
    testCase.verifyEqual([m.Precision m.Recall m.F1 m.IoU], [1 1 1 1]);
end

function testNoGroundTruthMaskGivesNaN(testCase)
    m = lka_seg_metrics(zeros(50, 50, 'single'), false(50, 50), 0.5, 3);
    testCase.verifyTrue(isnan(m.F1) && isnan(m.IoU));
end

function testBenchmarkOnSyntheticDataset(testCase)
    folder = tempname;
    make_synthetic_vision_dataset(folder, 8, testCase.TestData.V);
    T = lka_detector_benchmark(folder, {'mobilenet'}, testCase.TestData.V);
    testCase.verifyEqual(height(T), 1);
    % Regression floor, not a quality target: the Mobilenet Approach was trained on 31 real
    % college frames and has never seen a rendered lane. Measured 2026-11-16 on these
    % renders: F1 0.60-0.63, left detection 0.67-0.75, MaeY10 0.015 m (60 / 8 frames). The
    % archived classic detector scored F1 0.996 here (backup_lane_detection/README.md).
    % Raise the floor when the model is trained on simulator frames.
    testCase.verifyGreaterThan(T.F1, 0.45);
    testCase.verifyGreaterThan(T.DetRateLeft, 0.55);
    testCase.verifyLessThan(T.MaeY10_m, 0.1);
end
