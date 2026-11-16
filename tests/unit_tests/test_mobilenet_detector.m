% ==============================================================================
% test_mobilenet_detector.m
% The active lane detector (perception/Mobilenet_Approach) through the MATLAB ONNX
% adapter: factory, output contract, a real college-track frame, letterbox helper.
% ==============================================================================

function tests = test_mobilenet_detector
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(testCase.TestData.root, 'config', 'startup.m'));
    testCase.assumeTrue(exist('importNetworkFromONNX', 'file') == 2, ...
        'Deep Learning Toolbox Converter for ONNX Model Format not installed');
    LKA = lka_build_params('baja', evalin('base', 'vehicle'), evalin('base', 'sensor'));
    V = LKA.Vision;
    V.UseGpu = false;
    testCase.TestData.V = V;
    testCase.TestData.det = lka_detector_create(V);
    testCase.TestData.rgb = imread(fullfile(testCase.TestData.root, 'tests', 'fixtures', 'college_lane_t20.jpg'));
end

function testDefaultIsMobilenet(testCase)
    testCase.verifyEqual(testCase.TestData.V.Detector, 'mobilenet');
    testCase.verifyTrue(ismember('mobilenet', lka_detector_create('list')));
end

function testUnknownDetectorErrors(testCase)
    V = testCase.TestData.V;
    V.Detector = 'classic';                       % archived in backup_lane_detection/
    testCase.verifyError(@() lka_detector_create(V), 'lka:vision:unknownDetector');
end

function testOutputContract(testCase)
    det = testCase.TestData.det;
    [p, info] = det.Detect(testCase.TestData.rgb);
    testCase.verifyClass(p, 'single');
    testCase.verifySize(p, [720 1280]);
    testCase.verifyGreaterThanOrEqual(min(p(:)), 0);
    testCase.verifyLessThanOrEqual(max(p(:)), 1);
    testCase.verifyGreaterThan(info.Ms, 0);
    testCase.verifyEqual(det.MaxRange, 20);
    probs = det.DetectBatch(cat(4, testCase.TestData.rgb, testCase.TestData.rgb));
    testCase.verifySize(probs, [720 1280 1 2]);
    testCase.verifyEqual(probs(:, :, 1, 1), p, 'AbsTol', 1e-5);
end

function testFindsBothLinesOnCollegeFrame(testCase)
    % frame t = 20 s of college_lane.mp4: the left line enters at the bottom-left corner,
    % the right one at the right edge (see the fixture). Probably a training frame, so
    % this checks the wiring (channel, colour order, normalisation), not generalisation.
    p = testCase.TestData.det.Detect(testCase.TestData.rgb);
    band = p(560:700, :) > 0.5;
    cols = find(any(band, 1));
    testCase.verifyNotEmpty(cols);
    testCase.verifyLessThan(min(cols), 300, 'no left line near the bottom-left');
    testCase.verifyGreaterThan(max(cols), 900, 'no right line near the bottom-right');
    testCase.verifyLessThan(nnz(p > 0.5) / numel(p), 0.10, 'lane class covers too much');
end

function testWrongChannelOrderLosesLines(testCase)
    % guards the sidecar: the network was trained on RGB; feeding BGR must change the answer
    p = testCase.TestData.det.Detect(testCase.TestData.rgb);
    q = testCase.TestData.det.Detect(testCase.TestData.rgb(:, :, [3 2 1]));
    testCase.verifyGreaterThan(mean(abs(p - q), 'all'), 1e-4);
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
