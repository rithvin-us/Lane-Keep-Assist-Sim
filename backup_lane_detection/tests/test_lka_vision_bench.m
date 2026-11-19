% ==============================================================================
% test_lka_vision_bench.m
% Closed-loop camera-in-the-loop run on the official LKA track (Unreal).
% Opt-in: set environment variable LKA_RUN_UNREAL_TESTS=1 (opens the Unreal window).
% ==============================================================================

function tests = test_lka_vision_bench
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.assumeEqual(getenv('LKA_RUN_UNREAL_TESTS'), '1', 'Unreal tests are opt-in');
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(projectRoot, 'config', 'startup.m'));
end

function testClassicDetectorKeepsLane(testCase)
    r = run_lka_vision(Scenarios={'S1_Nominal'}, Detectors={'classic'}, Video=false);
    testCase.verifyEqual(r.kpi.InLaneRatio, 1, 'AbsTol', 1e-9);
    testCase.verifyLessThan(r.kpi.MaxLatOffset_m, 0.4);
    testCase.verifyFalse(r.kpi.SpeedDQ);
end

function testYolopDetectorKeepsLane(testCase)
    testCase.assumeEqual(exist('importNetworkFromONNX', 'file'), 2, 'ONNX importer add-on not installed');
    r = run_lka_vision(Scenarios={'S1_Nominal'}, Detectors={'yolop'}, Video=true);
    testCase.verifyEqual(r.kpi.InLaneRatio, 1, 'AbsTol', 1e-9);
    testCase.verifyLessThan(r.kpi.MaxLatOffset_m, 0.4);
end
