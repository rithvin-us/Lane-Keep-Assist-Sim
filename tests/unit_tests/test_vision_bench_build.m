% ==============================================================================
% test_vision_bench_build.m
% Pose mapping to Unreal and structure/compilation of LKA_Vision_TestBench.
% ==============================================================================

function tests = test_vision_bench_build
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(testCase.TestData.root, 'config', 'startup.m'));
end

function testPoseMapSkeletonConvention(testCase)
    [X, Y, yaw] = lka_pose_to_sim3d([10 5 deg2rad(30)], 3);
    testCase.verifyEqual([X Y yaw], [10 -5 -30], 'AbsTol', 1e-12);
end

function testPoseMapAllCandidatesDiffer(testCase)
    p = [10 5 deg2rad(30)];
    out = zeros(4, 3);
    for k = 1:4
        [out(k, 1), out(k, 2), out(k, 3)] = lka_pose_to_sim3d(p, k);
    end
    testCase.verifyEqual(size(unique(out, 'rows'), 1), 4);
end

function testBenchBuildsAndCompiles(testCase)
    build_lka_vision_bench();
    mdl = 'LKA_Vision_TestBench';
    testCase.addTeardown(@() close_system(mdl, 0));
    for b = {'LKA Camera', 'Vision Lane Detector', 'Ego Vehicle (Unreal)', 'Pose To Unreal', ...
             'Simulation 3D Scene Configuration', 'Vision Recorder', 'Ego Sensors (wheel speed + IMU)', ...
             'Frame Logger (dev only)', 'Ground Truth Boundaries (dev only)'}
        testCase.verifyGreaterThan(getSimulinkBlockHandle([mdl '/' b{1}]), 0, b{1});
    end
    testCase.verifyEqual(getSimulinkBlockHandle([mdl '/Virtual Lane Sensor']), -1);
    LKA = evalin('base', 'LKA'); LKA.Vision.Detector = 'mobilenet'; assignin('base', 'LKA', LKA);
    vehicle = evalin('base', 'vehicle');
    sc = generate_lka_scenarios(vehicle); sc = sc(1); r = sc.Road;
    assignin('base', 'SCN', sc.P);
    assignin('base', 'ROAD', struct('s', r.s, 'x', r.x, 'y', r.y, 'psi', r.psi, 'kappa', r.kappa, ...
        'dkappa', r.dkappa, 'width', r.width, 'lineWidth', r.lineWidth));
    set_param(mdl, 'SimulationCommand', 'update');          % throws on any error
end
