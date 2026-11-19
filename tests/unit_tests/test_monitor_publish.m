% ==============================================================================
% test_monitor_publish.m
% Dashboard telemetry (lka_monitor_publish): road, path trace, values merged from
% another block ('extra'), written to results/monitor/live.json.
% ==============================================================================

function tests = test_monitor_publish
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(testCase.TestData.root, 'config', 'startup.m'));
    testCase.TestData.file = fullfile(testCase.TestData.root, 'results', 'monitor', 'live.json');
end

function testRoadPathAndExtraReachLiveJson(testCase)
    lka_monitor_publish('start', struct('Name', 'unit test', 'Kind', 'sim', 'Frames', 3));
    lka_monitor_publish('road', struct('xL', 0:10, 'yL', ones(1, 11), 'xR', 0:10, 'yR', -ones(1, 11)));
    lka_monitor_publish('pose', [0 0]);
    lka_monitor_publish('pose', [0.05 0]);            % < 0.1 m: not a new path point
    lka_monitor_publish('pose', [1 0.2]);
    lka_monitor_publish('extra', struct('DetectMs', 12.5));
    lka_monitor_publish('sample', struct('Time', 0.1, 'Frame', 1, 'Steer', 0.3));
    lka_monitor_publish('done', struct());              % forces the write
    d = jsondecode(fileread(testCase.TestData.file));
    testCase.verifyEqual(numel(d.Road.xL), 11);
    testCase.verifyEqual(d.Path.x(:)', [0 1]);
    testCase.verifyEqual(d.Path.y(:)', [0 0.2], 'AbsTol', 1e-12);
    testCase.verifyEqual(d.Latest.DetectMs, 12.5);       % merged from 'extra'
    testCase.verifyEqual(d.Latest.Steer, 0.3);
end

function testSampleKeepsItsOwnValueOverExtra(testCase)
    lka_monitor_publish('start', struct('Name', 'unit test', 'Kind', 'sim'));
    lka_monitor_publish('extra', struct('Steer', 9));
    lka_monitor_publish('sample', struct('Time', 0, 'Steer', 1));
    lka_monitor_publish('done', struct());
    d = jsondecode(fileread(testCase.TestData.file));
    testCase.verifyEqual(d.Latest.Steer, 1);
end
