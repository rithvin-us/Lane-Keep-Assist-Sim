function report = run_lka_checks(options)
% RUN_LKA_CHECKS One command that checks what a change could break, with a written report.
%
% Syntax:
%   report = run_lka_checks()                     % quick: every unit test (a few minutes)
%   report = run_lka_checks(Level='full')         % + closed-loop MIL tests + every video in vids/
%   matlab -batch "addpath('scripts'); run_lka_checks"     % from a terminal or Task Scheduler
%
% Levels:
%   'quick'  runtests('tests/unit_tests'). Includes test_vision_tight_track: a snake lane
%            fitted almost straight (the college-track problem) fails it. Tests that need
%            the team CNN file, a GPU or Unreal skip themselves when those are missing.
%   'full'   also tests/scenario_tests/test_lka_mil_scenarios.m (rulebook and comfort
%            limits on the official track) and every video in VideoFolder through
%            run_lka_on_test_videos into results/checks/videos (no annotated video). A video
%            whose fitted lines are > 0.25 m off the paint in 10 % of the lines
%            (StraightAlarm in summary.csv) fails the check.
% The report is written to results/checks/lka_checks_<date-time>.md. With FailOnError
% (default true) the function ends with an error when anything failed, so matlab -batch
% returns a non-zero exit code to a scheduler or CI job.
%
% Name-value options:
%   Level         'quick' (default) | 'full'
%   VideoFolder   default vids/
%   TrackProfile  for the videos: 'tight' (default) | 'official'
%   FailOnError   default true

    arguments
        options.Level char {mustBeMember(options.Level, {'quick', 'full'})} = 'quick'
        options.VideoFolder char = ''
        options.TrackProfile char {mustBeMember(options.TrackProfile, {'tight', 'official'})} = 'tight'
        options.FailOnError (1,1) logical = true
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    run(fullfile(root, 'config', 'startup.m'));
    if isempty(options.VideoFolder), options.VideoFolder = fullfile(root, 'vids'); end
    outDir = fullfile(root, 'results', 'checks');
    if ~isfolder(outDir), mkdir(outDir); end
    stamp = char(datetime('now', 'Format', 'yyyy-MM-dd_HHmm'));
    t0 = tic;

    % ---------------- tests
    results = runtests(fullfile(root, 'tests', 'unit_tests'));
    if strcmp(options.Level, 'full')
        results = [results, runtests(fullfile(root, 'tests', 'scenario_tests', 'test_lka_mil_scenarios.m'))];
    end
    testTable = table({results.Name}', [results.Passed]', [results.Failed]', [results.Incomplete]', ...
        [results.Duration]', 'VariableNames', {'Test', 'Passed', 'Failed', 'Skipped', 'Seconds'});

    % ---------------- videos
    videoTable = table();
    if strcmp(options.Level, 'full')
        S = run_lka_on_test_videos(Folder=options.VideoFolder, OutRoot=fullfile(outDir, 'videos'), ...
            Video=false, TrackProfile=options.TrackProfile, Detector='');
        if ~isempty(S)
            videoTable = S(:, {'Video', 'Detector', 'TrackProfile', 'BothLines_pct', 'LkaActive_pct', ...
                'MedianViewRange_m', 'ShapeErrorP90_m', 'StraightAlarm'});
        end
    end

    nFailed = nnz(testTable.Failed);
    nAlarm = 0;
    if ~isempty(videoTable)
        nAlarm = nnz(videoTable.StraightAlarm);
    end
    ok = nFailed == 0 && nAlarm == 0;

    % ---------------- report
    file = fullfile(outDir, sprintf('lka_checks_%s.md', stamp));
    fid = fopen(file, 'w');
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid, '# LKA checks %s (%s)\n\n', stamp, options.Level);
    fprintf(fid, '**Result: %s.** %d tests, %d failed, %d skipped; %d video(s), %d straight-lane alarm(s). %.0f s.\n\n', ...
        ternary(ok, 'PASS', 'FAIL'), height(testTable), nFailed, nnz(testTable.Skipped), height(videoTable), nAlarm, toc(t0));
    fprintf(fid, '## Tests\n\n| Test | Result | s |\n| --- | --- | --- |\n');
    for k = 1:height(testTable)
        state = 'pass';
        if testTable.Failed(k), state = '**FAIL**'; elseif testTable.Skipped(k), state = 'skipped'; end
        fprintf(fid, '| %s | %s | %.1f |\n', testTable.Test{k}, state, testTable.Seconds(k));
    end
    if ~isempty(videoTable)
        fprintf(fid, '\n## Videos (%s profile)\n\n| Video | Both lines %% | Fit range [m] | Shape error p90 [m] | Alarm |\n| --- | --- | --- | --- | --- |\n', ...
            options.TrackProfile);
        for k = 1:height(videoTable)
            fprintf(fid, '| %s | %.0f | %.1f | %.2f | %s |\n', videoTable.Video(k), videoTable.BothLines_pct(k), ...
                videoTable.MedianViewRange_m(k), videoTable.ShapeErrorP90_m(k), ternary(videoTable.StraightAlarm(k), '**yes**', 'no'));
        end
    end
    clear cleaner
    fprintf('[LKA checks] %s: %d/%d tests passed, %d straight-lane alarm(s). Report: %s\n', ...
        ternary(ok, 'PASS', 'FAIL'), nnz(testTable.Passed), height(testTable), nAlarm, file);

    report = struct('Passed', ok, 'Tests', testTable, 'Videos', videoTable, 'File', file);
    if options.FailOnError && ~ok
        error('lka:checks:failed', 'LKA checks failed: %d test(s), %d video alarm(s). See %s', nFailed, nAlarm, file);
    end
end

function v = ternary(c, a, b)
    if c, v = a; else, v = b; end
end
