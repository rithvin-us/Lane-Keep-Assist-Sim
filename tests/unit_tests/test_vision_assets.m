% ==============================================================================
% test_vision_assets.m
% Perception assets: detector sidecars, the Mobilenet Approach ONNX, LKA track scene, git hygiene.
% ==============================================================================

function tests = test_vision_assets
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(testCase.TestData.root, 'config', 'startup.m'));
end

function testDetectorSidecarsParse(testCase)
    % every perception/<Approach>/deploy/<name>.json is a complete "seg" sidecar
    files = dir(fullfile(testCase.TestData.root, 'perception', '*', 'deploy', '*.json'));
    testCase.verifyNotEmpty(files);
    need = {'type', 'name', 'onnx', 'input_size', 'channel_order', 'scale', 'mean', 'std', ...
        'resize', 'output', 'lane_channel'};
    for f = files'
        cfg = jsondecode(fileread(fullfile(f.folder, f.name)));
        testCase.verifyTrue(all(isfield(cfg, need)), f.name);
        testCase.verifyEqual(cfg.type, 'seg', f.name);
        testCase.verifyNumElements(cfg.input_size, 2, f.name);
        testCase.verifyNumElements(cfg.mean, 3, f.name);
        testCase.verifyTrue(isfile(fullfile(f.folder, cfg.onnx)), [f.name ': ONNX missing']);
    end
end

function testMobilenetOnnxPresent(testCase)
    f = fullfile(testCase.TestData.root, 'perception', 'Mobilenet_Approach', 'deploy', 'mobilenet_unet_288x480.onnx');
    testCase.verifyTrue(isfile(f));
    info = dir(f);
    testCase.verifyGreaterThan(info.bytes, 20e6);
    testCase.verifyLessThan(info.bytes, 50e6);          % below GitHub's 50 MB warning
end

function testLightingTrackMatchesItsScene(testCase)
    % one MATLAB definition drives both the ground truth and the Blender / Unreal scene
    [road, patches] = lka_lighting_track();
    testCase.verifyEqual(road.length, 234, 'AbsTol', 1);
    testCase.verifyEqual(sort(unique({patches.Type})), {'bright', 'dark', 'glare'});
    testCase.verifyLessThanOrEqual(max(abs(road.kappa)), 1 / 15 + 1e-9);   % tightest bend R 15 m
    d = fullfile(testCase.TestData.root, 'data', 'scenes', 'LightingTrack');
    testCase.verifyTrue(isfile(fullfile(d, 'LightingTrack.usd')));
    testCase.verifyTrue(isfile(fullfile(d, 'textures', 'Asphalt1_Diff.png')));
    J = jsondecode(fileread(fullfile(d, 'track.json')));
    testCase.verifyEqual(J.s(end), road.s(end), 'AbsTol', 1e-6);         % JSON not stale
    sc = generate_lka_scenarios(evalin('base', 'vehicle'));
    testCase.verifyTrue(endsWith(sc(strcmp({sc.Name}, 'S6_LightingTrack')).Scene, 'LightingTrack.usd'));
end

function testVideoWriterFallsBackToFfmpeg(testCase)
    f = [tempname '.mp4'];
    w = LkaVideoWriter(f, 20);
    for k = 1:5
        w.write(uint8(randi(255, 72, 128, 3)));
    end
    w.close();
    testCase.verifyTrue(isfile(f));
    [d, n] = fileparts(f);
    testCase.verifyFalse(isfile(fullfile(d, [n '.avi'])));
    delete(f);
    g = [tempname '.mp4'];
    w = LkaVideoWriter(g, 20);                       % no frame written: no file left behind
    w.close();
    testCase.verifyFalse(isfile(g));
end

function testLkaSceneExported(testCase)
    d = fullfile(testCase.TestData.root, 'data', 'scenes', 'LKA');
    testCase.verifyTrue(isfile(fullfile(d, 'LKA.usd')));
    testCase.verifyTrue(isfile(fullfile(d, 'Roads.mesh.usd')));
end

function testAdas2026IsIgnored(testCase)
    [st, out] = system(sprintf('git -C "%s" check-ignore ADAS_2026/README.md', testCase.TestData.root));
    testCase.verifyEqual(st, 0, out);
end
