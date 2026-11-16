function report = calibrate_vision_bench(options)
% CALIBRATE_VISION_BENCH Verifies the camera-bench assumptions on the official LKA track.
%
% Syntax:
%   report = calibrate_vision_bench()
%   report = calibrate_vision_bench(FrameMaps=[2 3 1 4])
%
% The ego is parked at the S1_Nominal start (rear axle on the ego-lane centre, aligned).
% For each candidate track -> Unreal frame map one short simulation grabs a camera frame
% and its semantic labels; the classic detector + lka_vision_lane_step measure both lane
% lines. The first map that shows both lines with a plausible width and a centred,
% aligned lane wins (a wrong map puts the car off the road or off the terrain, which
% Unreal reports as an error; those maps are skipped). Reported:
%   FrameMap                  -> LKA.Vision.Bench.FrameMap
%   Left_m, Right_m, Width_m  measured line positions at the rear axle
%   CentreOffset_m, Heading_deg  measured lane centre (painted lines) vs the vehicle
%   ImpliedHeight_m           camera height implied by the measured width (3.5 m nominal)
%   SemanticIds               label ids in the frame (the RoadRunner USD export carries no
%                             lane-marking class: semantic ground truth is not available)
% Saves results/vision/calibration.png (annotated) and calibration_raw.png.

    arguments
        options.FrameMaps double = [2 3 1 4]
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    run(fullfile(root, 'config', 'startup.m'));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params; create_lka_buses;');
    vehicle = evalin('base', 'vehicle');
    LKA = evalin('base', 'LKA');
    LKA.Vision.UseGpu = false;
    LKA.Vision.Bench.VideoFile = '';
    LKA.Vision.Bench.DatasetDir = '';
    sc = generate_lka_scenarios(vehicle);
    sc = sc(strcmp({sc.Name}, 'S1_Nominal'));
    P = sc.P; P.VSet = 0; P.V0 = 0; P.StopTime = 0.5;
    r = sc.Road;
    assignin('base', 'SCN', P);
    assignin('base', 'ROAD', struct('s', r.s, 'x', r.x, 'y', r.y, 'psi', r.psi, 'kappa', r.kappa, ...
        'dkappa', r.dkappa, 'width', r.width, 'lineWidth', r.lineWidth));

    mdl = 'LKA_Vision_TestBench';
    load_system(mdl);
    restore = onCleanup(@() close_system(mdl, 0));
    ph = get_param([mdl '/LKA Camera'], 'PortHandles');
    l2 = get_param(ph.Outport(2), 'Line');
    if l2 > 0, delete_line(l2); end
    add_block('simulink/Sinks/To Workspace', [mdl '/Cal Image'], 'VariableName', 'calImg', ...
        'MaxDataPoints', '1', 'SaveFormat', 'Array');
    add_block('simulink/Sinks/To Workspace', [mdl '/Cal Label'], 'VariableName', 'calLabel', ...
        'MaxDataPoints', '1', 'SaveFormat', 'Array');
    add_line(mdl, 'LKA Camera/1', 'Cal Image/1');
    add_line(mdl, 'LKA Camera/2', 'Cal Label/1');

    V = LKA.Vision;
    cam = lka_camera_model(V.Camera, V.Bev);
    det = lka_detector_create(V);
    report = struct('FrameMap', NaN);
    for fm = options.FrameMaps
        LKA.Vision.Bench.FrameMap = fm;
        assignin('base', 'LKA', LKA);
        try
            out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
        catch ME
            fprintf('[calibrate] frame map %d: simulation failed (%s)\n', fm, strtok(ME.message, newline));
            continue
        end
        img = out.calImg(:, :, :, end);
        lab = out.calLabel(:, :, end);
        prob = det.Detect(img);
        s = lka_vision_lane_step(lka_bev_warp(cam, prob), cam, V, lka_vision_state_init(V), LKA.Ts);
        both = s.Left.Strength > 0 && s.Right.Strength > 0;
        w = s.Left.LateralOffset - s.Right.LateralOffset;
        centre = 0.5 * (s.Left.LateralOffset + s.Right.LateralOffset);
        heading = rad2deg(0.5 * (s.Left.HeadingAngle + s.Right.HeadingAngle));
        fprintf('[calibrate] frame map %d: both lines %d, width %.2f m, centre %+.2f m, heading %+.2f deg\n', ...
            fm, both, w, centre, heading);
        if both && abs(w - 3.5) < 0.4 && abs(centre) < 0.4 && abs(heading) < 3
            report = struct('FrameMap', fm, 'Left_m', s.Left.LateralOffset, 'Right_m', s.Right.LateralOffset, ...
                'Width_m', w, 'CentreOffset_m', centre, 'Heading_deg', heading, ...
                'ImpliedHeight_m', V.Camera.Height * 3.5 / w, 'SemanticIds', reshape(unique(lab), 1, []));
            outDir = fullfile(root, 'results', 'vision');
            if ~isfolder(outDir), mkdir(outDir); end
            frame = lka_annotate_frame(img, imresize(max(lka_bev_warp(cam, prob), [], 3), V.PreviewSize), s, ...
                struct('SteerDeg', 0, 'State', 0), cam);
            imwrite(frame, fullfile(outDir, 'calibration.png'));
            imwrite(img, fullfile(outDir, 'calibration_raw.png'));
            break
        end
    end
    if isnan(report.FrameMap)
        error('lka:vision:calibrationFailed', ['No frame map shows the ego lane at the S1 start. ' ...
            'Check that Unreal loads %s.'], V.Bench.ScenePath);
    end
    disp(report);
    fprintf('Set in lka_vision_params.m (Bench): FrameMap = %d\n', report.FrameMap);
end
