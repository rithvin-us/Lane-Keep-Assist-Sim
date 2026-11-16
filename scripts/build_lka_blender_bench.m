function build_lka_blender_bench()
% BUILD_LKA_BLENDER_BENCH Closed-loop camera bench with Blender as the camera (any OS).
%
% Syntax:
%   build_lka_blender_bench()
%
% Same structure as LKA_Vision_TestBench (build_lka_vision_bench), with the Unreal blocks
% replaced by one MATLAB System block, because MathWorks' Unreal build cannot load USD
% scenes at run time on Linux:
%   Baja plant rear-axle pose -> LkaBlenderCamera (tools/blender/render_server.py renders the
%   lighting track at that pose) -> Vision Lane Detector (LKA.Vision.Detector, mobilenet)
%   -> LKA_Controller (unchanged: tracker, supervisor, pure pursuit / Stanley / MPC, limits)
%   -> steering -> Baja plant.
% Speed and yaw rate reach the controller through the wheel-speed / gyro sensor model.
% Road geometry is used only for the ground-truth metrics. Nothing in the loop is
% scripted: the camera sees wherever the controller has driven the car.
% Output: models/LKA_Blender_TestBench.slx. Run it with run_lka_blender_sim.

    root = fileparts(fileparts(mfilename('fullpath')));
    run(fullfile(root, 'config', 'startup.m'));
    evalin('base', 'lkaProfile = ''baja''; lka_controller_params; create_lka_buses;');
    modelDir = fullfile(root, 'models');
    if ~isfile(fullfile(modelDir, 'LKA_MIL_TestBench.slx'))
        build_lka_models();
    end
    mdl = 'LKA_Blender_TestBench';
    for m = {mdl, 'LKA_MIL_TestBench'}
        if bdIsLoaded(m{1}), close_system(m{1}, 0); end
    end
    load_system(fullfile(modelDir, 'LKA_MIL_TestBench.slx'));
    save_system('LKA_MIL_TestBench', fullfile(modelDir, [mdl '.slx']));   % save as
    set_param(mdl, 'Description', ['Closed-loop camera bench: Baja single-track plant, Blender-rendered ' ...
        'lighting track (render_server.py), Mobilenet lane detector, wheel-speed and gyro models, ' ...
        'LKA_Controller, ground-truth metrics.']);

    % ---------------- remove the emulated lane sensor
    removeBlock([mdl '/Virtual Lane Sensor']);
    terminate(mdl, 'Lane Geometry (ground truth)', 1:3);
    terminate(mdl, 'Scenario', 2:3);

    % ---------------- Blender camera at the plant pose
    add_block('simulink/Discrete/Zero-Order Hold', [mdl '/Camera Rate'], 'SampleTime', 'LKA.Ts', ...
        'Position', [1250 700 1290 730]);
    cam = [mdl '/Blender Camera'];
    add_block('simulink/User-Defined Functions/MATLAB System', cam, 'System', 'LkaBlenderCamera', ...
        'Position', [1540 840 1700 920]);
    set_param(cam, 'Vision', 'LKA.Vision', 'Ts', 'LKA.Ts');
    lka_add_vision_detector(mdl, 'Vision Lane Detector', [1780 840 1960 960]);
    ln(mdl, 'Vehicle Dynamics (Baja single-track)/2', 'Camera Rate/1');
    ln(mdl, 'Camera Rate/1', 'Blender Camera/1');
    ln(mdl, 'Blender Camera/1', 'Vision Lane Detector/1');
    ln(mdl, 'Vision Lane Detector/1', 'LKA Controller/1');

    % ---------------- wheel-speed and gyro models in front of the controller
    delete_line(mdl, 'Speed To Controller Rate/1', 'LKA Controller/2');
    delete_line(mdl, 'Yaw Rate To Controller Rate/1', 'LKA Controller/3');
    es = 'Ego Sensors (wheel speed + IMU)';
    lka_add_ego_sensors(mdl, es, [1640 340 1740 440]);
    ln(mdl, 'Speed To Controller Rate/1', [es '/1']);
    ln(mdl, 'Yaw Rate To Controller Rate/1', [es '/2']);
    ln(mdl, [es '/1'], 'LKA Controller/2');
    ln(mdl, [es '/2'], 'LKA Controller/3');

    % ---------------- annotated video + live dashboard
    rec = [mdl '/Vision Recorder'];
    add_block('simulink/User-Defined Functions/MATLAB System', rec, 'System', 'LkaVisionRecorder', ...
        'Position', [2060 840 2220 960]);
    set_param(rec, 'Vision', 'LKA.Vision', 'Ts', 'LKA.Ts');      % path: LKA.Vision.Bench.VideoFile
    add_block('simulink/Math Operations/Gain', [mdl '/Steer Deg'], 'Gain', '180/pi', 'Position', [1990 1000 2030 1030]);
    add_block('simulink/Signal Routing/Bus Selector', [mdl '/Select State'], 'OutputSignals', 'State', ...
        'Position', [1990 1060 2000 1090]);
    ln(mdl, 'Blender Camera/1', 'Vision Recorder/1');
    ln(mdl, 'Vision Lane Detector/2', 'Vision Recorder/2');
    ln(mdl, 'Vision Lane Detector/1', 'Vision Recorder/3');
    ln(mdl, 'LKA Controller/1', 'Steer Deg/1');
    ln(mdl, 'Steer Deg/1', 'Vision Recorder/4');
    ln(mdl, 'LKA Controller/3', 'Select State/1');
    ln(mdl, 'Select State/1', 'Vision Recorder/5');
    ln(mdl, [es '/1'], 'Vision Recorder/6');                      % measured speed [m/s]
    ln(mdl, 'LKA Controller/2', 'Vision Recorder/7');             % turning radius [m]
    terminate(mdl, 'Vision Lane Detector', 3);

    % ---------------- logging
    logOut(mdl, 'Vision Lane Detector', 1, 'lane_sensor');
    logOut(mdl, 'Vision Lane Detector', 3, 'vision_info');
    logOut(mdl, es, 1, 'speed_meas');
    logOut(mdl, es, 2, 'yaw_rate_meas');

    save_system(mdl);
    fprintf('[LKA blender] Built %s\n', fullfile(modelDir, [mdl '.slx']));
end

function removeBlock(blk)
    lh = get_param(blk, 'LineHandles');
    h = [lh.Inport, lh.Outport];
    for k = find(h > 0)
        delete_line(h(k));
    end
    delete_block(blk);
end

function terminate(mdl, blk, ports)
    for p = ports
        t = add_block('simulink/Sinks/Terminator', sprintf('%s/%s Term %d', mdl, blk, p), 'MakeNameUnique', 'on');
        add_line(mdl, sprintf('%s/%d', blk, p), [get_param(t, 'Name') '/1'], 'autorouting', 'smart');
    end
end

function ln(sys, src, dst)
    add_line(sys, src, dst, 'autorouting', 'smart');
end

function logOut(mdl, blk, port, name)
    ph = get_param([mdl '/' blk], 'PortHandles');
    lh = get_param(ph.Outport(port), 'Line');
    if lh > 0
        set_param(lh, 'Name', name);
    end
    set_param(ph.Outport(port), 'DataLogging', 'on', 'DataLoggingNameMode', 'SignalName');
end
