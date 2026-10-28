function LKA = lka_build_params(profile, vehicle, sensor)
% LKA_BUILD_PARAMS Builds the complete calibration structure of the LKA system.
%
% Syntax:
%   LKA = lka_build_params(profile, vehicle, sensor)
%
% Inputs:
%   profile - 'baja'     : controller tuned for the aBaja ATV (vehicle_params.m),
%                          used by the standalone MIL test bench.
%             'skeleton' : controller adapted to the plant of the competition
%                          skeleton aBaja2026.slx (Simulation 3D Physics Vehicle,
%                          3.0 m wheelbase), used for RoadRunner co-simulation.
%   vehicle - struct from vehicle_params.m (steering limits are always taken from it)
%   sensor  - struct from sensor_params.m
%
% Conventions (whole LKA): ISO 8855 vehicle frame, x forward, y left, yaw
% counter-clockwise positive. Lane geometry is referenced to the rear-axle
% centre. Positive curvature / steering / turning radius = turning LEFT.
%
% References:
%   Coulter (1992) CMU-RI-TR-92-01 - pure pursuit.
%   Snider (2009) CMU-RI-TR-09-08 - Stanley / pure pursuit comparison.
%   UNECE R79 Rev.4 (ACSF category B1) - 3 m/s^2 lateral acceleration and
%   5 m/s^3 lateral jerk caps used for the curvature limiter.
%   MathWorks "Lane Keeping Assist System" block (MPC Toolbox).

    arguments
        profile (1,:) char {mustBeMember(profile, {'baja', 'skeleton'})}
        vehicle (1,1) struct
        sensor (1,1) struct
    end

    LKA = struct();
    LKA.Profile = profile;
    LKA.Ts = 0.05;                       % Controller sample time [s] (matches 20 Hz detector and skeleton timeStep)

    % ------------------------------------------------------------------ Vehicle
    switch profile
        case 'baja'
            Veh.L      = vehicle.wheelbase;
            Veh.a      = vehicle.a;
            Veh.b      = vehicle.b;
            Veh.m      = vehicle.mass;
            Veh.Iz     = vehicle.Iz;
            Veh.Cf     = vehicle.Cf;
            Veh.Cr     = vehicle.Cr;
            Veh.Track  = vehicle.trackWidth;
        case 'skeleton'
            % Simulation 3D Physics Vehicle inside aBaja2026.slx/Subsystem2/Vehicle Dynamics
            % (WheelBase 3, TrackWidth 2, Mass 1500, Cy 70e3 per tyre). CG assumed mid-wheelbase.
            Veh.L      = 3.0;
            Veh.a      = 1.5;
            Veh.b      = 1.5;
            Veh.m      = 1500;
            Veh.Iz     = 2875;
            Veh.Cf     = 2 * 70e3;
            Veh.Cr     = 2 * 70e3;
            Veh.Track  = 2.0;
    end
    Veh.TireWidth   = vehicle.tireWidth;
    % Understeer gradient K_us [rad/(m/s^2)] with axle cornering stiffnesses:
    % delta = L*kappa + K_us*a_y  (Gillespie 1992, eq. 6-22)
    Veh.Kus         = Veh.m / Veh.L * (Veh.b / Veh.Cf - Veh.a / Veh.Cr);
    Veh.SteerMax    = vehicle.maxSteerAngle;
    Veh.SteerRateMax = vehicle.maxSteerRate;
    Veh.SteerLagTau = vehicle.steerLagTau;
    LKA.Veh = Veh;

    % --------------------------------------------------------------------- Lane
    Lane.WidthNominal = 3.5;             % Lane width of the aBaja LKA track (LKA_IPG.xodr) [m]
    Lane.WidthMin     = 2.5;             % Plausible lane width band [m]
    Lane.WidthMax     = 5.0;
    Lane.LineWidth    = 0.125;           % Painted line width [m]
    Lane.WidthTau     = 2.0;             % Lane-width estimator time constant [s]
    LKA.Lane = Lane;

    % --------------------------------------------- Boundary validation (gating)
    Val.StrengthMin   = 0.30;            % Minimum detector strength to use a boundary [-]
    Val.SideMargin    = 0.50;            % Left boundary must satisfy offset > -SideMargin (mirror for right) [m]
    Val.OffsetMax     = 5.0;             % Max |lateral offset| of a boundary [m]
    Val.HeadingMax    = deg2rad(35);     % Max |relative heading| [rad]
    Val.CurvatureMax  = 0.1;             % Max |curvature| (R = 10 m) [1/m]
    Val.JumpMax       = 0.6;             % Max frame-to-frame offset jump of an accepted boundary [m]
    Val.CheckX        = 0;               % Distance ahead of the rear axle where side, heading,
                                         % curvature and width are checked [m] (tight tracks: 4)
    LKA.Val = Val;

    % ------------------------------------- Lane state tracker (Kalman filter)
    % State x = [y; psi; kappa; dkappa] of the lane centre at the rear axle.
    KF.SigmaMeas   = [sensor.lateralNoiseStd; sensor.headingNoiseStd; ...
                      sensor.curvatureNoiseStd; sensor.curvDerivNoiseStd];
    KF.Qdiag       = [0.02^2; deg2rad(0.5)^2; (1.0e-3)^2; (1.0e-4)^2];  % process noise per second
    % (initialised on the first valid measurement with P = R, the posterior of a flat prior)
    KF.Gate        = 13.82;              % Chi-square 99.9 % gate on [offset; heading], 2 DOF
    KF.MaxRejects  = 4;                  % Consecutive gate rejections before re-initialisation
    KF.SigmaYRef   = 0.40;               % Lateral std that maps to quality 0 [m]
    % Detector frames are LatencySteps old: the published estimate is predicted forward
    % by this time with the measured speed and yaw rate (ego-motion compensation).
    KF.LatencyComp = round(sensor.latencyMs / 1000 / sensor.sampleTime) * sensor.sampleTime;
    LKA.KF = KF;

    % ---------------------------------------------------------- Supervisor FSM
    Sup.SpeedOn      = 0.5;              % Activate above [m/s] (2025 min_control_speed)
    Sup.SpeedOff     = 0.2;              % Release below [m/s]
    Sup.SpeedMax     = 12.0;             % Release above [m/s] (43 km/h, beyond competition range)
    Sup.QualityOn    = 0.60;             % Lane quality to (re)activate [-]
    Sup.QualityOff   = 0.30;             % Lane quality below which the LKA degrades [-]
    Sup.CoastDegrade = 0.25;             % Time without lane measurements that degrades the LKA [s] (5 frames)
    Sup.CoastMax     = 1.0;              % Max time without lane measurements before take-over request [s]
    Sup.TorHold      = 2.0;              % Take-over request hold time [s]
    % Road curvature the LKA can follow, conservative over the speed range (understeer at SpeedMax)
    Sup.CurvatureCap = 0.95 * tan(Veh.SteerMax) / (Veh.L + Veh.Kus * Sup.SpeedMax^2);   % [1/m]
    LKA.Sup = Sup;

    % --------------------------------------------- Lane departure warning (LDW)
    Ldw.DlcWarn   = 0.15;                % Distance-to-line-crossing warning threshold [m]
    Ldw.TlcWarn   = sensor.tlcWarningThreshold;  % Time-to-line-crossing warning threshold [s]
    Ldw.MinSpeed  = 1.0;                 % LDW active above [m/s]
    Ldw.TlcMax    = 99;                  % Reported TLC when not approaching a line [s]
    LKA.Ldw = Ldw;

    % ----------------------------------------- Controller 1: pure pursuit (main)
    switch profile
        case 'baja'
            % Below 30 km/h the look-ahead no longer shrinks to 2.5 m: with a 1 m lane-centre
            % error a 2.5-3 m look-ahead asks for ~35 deg of heading change while the car
            % pulls away (S5: R 4 m command at 2 km/h, then a growing swing with the camera).
            PP.LdSpeed = [0    2    4    6    8.33  10   14 ];   % [m/s]
            PP.LdBase  = [4.0  4.3  4.8  5.3  6.0   6.8  8.5];  % [m]
        case 'skeleton'
            PP.LdSpeed = [0    2    4    6    8.33  10   14 ];
            PP.LdBase  = [4.0  4.5  5.3  6.3  7.5   8.3  10.0];
    end
    PP.LdMin       = 3.5;                % [m] (after the curve shortening)
    PP.LdMax       = 15.0;               % [m]
    PP.LdCurvGain  = 1.5;                % Look-ahead shortening in curves: Ld/(1+g*|kappa|*Ld) [-]
    PP.Ki          = 2.0e-3;             % Integral gain on lane-centre offset [1/(m^2 s)]
    PP.IntLimit    = 4.0e-3;             % Integral curvature authority [1/m]
    PP.IntMinSpeed = 2.0;                % Integrate only above [m/s]
    LKA.PP = PP;

    % --------------------------- Controller 2: 2025 Stanley + PID (baseline)
    % Port of aBaja 2025 ros2_ws/src/lka/scripts/lka.py with the frame bug fixed:
    % all errors are taken from the vehicle-frame lane estimate.
    St.K            = 0.3;               % stanley_k
    St.KSoft        = 1.0;               % stanley_softening [m/s]
    St.SpeedNorm    = 5.0;               % speed_factor = min(2, v/5)
    St.LdBase       = 6.0;               % lookahead_base_m
    St.LdSpeedGain  = 0.4;               % lookahead_speed_factor [s]
    St.LdMin        = 3.0;
    St.LdMax        = 15.0;
    St.Deadzone     = 0.1;               % cte_deadzone_m
    St.Kp           = 0.015;
    St.Ki           = 0.001;
    St.Kd           = 0.005;
    St.IntLimit     = 0.15;
    St.PidLimit     = deg2rad(3.0);
    St.DerivAlpha   = 0.7;
    St.SteerLimit   = deg2rad(12.0);     % max_lka_steering_wheel_angle
    St.RateLimit    = deg2rad(25.0);     % steering_rate_limit_rad_s
    St.SmoothAlpha  = 0.7;               % steering_smoothing_alpha
    St.DecayInactive = 0.9;
    LKA.St = St;

    % ----------------- Controller 3: MPC on lane errors (curvature input, preview)
    % See lka_mpc_curvature.m. The MPC Toolbox "Lane Keeping Assist System" block
    % was evaluated first and rejected: tuned for a 1.6 t sedan, no steering-rate
    % constraint, unstable against the R79 jerk limit on the 240 kg Baja.
    Mpc.Horizon        = 30;             % 1.5 s prediction and curvature preview
    % Weights chosen for robustness: stable with up to 4 samples (200 ms) of unmodelled
    % loop delay in a kinematic closed-loop sweep (detector latency, frame hold, output filter).
    Mpc.Qy             = 100;            % lateral deviation weight [1/m^2]
    Mpc.Qpsi           = 1000;           % heading error weight [1/rad^2]
    Mpc.Ru             = 1000;           % weight on (kappa_cmd - kappa_road) [m^2]
    Mpc.Rdu            = 5.0e5;          % weight on curvature-command increments [m^2]
    Mpc.TerminalFactor = 10;             % terminal state weight multiplier
    Mpc.InitialSpeed   = 1.0;            % speed floor of the prediction model [m/s]
    Mpc.VMax           = 15.0;           % speed cap of the predicted speed profile [m/s]
    Mpc.AccelMax       = 6.0;            % clip of the measured acceleration fed to the MPC [m/s^2]
    LKA.Mpc = Mpc;

    % ---------------------------------------- Common curvature limiter
    Lim.AyMax     = 3.0;                 % Max commanded lateral acceleration [m/s^2] (UNECE R79 B1)
    Lim.JerkMax   = 5.0;                 % Max commanded lateral jerk [m/s^3] (UNECE R79 B1)
    Lim.SpeedFloor = 2.0;                % Speed used below this for the a_y / jerk limits [m/s]
    LKA.Lim = Lim;

    % ---------------------------------------- Steering actuator manager
    Act.RampTime   = 0.5;                % Authority fade-in / fade-out time [s]
    Act.LpfCutoff  = 5.0;                % Output low-pass cut-off [Hz]
    Act.SatTol     = deg2rad(0.2);       % Within this of the limit counts as saturated [rad]
    LKA.Act = Act;

    % ---------------------------------------- Outputs
    Out.RadiusMax = 1000;                % |turning radius| clip for straight driving [m]
    LKA.Out = Out;

    % ---------------------------------------- Controller selection
    % 1 = Pure Pursuit (curvature FF + FB), 2 = Stanley 2025 baseline, 3 = MPC
    LKA.ControllerSelect = 1;
    LKA.Enable = true;

    % ---------------------------------------- Emulated lane sensor
    Sen.Ts           = sensor.sampleTime;
    Sen.LatencySteps = max(0, round(sensor.latencyMs / 1000 / sensor.sampleTime));
    Sen.Sigma        = [sensor.lateralNoiseStd; sensor.headingNoiseStd; ...
                        sensor.curvatureNoiseStd; sensor.curvDerivNoiseStd];
    Sen.Strength     = sensor.strengthNominal;
    Sen.DropoutProb  = sensor.dropoutProb;
    Sen.Seed         = sensor.noiseSeed;
    Sen.ViewRange    = sensor.maxRange;
    LKA.Sen = Sen;

    % ---------------------------------------- Co-simulation adapter (skeleton only)
    Cosim.PoseToRearAxle = -Veh.b;       % Longitudinal offset from skeleton CurrPose point to rear axle [m]
    Cosim.YawOffsetDeg   = 0;            % Added to skeleton CurrPose yaw before use [deg]
    Cosim.YawRateSign    = -1;           % CurrYawRate [deg/s] -> + CCW (CurrPose yaw = -90 - psi3D); checked in Phase 5
    Cosim.LkaModes       = [1 2];        % Skeleton Mode values in which the LKA steers (LKA, Endurance)
    LKA.Cosim = Cosim;

    % ---------------------------------------- Camera lane detector (lka_vision_params.m)
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    LKA.Vision = lka_vision_params(profile);
    LKA.Vision.DetectorDir = fullfile(root, 'models', 'perception');
    LKA.Vision.Bench.ScenePath = fullfile(root, 'data', 'scenes', 'LKA', 'LKA.usd');
    LKA.Vision.Ts = LKA.Ts;

    % ---------------------------------------- Ego-motion sensors (rule I.2.3: realistic sensors)
    % Assumed automotive-grade values: wheel-speed scale error from tyre radius, MEMS gyro
    % residual bias after start-up calibration and white noise per 20 Hz sample.
    EgoSen.SpeedScaleErr   = 0.01;           % [-]
    EgoSen.SpeedNoiseStd   = 0.05;           % [m/s]
    EgoSen.SpeedQuant      = 0.01;           % [m/s]
    EgoSen.YawRateBias     = deg2rad(0.2);   % [rad/s]
    EgoSen.YawRateNoiseStd = deg2rad(0.3);   % [rad/s]
    EgoSen.Seed            = [2101 2102];
    LKA.EgoSen = EgoSen;
end
