function V = lka_vision_params(profile)
% LKA_VISION_PARAMS Calibration of the camera lane detector (LKA.Vision).
%
% Syntax:
%   V = lka_vision_params(profile)        profile 'baja' | 'skeleton'
%
% Camera: Simulation 3D Camera of aBaja2026.slx (1280x720, f = 1109 px, no distortion).
% Extrinsics in the LKA vehicle frame (ISO 8855, origin = rear-axle centre on the ground):
%   'baja'     ZED-like mount used by the synthetic tests and LKA_Vision_TestBench.
%   'skeleton' "Hood center" mount of the Muscle car in aBaja2026.slx; initial estimate,
%              replaced by the calibrate_vision_bench measurement and the RoadRunner
%              co-simulation check (Phase 5).
% Pipeline: detector -> probability map -> metric bird's-eye (birdsEyeView) ->
% sliding-window search (Search) -> robust fit (Fit) -> frame-to-frame logic (Track).

    arguments
        profile (1,:) char {mustBeMember(profile, {'baja', 'skeleton'})}
    end

    % ---------------------------------------------------------- camera
    V.Camera.FocalLength    = [1109 1109];   % [px]
    V.Camera.PrincipalPoint = [640 360];     % [px]
    V.Camera.ImageSize      = [720 1280];    % [rows cols]
    switch profile
        case 'baja'
            V.Camera.X        = 1.60;        % ahead of the rear axle [m]
            V.Camera.Y        = 0;
            V.Camera.Height   = 1.20;        % [m]
            V.Camera.PitchDeg = 8;           % nose down [deg]
        case 'skeleton'
            V.Camera.X        = 2.60;
            V.Camera.Y        = 0;
            V.Camera.Height   = 1.10;
            V.Camera.PitchDeg = 0;
    end

    % ---------------------------------------------------------- bird's-eye view
    V.Bev.Resolution = 0.05;     % [m/px]
    V.Bev.Range      = 30;       % look-ahead from the camera [m]
    V.Bev.HalfWidth  = 6;        % lateral half-width [m]
    V.Bev.NearMargin = 0.5;      % start this far beyond the first visible ground point [m]

    % ---------------------------------------------------------- detector
    V.Detector    = 'yolop';     % 'yolop' | 'cnn' | 'classic'
    V.DetectorDir = '';          % set by lka_build_params (models/perception)
    V.UseGpu      = true;
    V.OnnxBackend = 'auto';      % 'auto' | 'matlab' (importer add-on, GPU) | 'onnxruntime' (Python, CPU)
    V.Monitor     = true;        % camera bench publishes live telemetry (lka_monitor_publish)

    % ---------------------------------------------------------- classic detector
    V.Classic.Offset     = 0.12;     % local-contrast offset of the paint logistic
    V.Classic.Scale      = 0.03;     % its width
    V.Classic.BoxSize    = 41;       % local-mean window [px]
    V.Classic.Adaptive   = false;    % per-frame offset (lka_paint_probability): measured
                                     % identical detection and 1.8 ms/frame slower on the
                                     % three clips, so it stays off
                                     % (docs/laneassist_comparison.md)
    V.Classic.Percentile = 99.9;     % of the positive paint-coloured contrast ...
    V.Classic.Factor     = 0.6;      % ... times this is the adapted offset ...
    V.Classic.OffsetMin  = 0.05;     % ... clamped to [OffsetMin, Offset]
    V.Classic.RowRange   = [];       % image rows to compute (empty = all); set from the
                                     % camera by lka_detector_create when RoiFromBev
    V.Classic.RoiFromBev = true;     % only the rows below the horizon: measured identical
                                     % detection and 11-37 % less detector time on the
                                     % three clips (docs/laneassist_comparison.md)
    V.Classic.RoiMargin  = 20;       % rows kept above the horizon [px]

    % ---------------------------------------------------------- LaneAssist detector (ported)
    % config.ini of tsiggi/LaneAssist (camera "455", 848 x 480) rescaled to our 1280 x 720:
    % column counts x1.51, row counts x1.5, second-degree limit x(1.51/1.5^2) = x0.671.
    % Their README says these are the parameters to re-tune per camera and track.
    V.LaneAssist.Slices              = 20;      % row histograms over the bottom BottomPerc
    V.LaneAssist.BottomPerc          = 0.5;
    V.LaneAssist.BottomOffset        = 15;      % first row above the image bottom [px]
    V.LaneAssist.PeaksMinWidth       = 5;       % pulse width at the top row [px]
    V.LaneAssist.PeaksMaxWidth       = 30;      % pulse width at the bottom row [px]
    V.LaneAssist.WidthError          = 15;      % added to the upper width limit [px]
    V.LaneAssist.PixDif              = 5;       % rise / fall measured over this gap [px]
    V.LaneAssist.MinHeight           = 170;     % pulse level [0-255]
    V.LaneAssist.MinHeightDif        = 60;      % rise / fall over PixDif [0-255]
    V.LaneAssist.ResetWidthOnExit    = false;   % true deviates from the original, see
                                                % lka_laneassist_peaks
    V.LaneAssist.MaxAllowedWidthPerc = 0.15;    % clustering slope gate, share of the width
    V.LaneAssist.WidthWeight         = 0.3;     % cost = WidthWeight * slope distance ...
    V.LaneAssist.ExpectedWeight      = 0.7;     % ... + ExpectedWeight * extrapolation error
    V.LaneAssist.MinPeaksForLane     = 3;       % shorter clusters are dropped
    V.LaneAssist.OptimalPeakPerc     = 0.3;     % share of Slices that makes a lane "long"
                                                % (also the degree 1 / 2 switch)
    V.LaneAssist.ExtremeCoef2        = 0.067;   % reject |a| above this (x = a*y^2 + b*y + c)
    V.LaneAssist.ExtremeCoef1        = 3;       % reject |b| above this on a degree 1 fit
    V.LaneAssist.CertaintyFromPeaks  = 0.5;     % certainty: this share from the point count,
                                                % the rest from the change since last frame
    V.LaneAssist.AllowedCertaintyDif = 20.0;    % certainty gap that distrusts the weaker lane
    V.LaneAssist.MinSingleCertainty  = 80;      % trust lane keeping on one lane
    V.LaneAssist.MinDualCertainty    = 60;      % ... or on both
    V.LaneAssist.LineWidthPx         = 6;       % painted line width of the output map [px]
    V.LaneAssist.Adaptive            = true;    % per-frame pulse threshold
    V.LaneAssist.AdaptiveFloor       = 110;     % lowest adapted MinHeight [0-255]
    V.LaneAssist.AdaptiveMargin      = 20;      % below the darkest accepted lane pixel
    V.LaneAssist.AdaptiveResetPoints = 10;      % fewer chosen points resets MinHeight

    % ---------------------------------------------------------- sliding-window search
    V.Search.ProbThreshold = 0.5;
    V.Search.SliceLength   = 1.0;    % [m]
    V.Search.ColdHalfWidth = 1.3;    % window half-width around the nominal lane [m]: acquires
                                     % the lane up to 1.3 m off centre, adjacent lines stay out
    V.Search.WarmHalfWidth = 0.45;   % around the previous boundary (lateral jump gate) [m]
    V.Search.MinSliceMass  = 4;      % summed probability needed in a slice window [px]
    V.Search.MaxGap        = 8;      % stop after this length without a hit [m]
    V.Search.MinCenterDist = 0.4;    % cold search keeps this far from the vehicle axis [m]
    V.Search.LineHalfWidth = 0.25;   % pixels this close to the slice peak feed the fit [m]
    V.Search.SeedRange     = 3.0;    % cold start: column histogram of the first SeedRange m ...
    V.Search.SeedMass      = 30;     % ... the nearest peak with this mass on each side [px]
    V.Search.SeedHalfWidth = 0.6;    % ... is searched +- this [m] (lane width not assumed)
    V.Search.SeedFirst     = false;  % cold order: nominal lane first, nearest line as fallback;
                                     % true while the lane width is unknown (auto-calibration)
    V.Search.SeedFirstWhenUnseeded = false;  % that order only until a pair sets the width
    V.Search.ColdHalfWidthFromWidth = false; % cold window from the tracked width, not 1.3 m
    V.Search.TaperExp      = 0;      % > 0 lets the slice thresholds follow the range.
                                     % 1.0 cost 3.5 points of both-lines ratio on clip
                                     % 12.23.28 and 0.5 cost 8.2, for 0.00003 m of MaeY20
                                     % on the synthetic set: the looser far-range mass
                                     % admits noise, so it stays off
    V.Search.TaperRef      = 10;     % range where the taper is 1 [m]
    V.Search.LineHalfWidthMax = 0.45;% upper limit of the tapered LineHalfWidth [m]

    % ---------------------------------------------------------- robust fit
    V.Fit.MinPoints        = 60;
    V.Fit.MinSpan          = 4;               % [m]
    V.Fit.QuadSpan         = 8;               % quadratic above this span [m]
    V.Fit.CubicSpan        = 15;              % cubic above this span [m]
    V.Fit.Ridge            = [0 0 1e-3 1e-2]; % penalty on the scaled [c0 c1 c2 c3] (numerical
                                              % safety only: larger values trade c2/c3 against
                                              % c0/c1 and bias the extrapolated offset at x = 0)
    V.Fit.HuberDelta       = 0.08;            % [m]
    V.Fit.Iterations       = 3;
    V.Fit.FullStrengthSpan = 12;              % span that gives full strength [m]
    V.Fit.ProbFull         = 0.7;             % mean probability that gives full strength
    V.Fit.ConsistencyWeight = 0.3;            % mixes the change since the previous frame
                                              % into Strength (lka_fit_boundary): both
                                              % lines in 96.5 % instead of 95.7 % of clip
                                              % 12.23.27 (1), unchanged on 12.23.28 and on
                                              % the synthetic set, no time cost. 0 = the
                                              % geometric Strength only
    V.Fit.ConsistencyScale = 0.30;            % change that takes that term to 0 [m rms]

    % ---------------------------------------------------------- frame-to-frame logic
    V.Track.HoldTime     = 0.3;               % previous direct fit is a prior this long [s]
                                              % (no ego-motion compensation: on R127 at
                                              % 30 km/h the prior drifts ~0.1 m per 0.3 s at 20 m)
    V.Track.WidthInit    = 3.5;               % [m]
    V.Track.WidthReseedFrames = 10;           % consecutive plausible pairs rejected by the
                                              % width check before the estimate is re-seeded
    V.Track.WidthTau     = 2.0;               % [s]
    V.Track.WidthTol     = 0.6;               % pair width vs estimate [m]
    V.Track.WidthMin     = 2.5;               % [m]
    V.Track.WidthMax     = 5.0;               % [m]
    V.Track.HeadingMax   = deg2rad(35);       % [rad]
    V.Track.CurvatureMax = 0.1;               % [1/m]
    V.Track.TrustFrames  = 1;                 % consecutive plausible fits before a side is
                                              % reported (1 = every plausible fit). 2 cost
                                              % 0.5 and 3.1 points of both-lines ratio on
                                              % the clips with no measurable gain, so it
                                              % waits for a detector that needs it

    % ---------------------------------------------------------- lane markings (reported only)
    V.Marking.ProbThreshold = 0.5;
    V.Marking.HalfWidth     = 0.25;  % corridor around the fitted boundary [m]
    V.Marking.MinRowMass    = 2;     % columns above the threshold that make a row painted
    V.Marking.DashMin       = 1.5;   % a dash is this long ... [m]
    V.Marking.DashMax       = 5.0;   % ... at most [m]
    V.Marking.GapMin        = 2.0;   % and the gap between two dashes ... [m]
    V.Marking.GapMax        = 10.0;  % ... at most [m]
    V.Marking.MinCycles     = 2;     % dash / gap pairs needed to call a line dashed
    V.Marking.HorCoverage   = 0.6;   % share of the lane width painted in one BEV row
    V.Marking.HorMin        = 0.15;  % length of a horizontal marking along x [m]
    V.Marking.HorMax        = 1.20;  % [m]

    % ---------------------------------------------------------- outputs
    V.PreviewSize  = [135 60];                % bird's-eye preview for the recorder [px]
    V.LatencySteps = 1;                       % modelled processing delay [frames]

    % ---------------------------------------------------------- Unreal test bench
    V.Bench.ScenePath    = '';                % set by lka_build_params (data/scenes/LKA/LKA.usd)
    V.Bench.FrameMap     = 2;                 % identity (ISO 8855 world), calibrate_vision_bench 2026-10-16
    V.Bench.MountZOffset = 0;                 % camera height correction [m] (calibrate_vision_bench)
    V.Bench.PitchSign    = 1;                 % sign of the Simulation 3D Camera pitch mount
    V.Bench.LaneLabelId  = 6;                 % semantic id of lane markings (sim3dSemanticTypes.xml
                                              % RoadLine/Marking); the RoadRunner USD export of the
                                              % LKA track carries no semantic classes (ids 2, 57 only)
    V.Bench.VideoFile    = '';
    V.Bench.RawVideoFile = '';                % unannotated camera frames (run_lka_on_video tests)
    V.Bench.DatasetDir   = '';
    V.Bench.Decimation   = 5;                 % dataset: keep every Nth frame
end
