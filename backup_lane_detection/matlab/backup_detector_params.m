function V = backup_detector_params(V)
% BACKUP_DETECTOR_PARAMS Parameters of the archived classic / LaneAssist detectors and of
% the lane-marking reports (lka_dashed_detect, lka_horizontal_detect).
%
% Cut out of src/vision/lka_vision_params.m when the Mobilenet Approach became the only
% active detector (2026-11-16). To restore one of those detectors see
% backup_lane_detection/README.md: V = backup_detector_params(LKA.Vision).

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
end
