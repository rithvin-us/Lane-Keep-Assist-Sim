function LKA = lka_set_track_profile(LKA, profile, options)
% LKA_SET_TRACK_PROFILE Tunes the camera lane chain and the lane checks to the kind of track.
%
% Syntax:
%   LKA = lka_set_track_profile(LKA, 'official')                 % no change (lka_build_params)
%   LKA = lka_set_track_profile(LKA, 'tight')                    % college snake track
%   LKA = lka_set_track_profile(LKA, 'tight', LaneWidth=2.0, MinRadius=3)
%
% 'official' keeps the values of lka_build_params / lka_vision_params, which are tuned
% for the aBaja LKA track: 3.5 m lanes, arcs of R 127 m and R 257 m (S3 tests R 50 m).
%
% 'tight' is for a painted test lane with bends down to MinRadius (the college track:
% about 2 m wide, S-bends of R 3-8 m). With the official values such a lane comes out
% almost straight, for five reasons that add up (docs/lka_controller.md section 9.1):
%   1. Fit degree from the span (linear < 8 m <= quadratic < 15 m <= cubic): one
%      polynomial over 4-20 m averages two or three bends to nearly zero curvature.
%      -> quadratic from 3 m, cubic from 6 m, and the fitted range is cut back until
%         the fit follows the line centre within 4 cm (Fit.AdaptiveRange, Fit.MaxResidual).
%   2. The search predicts the next window with a straight line, loses a tight bend and
%      stops, so only the straight start of the bend is fitted.
%      -> parabola prediction (Search.PredictQuadratic), widening after a miss.
%   3. Lines bending tighter than R 10 m are rejected, in the camera chain
%      (Vision.Track.CurvatureMax) and again in the controller (Val.CurvatureMax).
%      -> 1 / (0.8 * MinRadius) in both.
%   4. Heading and curvature are checked at the rear axle, 4-6 m behind the first road
%      row the camera sees; on a tight bend that extrapolation exaggerates both.
%      -> checked at the first visible row (Vision.Track.CheckAtNear), heading 50 deg.
%   5. The controller checks side, heading, curvature and lane width at the rear axle
%      too: with fixes 1-4 alone it still dropped one or both lines in 23-63 % of the
%      frames of the three college-track clips.
%      -> checked 4 m ahead (Val.CheckX), offset jump up to 1.0 m (Val.JumpMax).
% With LaneWidth the lane-width band is set too (lka_set_lane_width, 0.5x .. 2x).
%
% Base-workspace shortcut: set lkaTrackProfile = 'tight' (and optionally lkaMinRadius)
% before config/startup.m and the built LKA uses it (lka_controller_params.m).

    arguments
        LKA (1,1) struct
        profile (1,:) char {mustBeMember(profile, {'official', 'tight'})}
        options.LaneWidth (1,1) double = NaN               % [m], NaN: keep the current width
        options.MinRadius (1,1) double {mustBePositive} = 3 % tightest bend of the track [m]
    end

    if isfinite(options.LaneWidth)
        LKA = lka_set_lane_width(LKA, options.LaneWidth, MinFactor=0.5, MaxFactor=2);
    end
    if strcmp(profile, 'official')
        return
    end

    kMax = 1 / (0.8 * options.MinRadius);          % 20 % margin over the tightest bend
    V = LKA.Vision;
    V.Track.CurvatureMax = kMax;
    V.Track.HeadingMax = deg2rad(50);
    V.Track.CheckAtNear = true;
    V.Fit.QuadSpan = 3;
    V.Fit.CubicSpan = 6;
    V.Fit.AdaptiveRange = true;
    V.Fit.MaxResidual = 0.04;
    V.Fit.MinRange = 5;
    V.Fit.FullStrengthSpan = 6;                    % a 6 m fit of a tight bend is a full measurement
    V.Search.PredictQuadratic = true;
    V.Search.PredictCurvatureMax = kMax;
    V.Search.ReacquireGrow = 0.15;
    LKA.Vision = V;

    LKA.Val.CurvatureMax = kMax;
    LKA.Val.HeadingMax = deg2rad(50);
    LKA.Val.CheckX = 4;                % controller checks near the look-ahead, not at the axle
    LKA.Val.JumpMax = 1.0;             % offset memory is at the axle, where tight fits move more
end
