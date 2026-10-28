% ==============================================================================
% lka_controller_params.m
% Calibration and gain parameters for the LKA Steering Controller.
%
% Two structures are created in the base workspace:
%   LKA        - complete calibration of the Simulink LKA system
%                (models/LKA_Controller.slx), built by lka_build_params.m.
%                Profile chosen by base variable lkaProfile ('baja' default,
%                'skeleton' for the aBaja2026.slx RoadRunner co-simulation).
%                Optional base variable lkaLaneWidth [m]: lane width of another track
%                (default 3.5 m, the official LKA track; lka_set_lane_width).
%                Optional base variables lkaTrackProfile ('tight') and lkaMinRadius [m]:
%                tight test lane such as the college snake track (lka_set_track_profile).
%   controller - legacy gains used by scripts/run_baseline_simulation.m and
%                the script-based Stanley / pure pursuit functions.
%
% References:
%   1. Snider, J. M. (2009). Automatic Steering Methods for Autonomous
%      Automobile Path Tracking. CMU-RI-TR-09-08.
%   2. ISO 11270:2014 - LKAS Performance requirements & limits.
% ==============================================================================

if ~exist('lkaProfile', 'var')
    lkaProfile = 'baja';
end
LKA = lka_build_params(lkaProfile, vehicle, sensor);
if exist('lkaLaneWidth', 'var') && ~isempty(lkaLaneWidth)
    LKA = lka_set_lane_width(LKA, lkaLaneWidth);
end
% Optional base variable lkaTrackProfile ('official' default | 'tight' for a painted test
% lane with bends down to lkaMinRadius [m], default 3): lka_set_track_profile.
if exist('lkaTrackProfile', 'var') && ~isempty(lkaTrackProfile)
    if ~exist('lkaMinRadius', 'var') || isempty(lkaMinRadius)
        lkaMinRadius = 3;
    end
    LKA = lka_set_track_profile(LKA, lkaTrackProfile, MinRadius=lkaMinRadius);
end

% Lane source of the competition model (variant control of "Lane Source" in aBaja2026.slx):
%   1 = camera lane detector (submission, rule I.2.3)   2 = emulated sensor (development only)
if ~exist('LKA_LANE_SOURCE', 'var')
    LKA_LANE_SOURCE = 1;
end

controller = struct();

% --- Sample Time ---
controller.Ts = 0.02;           % Controller sample time [s] (50 Hz control loop)

% --- Stanley Steering Control Gains ---
controller.k_stanley = 1.25;    % Cross-track error gain (higher = more aggressive recovery)
controller.k_soft    = 1.00;    % Velocity softening factor [m/s] to avoid div by zero
controller.k_yaw     = 1.00;    % Heading error damping gain

% --- Pure Pursuit Controller Parameters ---
controller.lookaheadDist0 = 2.5; % Minimum lookahead distance at low speed [m]
controller.k_lookahead   = 0.35; % Velocity lookahead gain: Ld = Ld0 + k_v * Vx [s]

% --- Safety & Saturation Limits (per ISO 11270) ---
controller.maxLateralAccel = 3.0; % Maximum allowable lateral acceleration [m/s^2]
controller.maxSteerAngle   = vehicle.maxSteerAngle; % Steering command saturation [rad]
controller.maxSteerRate    = vehicle.maxSteerRate;  % Steering rate limit [rad/s]

% --- Lowpass Filter for Actuator Smoothing ---
controller.filterCutoffHz  = 5.0; % Lowpass cutoff frequency for steering output [Hz]

% Assign to base workspace
assignin('base', 'controller', controller);
assignin('base', 'LKA', LKA);
