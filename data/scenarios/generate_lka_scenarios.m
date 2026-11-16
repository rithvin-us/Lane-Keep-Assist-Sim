function scenarios = generate_lka_scenarios(vehicle)
% GENERATE_LKA_SCENARIOS Test scenarios for the LKA MIL test bench.
%
% Syntax:
%   scenarios = generate_lka_scenarios(vehicle)
%
% Each element has fields
%   Name, Description  text
%   Road               ego-lane description (lka_make_lane_road)
%   Scene              Unreal scene (USD) for the camera bench; '' = data/scenes/LKA/LKA.usd
%   P                  numeric parameters read by LKA_MIL_TestBench.slx:
%       V0, VSet, Accel   speed profile v(t) = min(V0 + Accel*t, VSet) [m/s, m/s^2]
%       StopTime          [s]
%       X0                initial plant state [X; Y; psi; vy; r] (CG)
%       NoiseScale        multiplier on the detector noise sigmas
%       LatencySteps      detector latency in frames (overrides LKA.Sen.LatencySteps)
%       Dropouts          N x 3 forced detector outages [tStart tEnd side],
%                         side 1 = left, 2 = right, 3 = both
%
% Scenarios
%   S1 Nominal       official aBaja LKA track (LKA_IPG.xodr), lane 1, 0 -> 30 km/h at
%                    4 m/s^2, 60 s stop (as LKA_IPG.xosc)
%   S2 SpeedLimit    same track at 33 km/h, the rulebook disqualification speed
%   S3 TightTrack    rulebook-style track ("subject to change") with the radii of the
%                    2025 aBaja sample track (R 50 / 60 / 85 m) at 30 km/h
%   S4 Degraded      S1 with 3x noise, 100 ms latency and forced detector outages
%                    (short both-sides loss in a curve, 3 s one-sided loss, and a
%                    1.5 s total loss that exceeds the coast limit -> take-over request)
%   S5 InitialOffset S1 starting 0.8 m left of the centre with 4 deg heading error
%   S6 LightingTrack Blender-built track (lka_lighting_track): 30 / 60 deg turns, S-bend,
%                    sun, glare and shadow patches; 3.0 m lane at 15 km/h. Camera bench
%                    scene data/scenes/LightingTrack/LightingTrack.usd

    arguments
        vehicle (1,1) struct
    end

    here = fileparts(mfilename('fullpath'));
    xodr = fullfile(here, 'tracks', 'LKA_IPG.xodr');
    lkaRoad = read_xodr_lane(xodr, 1, 0.25);
    lkaRoad.name = 'aBaja LKA track (LKA\_IPG.xodr, lane 1)';

    vSet = 30 / 3.6;
    scenarios = struct('Name', {}, 'Description', {}, 'Road', {}, 'P', {}, 'Scene', {});

    scenarios(end+1) = makeScenario('S1_Nominal', ...
        'Official LKA track, 0 to 30 km/h at 4 m/s^2, nominal detector', ...
        lkaRoad, 2.816, 0, 0, vSet, 4, 60, 1, [], [-1 -1 0], vehicle);

    scenarios(end+1) = makeScenario('S2_SpeedLimit', ...
        'Official LKA track at 33 km/h (rulebook DQ threshold)', ...
        lkaRoad, 2.816, 0, 0, 33 / 3.6, 4, 60, 1, [], [-1 -1 0], vehicle);

    tight = tightTrack();
    scenarios(end+1) = makeScenario('S3_TightTrack', ...
        'Rulebook-style S-curves with R 50/60/85 m (2025 aBaja sample track radii), 30 km/h', ...
        tight, 2.0, 0, 0, vSet, 4, ceil(tight.length / vSet) + 3, 1, [], [-1 -1 0], vehicle);

    drop = [14.0 14.6 3;     % both lines lost 0.6 s inside the R 127 m left curve
            21.0 24.0 1;     % left line lost 3 s in the right curve
            40.0 41.5 3];    % both lost 1.5 s on the straight (> coast limit -> TOR)
    scenarios(end+1) = makeScenario('S4_Degraded', ...
        'Official track, 3x detector noise, 100 ms latency, forced line outages', ...
        lkaRoad, 2.816, 0, 0, vSet, 4, 60, 3, 2, drop, vehicle);

    scenarios(end+1) = makeScenario('S5_InitialOffset', ...
        'Official track, start 0.8 m left of centre with 4 deg heading error', ...
        lkaRoad, 2.816, 0.8, deg2rad(4), vSet, 4, 60, 1, [], [-1 -1 0], vehicle);

    lit = lka_lighting_track();
    scenarios(end+1) = makeScenario('S6_LightingTrack', ...
        'Blender track: 30 / 60 deg turns, S-bend, sun / glare / shadow patches, 15 km/h', ...
        lit, 3, 0, 0, 15 / 3.6, 2, 52, 1, [], [-1 -1 0], vehicle);
    scenarios(end).Scene = fullfile(fileparts(here), 'scenes', 'LightingTrack', 'LightingTrack.usd');
end

function sc = makeScenario(name, desc, road, s0, offset0, dpsi0, vSet, accel, stopTime, noiseScale, latency, drop, vehicle)
    xc = interp1(road.s, road.x, s0);
    yc = interp1(road.s, road.y, s0);
    psic = interp1(road.s, road.psi, s0);
    psi0 = psic + dpsi0;
    xr = xc - offset0 * sin(psic);               % rear axle, offset to the left of the centre
    yr = yc + offset0 * cos(psic);
    Xcg = xr + vehicle.b * cos(psi0);
    Ycg = yr + vehicle.b * sin(psi0);

    P = struct();
    P.V0 = 0;
    P.VSet = vSet;
    P.Accel = accel;
    P.StopTime = stopTime;
    P.X0 = [Xcg; Ycg; psi0; 0; 0];
    P.NoiseScale = noiseScale;
    if isempty(latency)
        P.LatencySteps = -1;                     % use LKA.Sen.LatencySteps
    else
        P.LatencySteps = latency;
    end
    P.Dropouts = drop;
    sc = struct('Name', name, 'Description', desc, 'Road', road, 'P', P, 'Scene', '');
end

function road = tightTrack()
    % Straight 30 m, then alternating curves with 10 m straights between them:
    % left R50 over 30 deg, right R60 over 30 deg, left R85 over 20 deg,
    % right R50 over 20 deg, then 80 m straight.
    R   = [ inf  50  inf  -60  inf  85  inf  -50  inf];
    ang = [   0  30    0   30    0  20    0   20    0];
    len = [  30   0   10    0   10   0   10    0   80];
    geom = struct('s0', {}, 'x0', {}, 'y0', {}, 'hdg0', {}, 'length', {}, 'k0', {}, 'k1', {});
    x = 0; y = 0; h = 0; s = 0;
    for k = 1:numel(R)
        if isinf(R(k))
            G = struct('s0', s, 'x0', x, 'y0', y, 'hdg0', h, 'length', len(k), 'k0', 0, 'k1', 0);
        else
            Lk = abs(R(k)) * deg2rad(ang(k));
            G = struct('s0', s, 'x0', x, 'y0', y, 'hdg0', h, 'length', Lk, 'k0', 1/R(k), 'k1', 1/R(k));
        end
        geom(end+1) = G; %#ok<AGROW>
        [x, y, h] = endPose(G);
        s = s + G.length;
    end
    ref = lka_road_from_geometry(geom, 0.25);
    road = lka_make_lane_road(ref, 0, 3.5, 0.125, false, 'Tight rulebook-style track');
    road.name = 'Tight track (R 50/60/85 m)';
end

function [x, y, h] = endPose(G)
    if G.k0 == 0
        x = G.x0 + G.length * cos(G.hdg0);
        y = G.y0 + G.length * sin(G.hdg0);
        h = G.hdg0;
    else
        h = G.hdg0 + G.k0 * G.length;
        x = G.x0 + (sin(h) - sin(G.hdg0)) / G.k0;
        y = G.y0 - (cos(h) - cos(G.hdg0)) / G.k0;
    end
end
