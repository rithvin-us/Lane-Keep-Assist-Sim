function [road, patches] = lka_lighting_track(jsonFile)
% LKA_LIGHTING_TRACK Test track with 30 / 60 deg turns, an S-bend and bright / dark patches.
%
% Syntax:
%   [road, patches] = lka_lighting_track()
%   lka_lighting_track('data/scenes/LightingTrack/track.json')   % also writes the geometry
%                                                               % for tools/blender/build_lighting_track.py
%
% One definition for both sides of the test: road (lka_make_lane_road) is the ground truth
% the bench scores against, and the JSON is what Blender turns into the Unreal scene
% (data/scenes/LightingTrack/LightingTrack.usd). Lane 3.0 m, lines 0.12 m, all radii >= 15 m
% (inside LKA.Vision.Track.CurvatureMax = 0.1 1/m).
%
% Sections (s = station along the lane centre):
%   straight 40 m          bright sun patch across the lane (asphalt nearly as light as paint)
%   left 30 deg, R 25 m
%   straight 25 m          dappled tree shadow (dark blobs on both lines)
%   right 60 deg, R 15 m   shadow band across the whole bend
%   straight 20 m
%   left 60 deg, R 15 m    glare patch (washed-out asphalt) at the bend apex
%   straight 15 m
%   right 30 deg, R 25 m
%   S-bend 45 deg left / 45 deg right, R 20 m   dark on the left line only, then bright on the right
%   straight 45 m          alternating shadow bars (poles / fence), then a long dark section
%
% patches: struct array, one per patch: Type ('dark' | 'bright' | 'glare'), S [s0 s1] [m],
% D [dRight dLeft] lateral extent from the lane centre [m, + left].

    R   = [ inf  25  inf  -15  inf  15  inf  -25  20  -20  inf];
    ang = [   0  30    0   60    0  60    0   30  45   45    0];
    len = [  40   0   25    0   20   0   15    0   0    0   45];
    geom = struct('s0', {}, 'x0', {}, 'y0', {}, 'hdg0', {}, 'length', {}, 'k0', {}, 'k1', {});
    x = 0; y = 0; h = 0; s = 0;
    for k = 1:numel(R)
        if isinf(R(k))
            G = struct('s0', s, 'x0', x, 'y0', y, 'hdg0', h, 'length', len(k), 'k0', 0, 'k1', 0);
        else
            G = struct('s0', s, 'x0', x, 'y0', y, 'hdg0', h, 'length', abs(R(k)) * deg2rad(ang(k)), ...
                'k0', 1 / R(k), 'k1', 1 / R(k));
        end
        geom(end + 1) = G; %#ok<AGROW>
        [x, y, h] = endPose(G);
        s = s + G.length;
    end
    ref = lka_road_from_geometry(geom, 0.25);
    road = lka_make_lane_road(ref, 0, 3.0, 0.12, false, 'Lighting track (30 / 60 deg turns, bright / dark patches)');
    road.name = 'Lighting track';

    sEnd = [geom.s0] + [geom.length];             % end station of every section
    full = [-2.6 2.6];                             % both lines and some shoulder
    p = @(type, s01, d) struct('Type', type, 'S', s01, 'D', d);
    patches = [
        p('bright', [18 26], full)
        p('dark',   [sEnd(2) + 3,  sEnd(2) + 5.5],  [-2.0 -0.6])     % dappled shadow
        p('dark',   [sEnd(2) + 6,  sEnd(2) + 8],    [0.4 2.2])
        p('dark',   [sEnd(2) + 9,  sEnd(2) + 12],   [-2.4 1.0])
        p('dark',   [sEnd(2) + 14, sEnd(2) + 15.5], [-0.8 2.4])
        p('dark',   [sEnd(2) + 17, sEnd(2) + 20],   [-2.4 -0.2])
        p('dark',   [sEnd(3) + 2,  sEnd(4) - 3],    full)            % shadow over the right 60 deg bend
        p('glare',  [sEnd(5) + 4,  sEnd(6) - 4],    full)            % glare at the left 60 deg bend
        p('dark',   [sEnd(8) + 2,  sEnd(9) - 2],    [0.6 2.6])       % S-bend: left line in shadow
        p('bright', [sEnd(9) + 2,  sEnd(10) - 2],   [-2.6 -0.6])     % S-bend: right line in sun
        ];
    for b = 0:4                                     % shadow bars, 1 m dark / 2 m light
        patches(end + 1) = p('dark', sEnd(10) + 4 + 3 * b + [0 1], full); %#ok<AGROW>
    end
    patches(end + 1) = p('dark', [sEnd(10) + 22, sEnd(10) + 34], full);
    patches = patches(:);

    if nargin >= 1 && ~isempty(jsonFile)
        folder = fileparts(jsonFile);
        if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
        out = struct('name', road.name, 'laneWidth', 3.0, 'lineWidth', road.lineWidth, 'shoulder', 1.8, ...
            's', road.s, 'x', road.x, 'y', road.y, 'psi', road.psi, 'width', road.width, ...
            'patches', patches, 'sections', struct('R', num2cell(R), 'deg', num2cell(ang), ...
            's0', num2cell([geom.s0]), 's1', num2cell(sEnd)));
        fid = fopen(jsonFile, 'w');
        fwrite(fid, jsonencode(out, 'PrettyPrint', true));
        fclose(fid);
    end
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
