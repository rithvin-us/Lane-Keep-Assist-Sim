function [sensor, info] = lka_laneassist_sensor(detInfo, cam, V)
% LKA_LANEASSIST_SENSOR LaneAssist image-plane lanes -> LaneSensorBus, their stack only.
%
% Syntax:
%   [sensor, info] = lka_laneassist_sensor(detInfo, cam, V)
%
% The full-stack arm of the comparison in docs/laneassist_comparison.md: the lanes that
% tsiggi/LaneAssist itself would steer on, converted to metres through our camera model
% and nothing else of ours. Their polynomial x = a*y^2 + b*y + c is sampled over the rows
% it covers, each sample is projected to the ground with imageToVehicle, and the metric
% points are fitted with lka_fit_boundary so the output is comparable field for field
% with lka_vision_lane_step. Their own trust flags gate the sides, as their
% lane_keeping does.
%
% The detector plugin arm (lka_detector_laneassist + our BEV, search and fit) measures
% their feature extraction; this arm measures their perception stack. Neither number
% alone answers "whose algorithm is better", so both belong in the comparison.
%
% Inputs:
%   detInfo - info from lka_detector_laneassist's Detect (Coeffs, Selected, Certainty,
%             Trust)
%   cam     - lka_camera_model
%   V       - LKA.Vision
% Outputs:
%   sensor  - LaneSensorBus struct (Left, Right, FrameValid)
%   info    - [nPtsLeft nPtsRight foundLeft foundRight]

    empty = struct('LateralOffset', 0, 'HeadingAngle', 0, 'Curvature', 0, ...
        'CurvatureDerivative', 0, 'Strength', 0, 'ViewRange', 0);
    sensor = struct('Left', empty, 'Right', empty, 'FrameValid', false);
    names = {'Left', 'Right'};
    sides = [1, -1];
    nPts = zeros(1, 2);
    found = false(1, 2);

    for k = 1:2
        coef = detInfo.Coeffs{k};
        if isempty(coef) || ~detInfo.Trust(k)
            continue
        end
        lane = detInfo.Selected.(names{k});
        if isempty(lane)
            continue
        end
        v = (min(lane(:, 2)):max(lane(:, 2)))';
        u = polyval(coef, v);
        xy = imageToVehicle(cam.Sensor, [u, v]);
        keep = isfinite(xy(:, 1)) & xy(:, 1) >= cam.XNear & xy(:, 1) <= cam.XFar;
        w = max(0.1, min(1, detInfo.Certainty(k) / 100));
        pts = [xy(keep, 1), xy(keep, 2), repmat(w, sum(keep), 1)];
        nPts(k) = size(pts, 1);

        [b, ok] = lka_fit_boundary(pts, V.Fit);
        % side check at the first visible row, not at the rear axle: on a tight bend the
        % boundary extrapolated back to x = 0 can land on the other side (same rule as
        % lka_vision_lane_step)
        yNear = b.LateralOffset + tan(b.HeadingAngle) * cam.XNear ...
            + b.Curvature / 2 * cam.XNear^2 + b.CurvatureDerivative / 6 * cam.XNear^3;
        found(k) = ok && abs(b.HeadingAngle) <= V.Track.HeadingMax ...
            && abs(b.Curvature) <= V.Track.CurvatureMax && sides(k) * yNear > 0;
        if found(k)
            sensor.(names{k}) = b;
        end
    end

    sensor.FrameValid = any(found) && detInfo.Trust(3);
    info = [nPts, double(found)];
end
