function err = lka_lane_shape_error(bevProb, cam, sensor, checkRange)
% LKA_LANE_SHAPE_ERROR How far each fitted lane line is from the painted line it describes.
%
% Syntax:
%   err = lka_lane_shape_error(bevProb, cam, sensor)
%   err = lka_lane_shape_error(bevProb, cam, sensor, checkRange)
%
% For each side found in sensor (LaneSensorBus from lka_vision_lane_step), the painted
% line is traced without a curve model (lka_trace_line), starting at the fitted position
% just beyond cam.XNear. err(k) is the largest |fitted y - traced y| [m] over the first
% checkRange metres (default 8) of the fit, never beyond its ViewRange. A fit that is
% too straight for the lane (or follows the wrong line) gives a large value; on a lane
% the polynomial can follow it stays at a few centimetres. NaN when the side is not
% found or the paint cannot be traced over at least 2 m (also with a 1 m wide search).
%
% Inputs:
%   bevProb - bird's-eye probability, H x W (both sides) or H x W x 2 (page per side)
%   cam     - lka_camera_model
%   sensor  - struct with .Left / .Right LaneBoundaryBus fields
% Output:
%   err     - [left right] [m]

    if nargin < 4
        checkRange = 8;
    end
    err = [NaN NaN];
    sides = {'Left', 'Right'};
    for k = 1:2
        b = sensor.(sides{k});
        if b.Strength <= 0
            continue
        end
        map = double(gather(bevProb(:, :, min(k, size(bevProb, 3)))));
        xEnd = min(b.ViewRange, cam.XNear + checkRange);
        for window = [0.35 1.0]          % a fit that starts > 0.35 m off the paint: look wider
            [xs, ys] = lka_trace_line(map, cam, lineY(b, cam.XNear + 0.3), 0.25, window);
            use = xs <= xEnd;
            if nnz(use) >= 8
                err(k) = max(abs(lineY(b, xs(use)) - ys(use)));
                break
            end
        end
    end
end

function y = lineY(b, x)
    y = b.LateralOffset + tan(b.HeadingAngle) * x + b.Curvature / 2 * x.^2 + b.CurvatureDerivative / 6 * x.^3;
end
