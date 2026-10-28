function frame = lka_annotate_frame(img, preview, sensor, hud, cam, extra)
% LKA_ANNOTATE_FRAME Camera frame with the lane detection, the LKA command and a HUD.
%
% Syntax:
%   frame = lka_annotate_frame(img, preview, sensor, hud, cam)
%   frame = lka_annotate_frame(img, preview, sensor, hud, cam, extra)
%
% Drawn on the camera image (vehicle-frame curves projected with cam.Sensor, only where
% the ground is visible, cam.XNear .. cam.XFar):
%   - ego lane between the detected left and right lines, filled green (both lines found)
%   - left line green, right line blue (Strength > 0), lane centre dashed yellow
%   - the path the LKA command would drive (arc of curvature hud.CurvatureCmd), magenta
%   - raw detector output when extra.Lanes is given (team CNN lane points [u v ...], one
%     colour per lane slot extra.Ids: 1 orange, 2 green, 3 cyan, 4 pink)
% plus a HUD (LKA state, speed, steering, commanded turning radius, lane offset /
% heading, detector and time per frame), a steering bar and the bird's-eye inset
% (detector probability, parula) with the fitted lines on top.
% hud: SteerDeg [deg, + left], State (0 OFF 1 STANDBY 2 ACTIVE 3 DEGRADED 4 TAKEOVER);
% optional TurnRadius [m], SpeedKmh, CurvatureCmd [1/m], LaneOffset [m], HeadingDeg,
% Detector (char), Ms (ms per frame), Time [s].

    if nargin < 6
        extra = struct();
    end
    frame = img;
    [H, W, ~] = size(img);
    sides = {'Left', 'Right'};
    colors = {[40 220 60], [0 160 255]};
    yOf = @(b, x) b.LateralOffset + tan(b.HeadingAngle) * x + b.Curvature / 2 * x.^2 + b.CurvatureDerivative / 6 * x.^3;
    % Each line is drawn only as far as its fit has paint behind it (ViewRange). Drawn to
    % XFar, a fit of the first metres of a bend looked like a long straight lane.
    found = [sensor.Left.Strength > 0, sensor.Right.Strength > 0];
    xEnd = cam.XFar * [1 1];
    xb = cell(1, 2);
    yb = cell(1, 2);
    for k = 1:2
        b = sensor.(sides{k});
        if found(k)
            if b.ViewRange > cam.XNear
                xEnd(k) = min(cam.XFar, b.ViewRange);
            end
            xb{k} = linspace(cam.XNear, xEnd(k), 40)';
            yb{k} = yOf(b, xb{k});
        end
    end

    % ego lane fill and centre line, up to the shorter of the two fits
    if all(found)
        xs = linspace(cam.XNear, min(xEnd), 40)';
        yL = yOf(sensor.Left, xs);
        yR = yOf(sensor.Right, xs);
        uvL = project(cam, xs, yL, H, W);
        uvR = project(cam, xs, yR, H, W);
        poly = [uvL; flipud(uvR)];
        if size(poly, 1) >= 3 && all(isfinite(poly(:)))
            frame = insertShape(frame, 'FilledPolygon', reshape(poly', 1, []), 'Color', [0 200 80], 'Opacity', 0.22);
        end
        uvC = project(cam, xs, (yL + yR) / 2, H, W);
        for i = 1:2:size(uvC, 1) - 1
            frame = insertShape(frame, 'Line', [uvC(i, :) uvC(i + 1, :)], 'Color', [255 220 0], 'LineWidth', 3);
        end
    end
    % detected lines
    for k = 1:2
        if found(k)
            uv = project(cam, xb{k}, yb{k}, H, W);
            if size(uv, 1) >= 2
                frame = insertShape(frame, 'Line', reshape(uv', 1, []), 'Color', colors{k}, 'LineWidth', 5);
            end
        end
    end
    % raw detector lane points
    if isfield(extra, 'Lanes') && ~isempty(extra.Lanes)
        slotColor = [255 150 0; 60 255 60; 0 230 255; 255 80 200];
        for k = 1:numel(extra.Lanes)
            P = extra.Lanes{k};
            if isempty(P), continue; end
            s = min(max(extra.Ids(k), 1), 4);
            frame = insertShape(frame, 'FilledCircle', [P(:, 1:2), 3 * ones(size(P, 1), 1)], ...
                'Color', slotColor(s, :), 'Opacity', 0.9);
        end
    end
    % commanded path: circle of curvature kappa through the rear axle, y = kappa x^2 / 2
    if isfield(hud, 'CurvatureCmd') && isfinite(hud.CurvatureCmd) && hud.State == 2
        xp = linspace(cam.XNear, min(cam.XFar, cam.XNear + 15), 30)';
        uv = project(cam, xp, hud.CurvatureCmd / 2 * xp.^2, H, W);
        if size(uv, 1) >= 2
            frame = insertShape(frame, 'Line', reshape(uv', 1, []), 'Color', [255 0 255], 'LineWidth', 3);
        end
    end

    % bird's-eye inset with the fitted lines
    pv = im2uint8(ind2rgb(im2uint8(mat2gray(preview, [0 1])), parula(256)));
    sc = 3;
    pv = imresize(pv, sc, 'nearest');
    [ph, pw, ~] = size(pv);
    for k = 1:2
        if found(k)
            r = interp1(cam.RowX, 1:numel(cam.RowX), xb{k}, 'linear', NaN) * ph / numel(cam.RowX);
            c = interp1(cam.ColY, 1:numel(cam.ColY), yb{k}, 'linear', NaN) * pw / numel(cam.ColY);
            ok = isfinite(r) & isfinite(c);
            if nnz(ok) >= 2
                pv = insertShape(pv, 'Line', reshape([c(ok) r(ok)]', 1, []), 'Color', colors{k}, 'LineWidth', 2);
            end
        end
    end
    if ph + 20 <= H && pw + 20 <= W
        frame(11:10 + ph, end - pw - 9:end - 10, :) = pv;
    end

    % HUD
    names = {'OFF', 'STANDBY', 'ACTIVE', 'DEGRADED', 'TAKEOVER'};
    st = min(max(round(hud.State), 0), 4);
    line1 = sprintf('LKA %s', names{st + 1});
    if isfield(hud, 'SpeedKmh'), line1 = sprintf('%s   %.1f km/h', line1, hud.SpeedKmh); end
    if isfield(hud, 'Time'), line1 = sprintf('%s   t %.1f s', line1, hud.Time); end
    line2 = sprintf('steer %+.1f deg', hud.SteerDeg);
    if isfield(hud, 'TurnRadius')
        if abs(hud.TurnRadius) >= 999
            line2 = sprintf('%s   R_cmd straight', line2);
        else
            line2 = sprintf('%s   R_cmd %+.0f m', line2, hud.TurnRadius);
        end
    end
    line3 = sprintf('L %+.2f m   R %+.2f m', sensor.Left.LateralOffset * (sensor.Left.Strength > 0), ...
        sensor.Right.LateralOffset * (sensor.Right.Strength > 0));
    if isfield(hud, 'LaneOffset'), line3 = sprintf('%s   centre %+.2f m', line3, hud.LaneOffset); end
    if isfield(hud, 'HeadingDeg'), line3 = sprintf('%s   heading %+.1f deg', line3, hud.HeadingDeg); end
    txt = {line1, line2, line3};
    if isfield(hud, 'Detector')
        d = hud.Detector;
        if isfield(hud, 'Ms'), d = sprintf('%s   %.0f ms/frame', d, hud.Ms); end
        txt{end + 1} = d;
    end
    fs = max(12, round(H / 34));
    stateColor = {[90 90 90], [200 150 0], [0 150 60], [200 90 0], [200 0 0]};
    for i = 1:numel(txt)
        box = [0 0 0];
        if i == 1, box = stateColor{st + 1}; end
        frame = insertText(frame, [10, 10 + (i - 1) * round(1.6 * fs)], txt{i}, 'FontSize', fs, ...
            'BoxColor', box, 'BoxOpacity', 0.65, 'TextColor', 'white');
    end
    % steering bar (+-25 deg)
    bw = round(W * 0.3); bx = round((W - bw) / 2); by = H - 28;
    frame = insertShape(frame, 'FilledRectangle', [bx by bw 14], 'Color', [0 0 0], 'Opacity', 0.5);
    f = min(max(hud.SteerDeg / 25, -1), 1);
    cx = bx + bw / 2;
    frame = insertShape(frame, 'FilledRectangle', [min(cx, cx - f * bw / 2) by + 2 abs(f) * bw / 2 + 1 10], ...
        'Color', [255 0 255], 'Opacity', 0.9);
    frame = insertShape(frame, 'Line', [cx by - 3 cx by + 17], 'Color', [255 255 255], 'LineWidth', 2);
end

function uv = project(cam, x, y, H, W)
% Vehicle points -> image pixels, keeping only those inside the image.
    uv = vehicleToImage(cam.Sensor, [x(:) y(:)]);
    in = all(isfinite(uv), 2) & uv(:, 1) >= 1 & uv(:, 1) <= W & uv(:, 2) >= 1 & uv(:, 2) <= H;
    uv = uv(in, :);
end
