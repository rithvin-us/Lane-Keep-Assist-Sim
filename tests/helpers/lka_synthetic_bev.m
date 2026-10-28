function [bev, cam, lineY] = lka_synthetic_bev(centre, width, xNear, xFar, lineWidth)
% LKA_SYNTHETIC_BEV Bird's-eye lane-line probability map of a lane of any shape.
%
% Syntax:
%   [bev, cam, lineY] = lka_synthetic_bev(centre, width)
%   [bev, cam, lineY] = lka_synthetic_bev(centre, width, xNear, xFar, lineWidth)
%
% Draws the two lines straight into the metric bird's-eye grid (5 cm cells, +-6 m), so
% shapes the camera renderer cannot make (S-bends, snake lanes) can be tested without a
% camera model or toolbox. The lines are y = centre(x) +- width/2 (offset measured
% sideways, like the painted college lane), lineWidth wide (default 0.12 m),
% probability 0.95 on the paint and 0 elsewhere.
%
% Inputs:
%   centre      - @(x) lateral position of the lane centre [m] at x m ahead of the rear axle
%   width       - lane width [m]
%   xNear, xFar - bird's-eye range [m] (default 4.5 and 20, like the phone videos)
% Outputs:
%   bev   - H x W probability map (row 1 = far end, column 1 = left edge)
%   cam   - struct with the lka_camera_model fields the lane step uses: XNear, XFar,
%           RowX (column vector), ColY (row vector)
%   lineY - @(x, side) true y [m] of the left (side +1) or right (side -1) line

    if nargin < 3, xNear = 4.5; end
    if nargin < 4, xFar = 20; end
    if nargin < 5, lineWidth = 0.12; end
    res = 0.05;
    halfWidth = 6;
    nRows = round((xFar - xNear) / res);
    nCols = round(2 * halfWidth / res);
    cam.XNear = xNear;
    cam.XFar = xFar;
    cam.RowX = xFar - res * ((1:nRows)' - 0.5);
    cam.ColY = halfWidth - res * ((1:nCols) - 0.5);
    lineY = @(x, side) centre(x) + side * width / 2;
    [Y, X] = meshgrid(cam.ColY, cam.RowX);
    onLine = abs(Y - lineY(X, 1)) < lineWidth / 2 | abs(Y - lineY(X, -1)) < lineWidth / 2;
    bev = 0.95 * double(onLine);
end
