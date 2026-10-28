function m = lka_bev_odometry(bevPrev, bevCur, cam, options)
% LKA_BEV_ODOMETRY Vehicle motion between camera frames from the bird's-eye road texture.
%
% Syntax:
%   m = lka_bev_odometry(bevPrev, bevCur, cam)
%   m = lka_bev_odometry(bevPrev, bevCur, cam, NearRange=12, Bands=4, Whitening=0.3)
%
% bevPrev / bevCur are grayscale bird's-eye images of consecutive camera frames
% (lka_bev_warp(cam, gray)), or H x W x N stacks of N frame pairs; gpuArray inputs are
% processed on the GPU. The near part of the bird's-eye view (cam.XNear .. cam.XNear +
% NearRange, where the warp is sharp) is cut into Bands depth bands. In each band the road
% texture (high-passed, bright paint clipped so the continuous lane lines do not pull the
% match to zero) is matched by FFT correlation (Hann window, cross-power spectrum divided
% by its magnitude^Whitening: 1 = phase correlation, 0 = plain cross-correlation; 0.3 keeps
% a sharp peak without amplifying the noise of blurred, low-resolution video), parabolic
% sub-pixel peak; all bands of all pairs go through one batched fft2. The near range is
% kept short: a camera-pitch error scales the bird's-eye distances by about
% (1 + 2*pitchError*x/height), so far bands bias the speed.
% For a small heading change dpsi the ground shifts laterally by
%   dy(x) = dy0 + dpsi * x
% so a weighted straight-line fit of the band shifts over their distance x gives the
% heading change (slope) and the lateral displacement at the rear axle (intercept).
% (The 'rigid' log-polar phase correlation and intensity-based refinement were tried
% first: both fail on line-dominated road images.)
%
% Output (pose of the current frame in the previous one, ISO 8855, rear axle), one value
% per frame pair (double, on the CPU); NaN for a pair that could not be matched (too few
% textured bands, field of view too narrow), so a failure is never read as standstill:
%   m.dx, m.dy  displacement [m] (+ forward, + left)
%   m.dpsi      heading change [rad] (+ left / CCW)
%   m.Peak      mean correlation peak of the bands (quality)
%   m.BandX, m.BandDx  band distances [m] (Bands x 1) and forward shift per band [m]
%                      (Bands x N): equal in all bands for a well calibrated camera
% Speed = hypot(dx, dy)/dt, yaw rate = dpsi/dt.

    arguments
        bevPrev {mustBeFloat}
        bevCur {mustBeFloat}
        cam (1,1) struct
        options.NearRange (1,1) double = 12
        options.Bands (1,1) double = 4
        options.Whitening (1,1) double = 0.3
    end
    nPairs = size(bevCur, 3);
    m = struct('dx', nan(1, nPairs), 'dy', nan(1, nPairs), 'dpsi', nan(1, nPairs), ...
        'Peak', zeros(1, nPairs), 'BandX', nan(options.Bands, 1), 'BandDx', nan(options.Bands, nPairs));
    dxr = cam.RowX(2) - cam.RowX(1);          % metres per row (negative: row 1 is far)
    dyc = cam.ColY(2) - cam.ColY(1);          % metres per column (negative: column 1 is left)
    edges = linspace(cam.XNear, cam.XNear + options.NearRange, options.Bands + 1);
    rowsIn = find(cam.RowX >= edges(1) & cam.RowX < edges(end));
    if numel(rowsIn) < 16 * options.Bands
        return
    end
    % texture of the near region (one filter pass), then equal-height bands
    ta = texture(bevPrev(rowsIn, :, :));
    tb = texture(bevCur(rowsIn, :, :));
    nr = floor(numel(rowsIn) / options.Bands);
    K = options.Bands;
    % Columns seen by the camera in every row of a band, minus the texture filter
    % margin: outside the field of view both bird's-eye images are 0, and that static
    % border would pull the correlation peak to zero shift. All bands use the width of
    % the narrowest (nearest) band so they can share one batched fft2.
    seen = reshape(sum(cam.Sampler.Weight, 2) > 0, cam.Sampler.Size);
    seen = seen(rowsIn, :);
    margin = 12;
    c0 = zeros(K, 1); c1 = c0;
    for k = 1:K
        cols = find(all(seen((k - 1) * nr + (1:nr), :), 1));
        if isempty(cols)
            return
        end
        c0(k) = cols(1) + margin; c1(k) = cols(end) - margin;
    end
    nc = min(c1 - c0 + 1);
    if nc < 32
        return
    end
    xk = zeros(K, 1);
    A = zeros(nr, nc, K, nPairs, 'like', ta);
    B = A;
    for k = 1:K
        r = (k - 1) * nr + (1:nr);
        cc = round((c0(k) + c1(k) - nc) / 2) + (1:nc);
        xk(k) = mean(cam.RowX(rowsIn(r)));
        A(:, :, k, :) = reshape(ta(r, cc, :), nr, nc, 1, nPairs);
        B(:, :, k, :) = reshape(tb(r, cc, :), nr, nc, 1, nPairs);
    end
    win = cast(hannWindow(nr) * hannWindow(nc)', 'like', real(ta));
    R = fft2(A .* win) .* conj(fft2(B .* win));
    R = R ./ max(abs(R), 1e-12) .^ options.Whitening;
    c = real(ifft2(R));                       % peak at the shift that maps current onto previous
    c = c ./ max(mean(abs(R), [1 2]), 1e-12);       % peak in [0, 1], 1 = identical bands
    [pk, idx] = max(reshape(c, nr * nc, []), [], 1);
    [iy, ix] = ind2sub([nr nc], idx);
    ty = (iy - 1) + subPixel(c, iy, ix, nr, nc, 1);
    tx = (ix - 1) + subPixel(c, iy, ix, nr, nc, 2);
    ty = gather(ty - nr * (ty > nr / 2));
    tx = gather(tx - nc * (tx > nc / 2));
    pk = gather(pk);
    flat = reshape(gather(squeeze(max(max(abs(A), [], 1), [], 2)) < 1e-6 | squeeze(max(max(abs(B), [], 1), [], 2)) < 1e-6), 1, []);
    pk(flat) = 0;
    ty = reshape(ty, K, nPairs); tx = reshape(tx, K, nPairs); pk = reshape(double(pk), K, nPairs);
    m.BandX = xk;
    m.BandDx = ty * dxr;
    for p = 1:nPairs
        w = pk(:, p);
        ok = w > 0;
        if nnz(ok) < 2
            continue
        end
        dxk = ty(ok, p) * dxr;                % row shift -> forward displacement
        dyk = tx(ok, p) * dyc;                % column shift -> lateral displacement
        w = w(ok);
        Ax = [ones(nnz(ok), 1), xk(ok)];
        cf = (Ax' * (Ax .* w)) \ ((Ax .* w)' * dyk);
        m.dx(p) = sum(w .* dxk) / sum(w);
        m.dy(p) = cf(1);
        m.dpsi(p) = cf(2);
        m.Peak(p) = mean(w);
    end
end

function t = texture(g)
    t = min(max(g - imgaussfilt(g, 4), -0.08), 0.08);
end

function w = hannWindow(n)
% Symmetric Hann window, the values of hann(n) of the Signal Processing Toolbox, which
% this was the only call to: the toolbox is not part of the install the camera path has
% to run on.
    if n == 1
        w = 1;
        return
    end
    w = 0.5 * (1 - cos(2 * pi * (0:n - 1)' / (n - 1)));
end

function d = subPixel(c, iy, ix, nr, nc, dim)
% Parabolic interpolation of the correlation peak along one dimension (circular).
    n = numel(iy);
    pages = 1:n;
    if dim == 1
        im = mod(iy - 2, nr) + 1; ip = mod(iy, nr) + 1;
        f0 = c(sub2ind([nr nc n], iy, ix, pages));
        fm = c(sub2ind([nr nc n], im, ix, pages));
        fp = c(sub2ind([nr nc n], ip, ix, pages));
    else
        im = mod(ix - 2, nc) + 1; ip = mod(ix, nc) + 1;
        f0 = c(sub2ind([nr nc n], iy, ix, pages));
        fm = c(sub2ind([nr nc n], iy, im, pages));
        fp = c(sub2ind([nr nc n], iy, ip, pages));
    end
    den = fm - 2 * f0 + fp;
    d = 0.5 * (fm - fp) ./ den;
    d(~isfinite(d) | abs(d) > 1) = 0;
end
