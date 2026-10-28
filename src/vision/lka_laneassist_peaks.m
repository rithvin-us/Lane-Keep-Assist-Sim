function [pts, slice, rows] = lka_laneassist_peaks(gray, A)
% LKA_LANEASSIST_PEAKS Square-pulse lane points of the LaneAssist row histograms.
%
% Syntax:
%   [pts, slice, rows] = lka_laneassist_peaks(gray, A)        A = LKA.Vision.LaneAssist
%
% Port of find_lane_peaks / peaks_detection of tsiggi/LaneAssist (MIT,
% Christos-Alexandros Tsingiropoulos, VROOM - BFMC 2023); see
% third_party/LaneAssist_LICENSE.txt.
%
% The bottom A.BottomPerc of the frame is cut into A.Slices rows, scanned bottom to
% top. A lane point is the centre of a "square pulse" in the row: a run of pixels above
% A.MinHeight that starts with a rise of more than A.MinHeightDif over A.PixDif pixels
% and ends with the matching fall, whose width lies in
%   [A.PeaksMinWidth, peaksMaxWidth + A.WidthError],
% where peaksMaxWidth tapers linearly from A.PeaksMaxWidth at the bottom row to
% A.PeaksMinWidth at the top row. The test is a local contrast test, not a threshold on
% the absolute level, which is what makes it tolerant of overall brightness.
%
% A.ResetWidthOnExit keeps the original behaviour (false): the original leaves the pulse
% width counter standing when a pulse dies because the level fell below A.MinHeight, so
% the next pulse in that row starts with a stale count and can be rejected as too wide.
% true resets the counter and is the deliberate deviation measured in
% docs/laneassist_comparison.md.
%
% Inputs:
%   gray  - H x W grayscale, 0..255 (uint8 or double)
% Outputs:
%   pts   - [col row] of every detected lane point, scan order
%   slice - slice index 1..numel(rows) of every point (for the clustering order)
%   rows  - scanned row indices, bottom to top

    g = double(gray);
    [h, w] = size(g);
    bottomRow = h - A.BottomOffset;
    step = fix(-(h * A.BottomPerc / A.Slices));
    if step == 0
        % fewer than one row per slice (a frame under A.Slices/A.BottomPerc rows high)
        pts = zeros(0, 2); slice = zeros(0, 1); rows = zeros(1, 0);
        return
    end
    endRow = fix((1 - A.BottomPerc) * h);
    nSlices = fix((endRow - bottomRow) / step);
    rows = bottomRow + (0:nSlices) * step;
    rows = rows(rows >= 1 & rows <= h);
    heightNorm = linspace(0, 1, nSlices + 1);

    pts = zeros(0, 2);
    slice = zeros(0, 1);
    for k = 1:numel(rows)
        maxWidth = A.PeaksMaxWidth - (A.PeaksMaxWidth - A.PeaksMinWidth) * heightNorm(k);
        cols = rowPulses(g(rows(k), :), w, A, maxWidth + A.WidthError);
        pts = [pts; cols(:), repmat(rows(k), numel(cols), 1)]; %#ok<AGROW>
        slice = [slice; repmat(k, numel(cols), 1)]; %#ok<AGROW>
    end
end

function cols = rowPulses(p, w, A, upperLimit)
% The original per-pixel state machine, kept sequential so the ported behaviour matches.
    cols = zeros(1, 0);
    inside = false;
    pixNum = 0;
    d = A.PixDif;
    for i = d + 1:w - d
        pixel = p(i);
        if inside
            if pixel - p(i + d) > A.MinHeightDif
                inside = false;
                if pixNum >= A.PeaksMinWidth && pixNum <= upperLimit
                    cols(end + 1) = i - floor((pixNum - d) / 2); %#ok<AGROW>
                end
                pixNum = 0;
            elseif pixel > A.MinHeight
                pixNum = pixNum + 1;
            else
                inside = false;
                if A.ResetWidthOnExit
                    pixNum = 0;
                end
            end
        elseif pixel > A.MinHeight && pixel - p(i - d) > A.MinHeightDif
            inside = true;
            pixNum = pixNum + 1;
        end
    end
end
