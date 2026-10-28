function [validL, validR, widthMeas, widthValid] = lka_validate_boundaries(left, right, prevOffL, prevOffR, prevValidL, prevValidR, Val, Lane)
% LKA_VALIDATE_BOUNDARIES Plausibility gating of the detected lane boundaries.
%
% A boundary is used only if
%   1. the detector strength is high enough,
%   2. it lies on its own side of the vehicle (left boundary left, right right),
%      with SideMargin tolerance so a vehicle touching a line keeps the line,
%   3. heading and curvature are physically plausible,
%   4. its offset did not jump by more than JumpMax since the last accepted frame
%      (rejects mis-detections such as the opposite-lane line or a tar seam).
% When both pass, the implied lane width must lie inside [WidthMin, WidthMax];
% otherwise the weaker boundary is dropped.
%
% Checks 2-3 and the width are made at Val.CheckX metres ahead of the rear axle (0 =
% at the axle, the default). A camera first sees the road 4-6 m ahead; on a tight
% bend (R 3-8 m, lka_set_track_profile 'tight') the boundary extrapolated back to the
% axle can cross the vehicle axis or give a wrong width, so the checks are made near
% the look-ahead distance instead. The jump check stays at the axle (offset memory).
%
% Inputs:
%   left, right - LaneBoundaryBus structs from the lane detector
%   prevOffL/R  - lateral offsets accepted in the previous frame [m]
%   prevValidL/R- whether the previous frame accepted that boundary
%   Val, Lane   - LKA.Val and LKA.Lane calibration structs
%
% Outputs:
%   validL, validR - boundary accepted
%   widthMeas      - lane width implied by both boundaries [m] (0 if not both)
%   widthValid     - widthMeas is usable for the lane-width estimator

    x = 0;
    if isfield(Val, 'CheckX')
        x = Val.CheckX;
    end
    validL = boundaryOk(left,  +1, prevOffL, prevValidL, Val, x);
    validR = boundaryOk(right, -1, prevOffR, prevValidR, Val, x);

    widthMeas = 0;
    widthValid = false;
    if validL && validR
        [yL, psiL] = shapeAt(left, x);
        [yR, psiR] = shapeAt(right, x);
        psiMean = 0.5 * (psiL + psiR);
        w = (yL - yR) * cos(psiMean);
        if w >= Lane.WidthMin && w <= Lane.WidthMax
            widthMeas = w;
            widthValid = true;
        elseif left.Strength >= right.Strength
            validR = false;
        else
            validL = false;
        end
    end
end

function ok = boundaryOk(b, side, prevOff, prevValid, Val, x)
    [y, psi, kappa] = shapeAt(b, x);
    ok = b.Strength >= Val.StrengthMin ...
        && side * y > -Val.SideMargin ...
        && abs(y) <= Val.OffsetMax ...
        && abs(psi) <= Val.HeadingMax ...
        && abs(kappa) <= Val.CurvatureMax;
    if ok && prevValid
        ok = abs(b.LateralOffset - prevOff) <= Val.JumpMax;
    end
end

function [y, psi, kappa] = shapeAt(b, x)
% Offset [m], heading [rad] and curvature [1/m] of the boundary clothoid at distance x
% (x = 0: the bus values themselves).
    y = b.LateralOffset + tan(b.HeadingAngle) * x + b.Curvature / 2 * x^2 + b.CurvatureDerivative / 6 * x^3;
    psi = atan(tan(b.HeadingAngle) + b.Curvature * x + b.CurvatureDerivative / 2 * x^2);
    kappa = b.Curvature + b.CurvatureDerivative * x;
end
