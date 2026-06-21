function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL 속도추종 PI + 히스테리시스 ABS + release cap

    if ~isfield(ctrlState,'intError');  ctrlState.intError  = 0; end
    if ~isfield(ctrlState,'prevForce'); ctrlState.prevForce = 0; end
    if ~isfield(ctrlState,'wheelSlip'); ctrlState.wheelSlip = zeros(4,1); end
    if ~isfield(ctrlState,'absActive'); ctrlState.absActive = false; end

    %% (1) 속도추종 PI
    err = vxRef - vx;
    ctrlState.intError = ctrlState.intError + err*dt;
    ctrlState.intError = max(min(ctrlState.intError, CTRL.LON.intMax), -CTRL.LON.intMax);
    Fx_track = CTRL.LON.Kp*err + CTRL.LON.Ki*ctrlState.intError;

    %% (2) ABS — 히스테리시스 + release cap
    kappaEngage  = 0.15;
    kappaRelease = 0.08;
    Fx_abs = 0;
    if ax < 0
        kappaMeas = mean(abs(ctrlState.wheelSlip));

        if ~ctrlState.absActive && kappaMeas > kappaEngage
            ctrlState.absActive = true;
        elseif ctrlState.absActive && kappaMeas < kappaRelease
            ctrlState.absActive = false;
        end

        if ctrlState.absActive
            Fx_abs = CTRL.LON.absKp * (kappaMeas - kappaRelease) * LIM.MAX_BRAKE_TRQ;
        end
    end

    Fx_des = Fx_abs;
    if ax >= 0
        Fx_des = Fx_track;
    end

    %% (3) jerk limit
    maxDeltaFx = LIM.MAX_DFX * dt;
    deltaFx = max(min(Fx_des - ctrlState.prevForce, maxDeltaFx), -maxDeltaFx);
    Fx_total = ctrlState.prevForce + deltaFx;
    ctrlState.prevForce = Fx_total;

    forceCmd.Fx_total = Fx_total;
    if Fx_total < 0
        forceCmd.brakeRatio = min(abs(Fx_total)/LIM.MAX_BRAKE_TRQ, 1);
    else
        forceCmd.brakeRatio = 0;
    end
end