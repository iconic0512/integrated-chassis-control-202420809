function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL 속도추종 PI + 게이트 있는 κ=0.12 직접추종 ABS

    if ~isfield(ctrlState,'intError');   ctrlState.intError   = 0; end
    if ~isfield(ctrlState,'prevForce');  ctrlState.prevForce  = 0; end
    if ~isfield(ctrlState,'wheelSlip');  ctrlState.wheelSlip  = zeros(4,1); end
    if ~isfield(ctrlState,'absActive');  ctrlState.absActive  = false; end
    if ~isfield(ctrlState,'slipIntErr'); ctrlState.slipIntErr = 0; end

    err = vxRef - vx;
    ctrlState.intError = ctrlState.intError + err*dt;
    ctrlState.intError = max(min(ctrlState.intError, CTRL.LON.intMax), -CTRL.LON.intMax);
    Fx_track = CTRL.LON.Kp*err + CTRL.LON.Ki*ctrlState.intError;

    kappaEngage  = 0.15;
    kappaRelease = 0.05;
    kappaTarget  = 0.12;
    Fx_abs = 0;

    if ax < 0
        kappaMeas = mean(abs(ctrlState.wheelSlip));

        if ~ctrlState.absActive && kappaMeas > kappaEngage
            ctrlState.absActive = true;
            ctrlState.slipIntErr = 0;
        elseif ctrlState.absActive && kappaMeas < kappaRelease
            ctrlState.absActive = false;
        end

        if ctrlState.absActive
            kappaErr = kappaMeas - kappaTarget;
            ctrlState.slipIntErr = ctrlState.slipIntErr + kappaErr*dt;
            ctrlState.slipIntErr = max(min(ctrlState.slipIntErr, 1), -1);
            Fx_abs = (CTRL.LON.absKp*kappaErr + CTRL.LON.absKi*ctrlState.slipIntErr) * LIM.MAX_BRAKE_TRQ;
        end
    else
        ctrlState.absActive = false;
    end

    if ax < 0
        Fx_des = Fx_abs;
    else
        Fx_des = Fx_track;
    end

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
