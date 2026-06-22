function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL AFS (PID + filtered derivative) + ESC

    if ~isfield(ctrlState,'intError');  ctrlState.intError  = 0; end
    if ~isfield(ctrlState,'prevError'); ctrlState.prevError = 0; end
    if ~isfield(ctrlState,'filtDeriv'); ctrlState.filtDeriv = 0; end

    err = yawRateRef - yawRate;
    ctrlState.intError = ctrlState.intError + err*dt;
    ctrlState.intError = max(min(ctrlState.intError, CTRL.LAT.intMax), -CTRL.LAT.intMax);

    %% 미분항에 저역통과 필터 적용 (노이즈/발산 억제)
    rawDeriv = (err - ctrlState.prevError)/dt;
    ctrlState.prevError = err;

    alpha = 0.1;
    ctrlState.filtDeriv = alpha*rawDeriv + (1-alpha)*ctrlState.filtDeriv;
    derivError = ctrlState.filtDeriv;

    steerAdd = CTRL.LAT.Kp*err + CTRL.LAT.Ki*ctrlState.intError + CTRL.LAT.Kd*derivError;
    steerAdd = max(min(steerAdd, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);

    speedFactor = min(vx/CTRL.LAT.vRef, 2);

    yawMoment = 0;
    if abs(slipAngle) > CTRL.LAT.betaTh
        yawMoment = -CTRL.LAT.Kbeta * sign(slipAngle) * (abs(slipAngle) - CTRL.LAT.betaTh) * speedFactor;
    end

    deltaAdd.steerAngle = steerAdd;
    deltaAdd.yawMoment  = yawMoment;
end