function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL AFS (PID yaw rate 추종) + ESC (slip angle 리미터, 속도 스케줄링)

    %% 내부 상태 초기화
    if ~isfield(ctrlState,'intError');  ctrlState.intError  = 0; end
    if ~isfield(ctrlState,'prevError'); ctrlState.prevError = 0; end

    %% (1) yaw rate 추종 PID → AFS steerAngle
    err = yawRateRef - yawRate;
    ctrlState.intError = ctrlState.intError + err*dt;
    ctrlState.intError = max(min(ctrlState.intError, CTRL.LAT.intMax), -CTRL.LAT.intMax);  % (4) anti-windup

    derivError = (err - ctrlState.prevError)/dt;
    ctrlState.prevError = err;

    steerAdd = CTRL.LAT.Kp*err + CTRL.LAT.Ki*ctrlState.intError + CTRL.LAT.Kd*derivError;

    %% (4) saturation
    steerAdd = max(min(steerAdd, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);

    %% (3) speed scheduling factor
    speedFactor = min(vx/CTRL.LAT.vRef, 2);

    %% (2) slip angle 임계 초과 시 ESC yaw moment (driver intent와 반대 방향)
    yawMoment = 0;
    if abs(slipAngle) > CTRL.LAT.betaTh
        yawMoment = -CTRL.LAT.Kbeta * sign(slipAngle) * (abs(slipAngle) - CTRL.LAT.betaTh) * speedFactor;
    end

    %% 출력
    deltaAdd.steerAngle = steerAdd;
    deltaAdd.yawMoment  = yawMoment;
end