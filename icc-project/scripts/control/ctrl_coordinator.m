function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator Allocation (부호 있는 brake 보정 — release 가능)

    r_w = VEH.rw;
    ratio_f = 0.6;

    %% (1) Fx_total → 부호 있는 4륜 분배
    %  Fx_total<0(브레이크 요청) → 양의 토크(브레이크 추가)
    %  Fx_total>0(가속/해제 요청) → 음의 토크(브레이크 감소=release)
    T_total_signed = -lonCmd.Fx_total * r_w;
    T_f_each = T_total_signed * ratio_f / 2;
    T_r_each = T_total_signed * (1-ratio_f) / 2;

    brakeTorque = [T_f_each; T_f_each; T_r_each; T_r_each];

    %% (2) ESC yaw moment 차동
    Mz  = latCmd.yawMoment;
    t_f = VEH.track_f/2;
    t_r = VEH.track_r/2;
    dT_f = Mz * ratio_f / t_f;
    dT_r = Mz * (1-ratio_f) / t_r;
    if dT_f >= 0; brakeTorque(1)=brakeTorque(1)+dT_f; else; brakeTorque(2)=brakeTorque(2)+abs(dT_f); end
    if dT_r >= 0; brakeTorque(3)=brakeTorque(3)+dT_r; else; brakeTorque(4)=brakeTorque(4)+abs(dT_r); end

    %% (5) 최종 saturation — 음수(release)도 허용
    brakeTorque = max(min(brakeTorque, LIM.MAX_BRAKE_TRQ), -LIM.MAX_BRAKE_TRQ);

    actuatorCmd.steerAngle   = max(min(latCmd.steerAngle, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);
    actuatorCmd.brakeTorque  = brakeTorque;
    actuatorCmd.dampingCoeff = verCmd;
end