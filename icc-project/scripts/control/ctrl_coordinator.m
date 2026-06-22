function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator Allocation + 마찰원 제한 (부호 있는 release 허용)

    g = 9.81;
    r_w = VEH.rw;
    ratio_f = 0.6;

    L = VEH.lf + VEH.lr;
    Fz_f_total = VEH.mass * g * VEH.lr / L;
    Fz_r_total = VEH.mass * g * VEH.lf / L;
    Fz = [Fz_f_total/2; Fz_f_total/2; Fz_r_total/2; Fz_r_total/2];
    muEstimate = 1.0;
    Fmax = muEstimate * Fz;

    %% (1) 부호 있는 분배 — release(양수 Fx)도 허용
    T_total_signed = -lonCmd.Fx_total * r_w;
    w = Fmax / sum(Fmax);
    brakeTorque = T_total_signed * w;

    %% (2) ESC yaw moment 차동 분배
    Mz = latCmd.yawMoment;
    t_f = VEH.track_f/2;
    t_r = VEH.track_r/2;
    dT_f = Mz * ratio_f / t_f;
    dT_r = Mz * (1-ratio_f) / t_r;
    if dT_f >= 0; brakeTorque(1)=brakeTorque(1)+dT_f; else; brakeTorque(2)=brakeTorque(2)+abs(dT_f); end
    if dT_r >= 0; brakeTorque(3)=brakeTorque(3)+dT_r; else; brakeTorque(4)=brakeTorque(4)+abs(dT_r); end

    %% (3) 마찰원 제한
    Fx_demand = brakeTorque / r_w;
    scale = min(Fmax ./ max(abs(Fx_demand), 1e-6), 1);
    brakeTorque = brakeTorque .* scale;

    %% (4) 최종 saturation
    brakeTorque = max(min(brakeTorque, LIM.MAX_BRAKE_TRQ), -LIM.MAX_BRAKE_TRQ);

    actuatorCmd.steerAngle   = max(min(latCmd.steerAngle, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);
    actuatorCmd.brakeTorque  = brakeTorque;
    actuatorCmd.dampingCoeff = verCmd;
end