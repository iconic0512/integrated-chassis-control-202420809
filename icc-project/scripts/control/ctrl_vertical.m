function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL On-off Skyhook (semi-active)

    %% 입력 안전장치 — suspState가 비어있으면(bicycle/3dof 모델 등) passive로 fallback
    if isempty(fieldnames(suspState)) || ~isfield(suspState,'zs_dot') || ~isfield(suspState,'zu_dot')
        dampingCmd = 1500 * ones(4,1);
        return;
    end

    zs_dot = suspState.zs_dot;   % 4×1
    zu_dot = suspState.zu_dot;   % 4×1

    %% (1) On-off Skyhook
    relVel    = zs_dot - zu_dot;
    indicator = zs_dot .* relVel;   % >0 이면 강한 댐핑 필요

    dampingCmd = zeros(4,1);
    for i = 1:4
        if indicator(i) > 0
            dampingCmd(i) = CTRL.VER.cMax;
        else
            dampingCmd(i) = CTRL.VER.cMin;
        end
    end

    %% (2) 최종 클리핑 (안전장치)
    dampingCmd = max(min(dampingCmd, CTRL.VER.cMax), CTRL.VER.cMin);
end
