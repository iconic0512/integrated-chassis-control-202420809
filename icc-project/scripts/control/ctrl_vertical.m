function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL Skyhook + 빈도 분리 (body bounce vs wheel hop)

    if isempty(fieldnames(suspState)) || ~isfield(suspState,'zs_dot') || ~isfield(suspState,'zu_dot')
        dampingCmd = 1500 * ones(4,1);
        return;
    end

    zs_dot = suspState.zs_dot;
    zu_dot = suspState.zu_dot;

    %% 초기화
    if ~isfield(ctrlState,'zsDotLP'); ctrlState.zsDotLP = zeros(4,1); end
    if ~isfield(ctrlState,'zuDotLP'); ctrlState.zuDotLP = zeros(4,1); end

    %% 저역통과 필터 — body bounce(저주파) 대역 추출, fc≈4Hz
    fc  = 4;
    tau = 1/(2*pi*fc);
    alpha = dt/(tau + dt);

    ctrlState.zsDotLP = ctrlState.zsDotLP + alpha*(zs_dot - ctrlState.zsDotLP);
    ctrlState.zuDotLP = ctrlState.zuDotLP + alpha*(zu_dot - ctrlState.zuDotLP);

    zs_dot_bb = ctrlState.zsDotLP;     % body-bounce 대역
    zu_dot_bb = ctrlState.zuDotLP;
    zs_dot_wh = zs_dot - zs_dot_bb;    % wheel-hop 대역 (고주파 잔차)

    %% (1) Body-bounce skyhook — 저주파 성분 기준
    relVel_bb = zs_dot_bb - zu_dot_bb;
    indicator_bb = zs_dot_bb .* relVel_bb;

    cSky = zeros(4,1);
    for i = 1:4
        if indicator_bb(i) > 0
            cSky(i) = CTRL.VER.cMax;
        else
            cSky(i) = CTRL.VER.cMin;
        end
    end

    %% (2) Wheel-hop 보호 — 고주파 우세 구간에서 댐핑 완화 (전달력 억제)
    whEnergy = abs(zs_dot_wh);
    bbEnergy = abs(zs_dot_bb) + 1e-3;
    whRatio  = whEnergy ./ (whEnergy + bbEnergy);   % 0~1

    dampingCmd = cSky .* (1 - whRatio) + CTRL.VER.cMin .* whRatio;

    %% (3) 클리핑
    dampingCmd = max(min(dampingCmd, CTRL.VER.cMax), CTRL.VER.cMin);
end