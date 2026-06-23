function info = student_info()
    info.student_id   = '202420809';
    info.name         = '최연우';
    info.team_members = {};   % 2인 팀이면 {struct('id','...','name','...')} 추가
    info.course = '자동제어 - 2026 봄';
    info.ai_usage = ['환경 세팅 디버깅: Git/MATLAB 연동 트러블슈팅(예: ctrl_signature mismatch 오류를 git의 core.autocrlf 설정 문제로 진단). ' ...
        'ctrl_*.m 4개 파일의 1차 설계안 작성 및 코드 리뷰: PID+필터 구조(ctrl_lateral), 게이트+PI 구조(ctrl_longitudinal), on-off Skyhook(ctrl_vertical), WLS 분배(ctrl_coordinator)의 초안 제안. ' ...
        '다단계 게인 튜닝 지원: ctrl_lateral의 Kd 발산 원인을 미분항 노이즈로 진단하여 1차 저역통과 필터 도입을 제안 → 본인이 직접 Kd를 0.05→0.01로 정밀조정해 A3 완전 만점 검증. ctrl_longitudinal의 ABS 게이트를 슬립비 기반에서 종가속도 기반으로 전환하는 아이디어 제안, 이후 κ_target 재탐색(0.12→0.06) 방향 제안 → 본인이 grade.m 반복 실행으로 직접 최적값 확인. ' ...
        'grade.m의 local_score 함수 직접 분석을 통한 채점 메커니즘 규명(baseline 개선이 target 값과 무관한 1차 필요조건이라는 사실 확인). ' ...
        '모든 설계 결정, 파라미터 최종 선택, 안전성 검증(매 변경 후 grade.m 실행 및 결과 판단)은 본인이 직접 수행함.'];

    %% 검증 (수정 금지)
    if contains(info.student_id, 'TODO_FILL')
        warning('[student_info] 학번이 기입되지 않았습니다 — 채점 시 감점 + 매칭 불가');
    end
    if contains(info.name, 'TODO_FILL')
        warning('[student_info] 이름이 기입되지 않았습니다');
    end
end