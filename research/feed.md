# Research feed

_cap 30, 최신이 위. REVIEW 단계는 상위 5개만 읽는다._

- [2026-09-03] [003](2026-09/003-vamp-mr.md) 사용자 제안 VAMP-MR(SIMD 가속
  multi-arm 플래너) 검토 — GitHub API로 사실관계 확인 결과 기술 주장은
  정확(10-100x, IROS 2026, Baxter dual-arm 지원 등). 다만 PRD Non-Goal(외부
  플래닝 라이브러리, OMPL 명시 제외)과 정면 충돌 + `libompl-dev` 등 전이
  의존성으로 실제로 OMPL을 끌고 들어옴 + sudo 시스템 패키지가 필요한 무거운
  C++ 빌드. hydrax(MP-0021)와 같은 취급으로 owner=user TODO만 등록,
  구현은 사용자의 명시적 PRD 수정 + 시스템 설치 승인 대기. 신규 TODO 1건
  (MP-0025).
- [2026-09-02] [002](2026-09/002.md) MP-0013 성공률은 raw 퍼센트 대신 Wilson
  score 95% CI로 보고할 것 — n=50 근처에서 반폭이 ~±8~10%p라 "45/50=90%"가
  실제로는 임계값 미달일 수 있음. `collision_state.py._forward`가 매 `is_valid`
  마다 스크래치 모델 전체(양팔·베이스 등)에 `mj_kinematics`+`mj_collision`을
  돌리는 구조 확인 — 2분 예산 초과 시 `state_checks` 카운터로 먼저 프로파일링
  후 `<exclude>` 보강, 모델 축소는 실측 없이 하지 말 것. 신규 TODO 1건
  (MP-0022, aggregate_results.py에 CI 계산 추가).
- [2026-09-01] [001](2026-09/001.md) MP-0013 벤치마크 하네스: 주 지표는 예산 내
  성공률, TSV엔 seed별 raw 행(집계는 별도)을 남겨 나중에 percentile/CDF 재계산
  가능하게 할 것, baseline vs shortcut/time_parameterize는 변수 하나만 바꿔
  비교(factorial). MP-0008 실행 모듈: `ArmTorqueController.apply`는 목표 속도가
  없어 궤적 추종 중 항상 위상 지연이 남을 수 있음(velocity feedforward 옵션
  기록) — 다만 P3 성공 기준이 "최종 정지 오차"라 먼저 feedforward 없이 측정 후
  필요시 추가 권장. 신규 TODO 없음(MP-0013/MP-0008/MP-0009가 이미 커버).
- [2026-08-31] [001](2026-08/001.md) MP-0006 시간 파라미터화: 관절별 사다리꼴
  시간을 계산해 `max_j(t_j)`로 세그먼트 동기화 + 역산 스케일 권장(moveit
  IterativeParabolicTimeParameterization과 동일 계열). TOPP-RA는 더 짧은
  실행시간을 낼 수 있으나 외부 라이브러리 의존 우려로 지금은 보류, P5 비교
  연구 후보로만 남김.
