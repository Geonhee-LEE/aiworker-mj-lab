# Research feed

_cap 30, 최신이 위. REVIEW 단계는 상위 5개만 읽는다._

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
