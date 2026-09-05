# Research feed

_cap 30, 최신이 위. REVIEW 단계는 상위 5개만 읽는다._

- [2026-09-06] [006](2026-09/006.md) PR 큐 완전 소진 후 다음 착수 후보인
  MP-0017(RRT-Connect vs RRT* 50-seed 비교표)을 위해 통계 검정 방법을 미리
  확정 — 동일 seed로 두 플래너를 돌리는 대응 표본 구조이므로 독립 표본
  검정(Mann-Whitney) 대신 **Wilcoxon signed-rank test**(대응 t-test의
  비모수 버전) + 중앙값 effect size를 함께 보고할 것, 한쪽만 실패한 seed는
  성공 교집합에서 제외하고 교집합이 너무 작으면 검정 자체를 생략. MP-0022
  (Wilson CI)는 성공률 축이라 이것과 별개. 신규 TODO 0건(MP-0017이 이미
  커버).
- [2026-09-05] [005](2026-09/005.md) 충돌 검사 가속 후보로 safety-certificate
  스타일 캐싱(Bialkowski et al., IJRR 2016) 조사 — 한 번 정밀 검사한 점의
  `clearance()` 반경 안에서는 이후 `is_valid`를 생략할 수 있다는 아이디어.
  다만 이 저장소는 인증서 반경을 얻으려면 `is_valid`보다 훨씬 비싼
  `clearance()`를 호출해야 해 순이득이 반경≫`resolution_rad`일 때만 나온다 —
  실측 없이 판단 불가. 바로 구현하지 않고 프로파일링부터 하는 걸로 스코프를
  좁힌 신규 TODO 1건(MP-0028, 벤치마크 하네스 PR #7 병합 후 자연스러움).
- [2026-09-04] [004](2026-09/004.md) 병목이 PR 리뷰 큐라 신규 설계보다 다음
  착수 후보(MP-0008 실행 feedforward, MP-0011 IK 시딩) 검증에 집중. 문헌
  확인 결과 둘 다 기존 계획(velocity feedforward는 보류 후 필요시 추가,
  IK는 이전 성공 해+reachability 격자 시딩)이 이미 올바른 방향임을 재확인
  — 특이점 근처에서는 warm-start만으론 부족해 무작위 재시도 폴백 병행이
  필요하다는 근거(GNN warm-start 100%→93% 사례)만 추가. 신규 TODO 0건.
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
