# Research State — auto-generated each cycle

_Last updated: 2026-09-03 · cycle chomp-posture-smoothing_

## North star distance

P0(관절공간 추상화 + 충돌 검사기)와 P1(RRT-Connect 코어)이 모두 구현·검증되었다.
P2(경로 후처리)는 shortcut 평활화(MP-0005, PR #1), 시간 파라미터화(MP-0006,
PR #2), 데모 실행 경로 연결(MP-0023, PR #3), 그리고 이번에 추가된 CHOMP류
궤적 최적화 후처리(`planning.chomp`, MP-0024, PR #5)까지 네 조각이 모두
구현·테스트 완료 상태다.

사용자가 남은 자연스러움 문제를 세 층으로 명확히 짚었다: **IK 계산이
자주 실패**(계획 이전 목표 탐색 문제) / **연속 동작이 부드럽지 않음**
(실행·제어 층 문제) / **팔 자세가 기괴함**(비용 함수 부재 문제 — RRT-Connect는
비용이 없고 RRT*의 비용은 경로 길이일 뿐, shortcut도 경로를 짧게 할 뿐
매끄럽게 하지 않는다). 이 중 **자세 기괴함**을 CHOMP류 가속도 최소화
QP(`planning/chomp.py`)로 해결했다 — 관절 간 결합이 없어 7-DOF를 독립
1-D QP 7개로 풀고, `kinematics/optimization.py`의 기존 QP 유틸리티를
수정 없이 재사용. trust region + 재검증 + 폴백으로 무효 경로 불변식을
유지한다. 데모 `--posture-smooth`로 연결, 매끄러움 비용이 실측상 대부분
0에 가깝게 개선됐다.

P5(RRT* 대안 플래너)도 착수·완료됐다(MP-0016, PR #4).

**다섯 PR(#1~#5) 모두 사람 리뷰/병합 대기 중**이다. 북극성까지는 P3(정식
실행 모듈) → P4(Cartesian goal·벤치마크) → P5(RRT-Connect vs RRT* 정식
비교표)가 남고, 사용자가 지적한 나머지 두 한계(IK 실패, 연속 동작)도
후속 우선순위로 남아 있다.

## Current bottleneck

**PR #1~#5(MP-0005/0006/0023/0016/0024) 사람 리뷰/병합 대기** — 다섯 다
테스트 통과·구현 완료 상태로 막혀 있다. PR 큐가 5개로 늘었으니, 지금은
새 코드 작업보다 **기존 PR을 먼저 소화하는 게 우선순위가 높다** — 이
경고는 지난 cycle부터 반복되고 있다.

벤치마크 하네스(MP-0013) 부재도 여전한 2차 병목 — 네 조각(shortcut·
time_parameterize·RRT*·chomp) 전부 "합성 시나리오/비공식" 수치만 있고,
실제 can-sort 장면 기준 정식 측정이 `results/*.tsv`에 없다. MP-0004·
MP-0007·MP-0014·MP-0017이 전부 이 하네스를 기다리고 있다.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p2-shortcut-smoothing | 2026-08-30 21:04 KST | MP-0005 shortcut 평활화, PR #1 리뷰 대기 | 4 |
| planning/p2-time-parameterize | 2026-08-31 11:10 KST | MP-0006 time_parameterize, PR #2 리뷰 대기 | 3 |
| planning/p2-demo-natural-motion | 2026-09-03 | MP-0023 데모 실행 경로 연결, PR #3 리뷰 대기 | 0 |
| planning/p5-rrt-star-planner | 2026-09-03 | MP-0016 RRT* 대안 플래너, PR #4 리뷰 대기 | 0 |
| planning/chomp-posture-smoothing | 2026-09-03 | MP-0024 CHOMP류 궤적 최적화, PR #5 리뷰 대기 | 0 |

## Recent learnings (last 3 cycles)

- **관절 간 결합이 없는 비용+제약이면 D개의 독립 1-D QP가 1개의 D·K차원
  QP보다 낫다.** CHOMP류 가속도 최소화 비용은 관절별로 완전히 분리되므로
  7-DOF를 7개의 작은 QP로 풀었다 — 구현이 단순해지고 손으로 검증하기도
  쉬웠다(`kinematics/optimization.py`를 전혀 수정하지 않고 그대로 재사용).
- **QP 기반 경로 후처리의 안전장치(trust region + 재검증 + 폴백)는 실제
  장면에서도 반드시 발동을 확인해야 한다.** 합성 fixture 테스트뿐 아니라
  실제 can-sort 장면 데모(`--loop 3`)에서도 한 cycle에서 최적화가
  충돌을 유발해 폴백이 정확히 작동함을 확인했다 — "이론상 안전하다"와
  "실제로 안전하게 동작한다"의 차이를 좁혀준다.
- **단일 트리 sampling 플래너의 하이퍼파라미터는 bidirectional 플래너의
  기본값을 그대로 물려받으면 안 된다.** RRT*에 RRT-Connect의
  `goal_bias=0.1`을 그대로 썼더니 실제 장면에서 목표에 못 닿는 경우가
  있었다 — `goal_bias=0.3`/`goal_tolerance_rad=0.5`로 해결. 합성 테스트
  통과가 실제 장면 성공을 보장하지 않는다.
- **하드웨어 관절 한계 ≠ 시뮬레이션 재생 컨트롤러가 실제로 추종 가능한
  속도.** config의 하드웨어 속도 상한(4.8 rad/s)을 그대로 재생하면 이
  데모의 오픈루프 PD+중력보상 토크 컨트롤러가 못 따라간다. 하드웨어
  스펙과 "이 컨트롤러가 실제로 추종 가능한 속도"는 분리 관리해야 한다.

## Next claude-actionable

1. **MP-0013** `scripts/benchmark_planning.py` — TSV append, 2분 예산.
   MP-0004/0007/0014/0017이 전부 이걸 기다리고 있어 우선순위가 가장 높다.
2. 사용자가 우선순위를 정하면 나머지 두 한계(IK 실패 개선 — `MP-0011`
   `planning/goals.py` 스마트 시딩; 연속 동작 부드러움 — 실행/제어 층,
   `MP-0021` hydrax/MPPI와 연관) 중 하나를 이어서 진행.
3. **MP-0008** `planning/execution.py` — `ArmTorqueController` 연결(P3).
   MP-0023에서 발견한 "재생 컨트롤러 대역폭 ≠ 하드웨어 스펙" 구분을 설계에
   반영해야 한다.

## Next user-blocked

1. **PR #1~#5(MP-0005/0006/0023/0016/0024) 사람 리뷰/병합** — 다섯 다
   테스트 통과, 병합 대기 중. PR 큐가 늘어나고 있어 우선순위가 높다.
2. **MP-0020** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만 가능) —
   현재 알림이 전부 `research/cron_activity.md`에만 조용히 기록되고 있다.
3. **MP-0004** can-sort 10 seed 성공률 정식 측정 — 벤치마크 하네스(MP-0013)가
   먼저 있어야 TSV로 기록 가능.

## Cycles to date

12 (2026-08-30~09-03 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
반복/트리 시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스
목표+버그 수정 3건, nullspace 정칙화+hydrax 조사, 데모 실행 경로에 shortcut+
시간 파라미터화 연결, RRT* 대안 플래너, CHOMP류 궤적 최적화 후처리; 자율
루프: shortcut 평활화, 시간 파라미터화)
