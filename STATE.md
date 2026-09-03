# Research State — auto-generated each cycle

_Last updated: 2026-09-03 · cycle rrt-star-planner_

## North star distance

P0(관절공간 추상화 + 충돌 검사기)와 P1(RRT-Connect 코어)이 모두 구현·검증되었다.
P2(경로 후처리)는 shortcut 평활화(`planning.shortcut`, MP-0005, PR #1)와 시간
파라미터화(`planning.trajectory`, MP-0006, PR #2)가 구현·테스트 완료 상태이고,
데모 실행 경로(`scripts/demo_plan_right_arm.py`)에도 실제로 연결됐다(MP-0023,
PR #3) — RRT-Connect의 raw(지그재그) 경로를 그대로 재생하던 것을 shortcut+
시간 파라미터화를 거친 경로로 교체해, 사용자가 지적한 "중간 자세가
부자연스럽다"는 문제를 직접 수정했다.

P5(RRT* 대안 플래너)도 착수·완료됐다(`planning/p5-rrt-star-planner`, MP-0016,
PR #4) — 단일 트리 RRT*가 RRT-Connect와 같은 `PlannerResult`/`TreeSnapshot`
인터페이스를 공유해 데모에 `--planner {rrt_connect,rrt_star}`로 연결됐다.
MP-0015(RRT* 문헌조사)도 이 설계 과정으로 충족돼 Done 처리했다.

네 PR(#1/#2/#3/#4) 모두 사람 리뷰/병합 대기 중이다. 북극성까지는 P3(정식
실행 모듈) → P4(Cartesian goal·벤치마크) → P5(RRT-Connect vs RRT* 정식
비교표, 벤치마크 하네스 대기)가 남는다.

## Current bottleneck

**PR #1/#2/#3/#4(MP-0005/0006/0023/0016) 사람 리뷰/병합 대기** — 넷 다
테스트 통과·구현 완료 상태로 막혀 있다. `claude Doing` 슬롯은 현재 0건으로
비어 있지만(이번 cycle에서 네 항목 모두 Blocked로 재분류), 새 코드가
쌓일수록 리뷰 부담도 커진다 — PR 큐를 더 늘리기 전에 기존 4건을 먼저
소화하는 게 우선순위가 높다.

벤치마크 하네스(MP-0013) 부재도 여전한 2차 병목 — shortcut·time_parameterize·
RRT* 전부 "합성 시나리오/비공식" 수치만 있고, 실제 can-sort 장면 기준 정식
측정이 `results/*.tsv`에 없다. MP-0004(P1 성공률)·MP-0007(shortcut 전/후
비교)·MP-0014(pose goal 성공률)·MP-0017(RRT-Connect vs RRT* 비교표)이 전부
이 하네스를 기다리고 있다.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p2-shortcut-smoothing | 2026-08-30 21:04 KST | MP-0005 shortcut 평활화, PR #1 리뷰 대기 | 4 |
| planning/p2-time-parameterize | 2026-08-31 11:10 KST | MP-0006 time_parameterize, PR #2 리뷰 대기 | 3 |
| planning/p2-demo-natural-motion | 2026-09-03 | MP-0023 데모 실행 경로 연결, PR #3 리뷰 대기 | 0 |
| planning/p5-rrt-star-planner | 2026-09-03 | MP-0016 RRT* 대안 플래너, PR #4 리뷰 대기 | 0 |

## Recent learnings (last 3 cycles)

- **단일 트리 sampling 플래너의 하이퍼파라미터는 bidirectional 플래너의
  기본값을 그대로 물려받으면 안 된다.** RRT*(단일 트리)에 RRT-Connect의
  `goal_bias=0.1`을 그대로 썼더니 실제 can-sort 장면에서 40초·6750회
  반복 안에도 목표에 못 닿는 경우가 있었다 — bidirectional CONNECT처럼
  한 반복에 여러 스텝을 전진하지 못해서다(RRT-Connect가 애초에 고안된
  이유이기도 한 잘 알려진 트레이드오프). `goal_bias=0.3`/
  `goal_tolerance_rad=0.5`로 올려 해결. **합성 테스트에서 통과한다고
  실제 장면에서도 같은 파라미터가 통하는 게 아니다 — 반드시 실제 장면에서
  실측해야 한다.**
- **참조 구현의 property test를 그대로 이식하면 재검증 노출도 차이로
  깨질 수 있다.** RRT-Connect의 무충돌 경로 시험(계획보다 촘촘한 해상도로
  재검증)을 RRT*에 그대로 복사했다가 실패 — RRT*는 예산이 끝날 때까지
  계속 반복해 RRT-Connect보다 훨씬 많은 edge를 검사·rewire하므로 "해상도
  경계에 걸친 좁은 틈"에 노출될 확률이 커진다. 시험을 이식할 때는 "이
  시험이 실제로 보장하는 성질이 뭔가"를 다시 따져야 한다.
- **하드웨어 관절 한계 ≠ 시뮬레이션 재생 컨트롤러가 실제로 추종 가능한
  속도.** `Trajectory`를 처음 실제로 소비하는 코드(MP-0023)를 짜면서 발견:
  config의 하드웨어 속도 상한(4.8 rad/s)을 그대로 재생하면 이 데모의
  오픈루프 PD+중력보상 토크 컨트롤러가 못 따라가 중간 추종 오차가 1.5 rad까지
  벌어진다. 하드웨어 스펙과 "이 컨트롤러가 실제로 추종 가능한 속도"는 서로
  다른 값으로 분리해서 관리해야 한다. MP-0008(P3 실행 모듈) 설계 시 이
  구분을 반영해야 한다.
- **PR로 남겨 둔 검증된 코드는 review 지연과 무관하게 앞당겨 쓸 수 있다.**
  PR #1/#2가 병합 안 됐다고 그 코드가 못 쓰는 건 아니다 — 파일 내용을
  그대로 복사해 새 브랜치에서 먼저 쓰고, 나중에 원본 PR이 그대로
  병합되면 내용이 같아 충돌이 안 난다.

## Next claude-actionable

1. **MP-0013** `scripts/benchmark_planning.py` — TSV append, 2분 예산.
   MP-0004/0007/0014/0017이 전부 이걸 기다리고 있어 우선순위가 가장 높다.
2. **MP-0008** `planning/execution.py` — `ArmTorqueController` 연결(P3).
   MP-0023에서 발견한 "재생 컨트롤러 대역폭 ≠ 하드웨어 스펙" 구분을 설계에
   반영해야 한다.
3. PR #3(데모 natural-motion)이 병합되면 RRT*에도 shortcut+시간
   파라미터화를 연결해야 한다 — 지금은 두 브랜치가 독립적이라 `--planner
   rrt_star`의 경로는 아직 shortcut 후처리를 거치지 않는다.

## Next user-blocked

1. **PR #1(MP-0005)·PR #2(MP-0006)·PR #3(MP-0023)·PR #4(MP-0016) 사람
   리뷰/병합** — 넷 다 테스트 통과, 병합 대기 중.
2. **MP-0020** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만 가능) —
   현재 알림이 전부 `research/cron_activity.md`에만 조용히 기록되고 있다.
3. **MP-0004** can-sort 10 seed 성공률 정식 측정 — 벤치마크 하네스(MP-0013)가
   먼저 있어야 TSV로 기록 가능.

## Cycles to date

11 (2026-08-30~09-03 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
반복/트리 시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스
목표+버그 수정 3건, nullspace 정칙화+hydrax 조사, 데모 실행 경로에 shortcut+
시간 파라미터화 연결, RRT* 대안 플래너; 자율 루프: shortcut 평활화, 시간
파라미터화)
