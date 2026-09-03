# Research State — auto-generated each cycle

_Last updated: 2026-09-03 · cycle demo-natural-motion_

## North star distance

P0(관절공간 추상화 + 충돌 검사기)와 P1(RRT-Connect 코어)이 모두 구현·검증되었다.
P2(경로 후처리)는 shortcut 평활화(`planning.shortcut`, MP-0005, PR #1)와 시간
파라미터화(`planning.trajectory`, MP-0006, PR #2)가 구현·테스트 완료 상태이고,
이제 **데모 실행 경로(`scripts/demo_plan_right_arm.py`)에도 실제로 연결됐다**
(MP-0023, PR #3) — RRT-Connect의 raw(지그재그) 경로를 그대로 재생하던 것을
shortcut+시간 파라미터화를 거친 경로로 교체해, 사용자가 지적한 "중간 자세가
부자연스럽다"는 문제를 직접 수정했다. 세 PR(#1/#2/#3) 모두 사람 리뷰/병합 대기 중.

동시에 P5(RRT* 대안 플래너)를 `planning/p5-rrt-star-planner`(MP-0016)에서
착수했다 — RRT-Connect는 첫 해를 찾으면 즉시 반환해 경로 품질이 운에
좌우되는데, RRT*는 시간 예산 안에서 계속 비용을 개선한다.

북극성까지는 P3(정식 실행 모듈) → P4(Cartesian goal·벤치마크) → P5(RRT*
비교연구, 진행 중)가 남는다.

## Current bottleneck

**PR #1/#2/#3(MP-0005/0006/0023) 사람 리뷰/병합 대기** — 셋 다 테스트
통과·구현 완료 상태로 막혀 있다. 이 중 하나라도 병합되기 전까지 executor는
`claude Doing 항목 1건만 허용` 불변식 때문에 새 planning TODO를 못 집는다
(이번 cycle에서 MP-0005/0006/0023을 Blocked로 재분류해 이 불변식은 다시
지켜지는 상태로 고쳤다 — 이전엔 Doing에 2건이 남아 있었다).

벤치마크 하네스(MP-0013) 부재도 여전한 2차 병목 — shortcut·time_parameterize
둘 다 "합성 시나리오/비공식" 수치만 있고, 실제 can-sort 장면 기준 정식
측정이 `results/*.tsv`에 없다. MP-0004(P1 성공률)·MP-0007(shortcut 전/후
비교)·MP-0014(pose goal 성공률)가 전부 이 하네스를 기다리고 있다.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p2-shortcut-smoothing | 2026-08-30 21:04 KST | MP-0005 shortcut 평활화, PR #1 리뷰 대기 | 4 |
| planning/p2-time-parameterize | 2026-08-31 11:10 KST | MP-0006 time_parameterize, PR #2 리뷰 대기 | 3 |
| planning/p2-demo-natural-motion | 2026-09-03 | MP-0023 데모 실행 경로 연결, PR #3 리뷰 대기 | 0 |
| planning/p5-rrt-star-planner | (진행 중) | MP-0016 RRT* 대안 플래너 초안 | 0 |

## Recent learnings (last 3 cycles)

- **하드웨어 관절 한계 ≠ 시뮬레이션 재생 컨트롤러가 실제로 추종 가능한
  속도.** `Trajectory`를 처음 실제로 소비하는 코드(MP-0023)를 짜면서 발견:
  config의 하드웨어 속도 상한(4.8 rad/s)을 그대로 재생하면 이 데모의
  오픈루프 PD+중력보상 토크 컨트롤러(`ArmTorqueController`)가 못 따라가
  중간 추종 오차가 1.5 rad까지 벌어진다. 하드웨어 스펙과 "이 컨트롤러가
  실제로 추종 가능한 속도"는 서로 다른 값으로 분리해서 관리해야 한다 —
  하나를 다른 하나에 맞춰 낮추면 스펙 자체가 왜곡된다. MP-0008(P3 실행
  모듈) 설계 시 이 구분을 반영해야 한다.
- **PR로 남겨 둔 검증된 코드는 review 지연과 무관하게 앞당겨 쓸 수 있다.**
  PR #1/#2가 병합 안 됐다고 그 코드가 못 쓰는 건 아니다 — 파일 내용을
  그대로 복사해 새 브랜치에서 먼저 쓰고, 나중에 원본 PR이 그대로
  병합되면 내용이 같아 충돌이 안 난다. 다만 TODO 상태(Doing/Blocked)는
  원본 PR의 병합 여부와 독립적으로 관리해야 한다.
- **속도 상한을 지키는 것과 가속도 상한을 지키는 것은 다른 조건이다**:
  각 경로 세그먼트 안에서 관절 속도 크기가 상한 이내라는 걸 증명해도,
  세그먼트를 이어붙일 때 방향이 바뀌면(코너) 그 경계의 관절 속도 방향
  전환 자체가 사실상 무한 가속도가 된다. "웨이포인트에서 멈추는" 기본형
  사다리꼴이 지름길이 아니라, 이 불연속을 원천적으로 없애는 정확한 해법이었다.
- **nullspace 정칙화 없이는 여유 자유도가 매번 임의로 재배치된다**:
  position-only IK가 남기는 4개 자유도를 무작위 재시도에만 맡기면 팔
  자세가 매번 크게 바뀐다. 단, 정칙화가 충돌 회피를 이기면 안 된다.

## Next claude-actionable

1. **`planning/p5-rrt-star-planner`(MP-0016)** — 진행 중. `rrt_star.py` 초안,
   `tests/test_planning_rrt_star.py`, 데모 `--planner {rrt_connect,rrt_star}`
   플래그, 문서화까지가 이번 cycle 범위.
2. **MP-0013** `scripts/benchmark_planning.py` — TSV append, 2분 예산. PR
   병합 후 우선순위가 가장 높다. MP-0004/0007/0014/0017(RRT* 비교표)이
   전부 이걸 기다리고 있다.
3. **MP-0008** `planning/execution.py` — `ArmTorqueController` 연결(P3).
   MP-0023에서 발견한 "재생 컨트롤러 대역폭 ≠ 하드웨어 스펙" 구분을 설계에
   반영해야 한다.

## Next user-blocked

1. **PR #1(MP-0005)·PR #2(MP-0006)·PR #3(MP-0023) 사람 리뷰/병합** — 셋 다
   테스트 통과, 병합 대기 중. 하나라도 병합돼야 executor의 Doing 슬롯이
   실질적으로 비워진다.
2. **MP-0020** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만 가능) —
   현재 알림이 전부 `research/cron_activity.md`에만 조용히 기록되고 있다.
3. **MP-0004** can-sort 10 seed 성공률 정식 측정 — 벤치마크 하네스(MP-0013)가
   먼저 있어야 TSV로 기록 가능.

## Cycles to date

10 (2026-08-30~09-03 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
반복/트리 시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스
목표+버그 수정 3건, nullspace 정칙화+hydrax 조사, 데모 실행 경로에 shortcut+
시간 파라미터화 연결, RRT* 대안 플래너 착수; 자율 루프: shortcut 평활화,
시간 파라미터화)
