# Research State — auto-generated each cycle

_Last updated: 2026-08-31 11:10 KST · cycle p2-time-parameterize_

## North star distance

P0(관절공간 추상화 + 충돌 검사기)와 P1(RRT-Connect 코어)이 모두 구현·검증되었다.
P2(경로 후처리)는 이제 두 조각 — shortcut 평활화(`planning.shortcut`, MP-0005,
PR #1)와 시간 파라미터화(`planning.trajectory`, MP-0006, PR #2) — 모두
구현·테스트 완료 상태다(두 PR 다 사람 리뷰/병합 대기). 실제 can-sort 장면에서
계획한 경로를 `ArmTorqueController`로 재생해 목표에 수렴하는 것까지 확인했다
(데모 스크립트, 정식 P3는 아직 아님). 두 PR이 병합되면 P2가 완전히 닫히고,
북극성까지는 P3(정식 실행 모듈) → P4(Cartesian goal·벤치마크) 두 단계가 남는다.

## Current bottleneck

벤치마크 하네스(MP-0013) 부재 — shortcut과 time_parameterize 둘 다 "합성
시나리오/비공식" 수치만 있고, 실제 can-sort 장면 기준 정식 측정이
`results/*.tsv`에 없다. MP-0004(P1 성공률)·MP-0007(shortcut 전/후 비교)·
MP-0014(pose goal 성공률)가 전부 이 하네스를 기다리고 있어 대기열이 늘고
있다. 그 다음 병목은 P3 실행 모듈(`planning.execution.follow_trajectory`,
MP-0008) — 이번에 나온 `Trajectory(times, positions)`를 실제로 소비하는
첫 지점인데 아직 없다.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p2-shortcut-smoothing | 2026-08-30 21:04 KST | MP-0005 shortcut 평활화, PR #1 리뷰 대기 | 0 |
| planning/p2-time-parameterize | 2026-08-31 11:10 KST | MP-0006 time_parameterize, PR #2 리뷰 대기 | 0 |

## Recent learnings (last 3 cycles)

- **속도 상한을 지키는 것과 가속도 상한을 지키는 것은 다른 조건이다**:
  각 경로 세그먼트 안에서 관절 속도 크기가 상한 이내라는 걸 증명해도,
  세그먼트를 이어붙일 때 방향이 바뀌면(코너) 그 경계의 관절 속도 방향
  전환 자체가 사실상 무한 가속도가 된다. "웨이포인트에서 멈추는" 기본형
  사다리꼴이 지름길이 아니라, 이 불연속을 원천적으로 없애는 정확한 해법이었다
  (실측: 전역 단일 프로파일에서 상한 4.0 대비 173 rad/s² 위반 발견 후 재설계).
  코너를 매끄럽게 잇는 blending은 MP-0006 범위 밖으로 명시 보류.
- **모든 관절이 같은 스칼라 속도·가속도 상한을 쓸 때, "관절별 독립 계산 후
  최댓값으로 동기화"와 "Linf(최대 성분) 거리 하나로 재파라미터화"는 수학적으로
  동치다** — 나중에 관절별로 다른 상한이 필요해지면 이 동치가 깨지므로 그때는
  진짜 관절별 계산으로 돌아가야 한다.
- **nullspace 정칙화 없이는 여유 자유도가 매번 임의로 재배치된다**:
  position-only IK가 남기는 4개 자유도를 무작위 재시도에만 맡기면 팔
  자세가 매번 크게 바뀐다. 표준 redundancy resolution(현재 관절값에 가깝게
  유지)을 추가했다. 단, 정칙화가 충돌 회피를 이기면 안 된다 — "부자연스러운"
  해가 실제로는 장애물 회피로 정당하게 재배치된 경우일 수 있다.
- **`research/feed.md`의 researcher 노트가 실행 가능한 설계 출발점을 준
  첫 사례**: MP-0006 사전조사(사다리꼴 동기화 방법론)가 이번 cycle의 구현
  방향을 그대로 제공했다 — Phase 0 인테이크가 값어치를 증명함.

## Next claude-actionable

1. **MP-0013** `scripts/benchmark_planning.py` — TSV append, 2분 예산. shortcut·
   time_parameterize 전/후 실제 can-sort 장면 수치를 재는 유일한 방법.
   MP-0004/0007/0014가 전부 이걸 기다리고 있어 우선순위를 올릴 가치가 크다.
2. **MP-0018** `aggregate_results.py`는 이미 있음 — 벤치마크 하네스(MP-0013)와
   연결해 P1/P2 결과를 `results/*.tsv`에 남기는 작업
3. **MP-0008** `planning/execution.py` — `ArmTorqueController` 연결(P3). PR #1/#2가
   병합되면 바로 착수 가능. `Trajectory(times, positions)`를 처음 실제 소비하는 지점.

## Next user-blocked

1. **MP-0020** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만 가능)
2. **MP-0004** can-sort 10 seed 성공률 정식 측정 — 벤치마크 하네스(MP-0013)가
   먼저 있어야 TSV로 기록 가능. 데모 스크립트로는 이미 여러 seed에서 성공을
   비공식 확인함(`scripts/demo_plan_right_arm.py --seed N --execute`)
3. PR #1(MP-0005)·PR #2(MP-0006) 사람 리뷰/병합 — 둘 다 테스트 통과, 병합
   대기 중

## Cycles to date

8 (2026-08-30~31 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모 반복/트리
시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스 목표+버그
수정 3건, nullspace 정칙화+hydrax 조사; 자율 루프: shortcut 평활화, 시간
파라미터화)
