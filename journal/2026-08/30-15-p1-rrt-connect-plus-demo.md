# P1 RRT-Connect 코어 구현 + 실행 데모

- **Cycle**: 2026-08-30 15:35 KST
- **Branch**: (사람 주도, main 직접 작업)
- **TODO**: `MP-0002`, `MP-0003` [planner] RRT-Connect core + 무충돌 경로 속성 시험
- **Phase**: P1
- **Status**: keep

## What I tried

`src/ffw_sh5_grasp/planning/rrt_connect.py`에 표준 RRT-Connect(두 트리
EXTEND/CONNECT, goal-bias 샘플링, 결정론적 seed)를 구현하고, 순수 numpy
성질 시험(`tests/test_planning_rrt.py`)과 실제 can-sort 장면 시험
(`tests/test_planning_rrt_scene.py`)을 추가했다. 이어서 사용자 요청으로
`scripts/demo_plan_right_arm.py`를 만들어 계획 + (선택) MuJoCo 물리 재생 +
(선택) 실시간 뷰어까지 엔드투엔드로 실행 가능하게 했다.

## What worked / what failed

**버그 1 (CONNECT 로직)**: 최초 구현에서 `_connect`가 target이 아니라 방금
추가한 노드를 향해 확장하는 실수가 있었다. `test_planner_never_returns_a_
colliding_path` 속성 시험(20 seed)이 즉시 2.7 rad짜리 무효 edge를 잡아냈다.
표준 pseudocode(`repeat EXTEND(tree, target) until status != ADVANCED`)로
단순화해 해결했다.

**설계 실수 (시험 장애물)**: 처음 slab 장애물을 "joint0 in [1,2]는 항상 무효"로
정의했는데, 이건 다른 자유도로 돌아갈 틈이 없는 완전히 막힌 벽이라 RRT-Connect가
원리적으로 풀 수 없었다(에지 검사가 촘촘해서 넘어갈 수 없음). "joint1로 우회
가능한 틈이 있는 벽"으로 바꾸니 정상적으로 풀렸다 — 이건 알고리즘 버그가
아니라 시험 설계 실수였다.

**버그 2 (실행 재생)**: `--execute` 데모에서 최종 관절 오차가 2.4 rad로 전혀
수렴하지 않았다. 원인은 live `MjData`가 여전히 `home` 키프레임에 남아 있는
채로 재생을 시작한 것 — planner가 검증한 `start`로 옮기지 않았다. `home`
키프레임 자체가 상자 승격 후 겹치는 자세라는 P0의 기존 발견과 결합되어, 물리
시뮬레이션이 엉뚱한(충돌 중인) 자세에서 출발해 발산했다. 실행 전
`space.write(data.qpos, start)` + `mj_forward`를 추가하고 회귀 시험
(`test_plan_then_execute_converges`)으로 고정했다. 수정 후 최종 오차 ≈ 0.02 rad.

## North-star delta

전역 플래너가 실제로 경로를 만들고, 그 경로가 물리 시뮬레이션에서 목표에
수렴함을 확인했다 — "계획 → 실행"의 최소 엔드투엔드 고리가 처음 닫혔다.
다만 이건 정식 P3(시간 파라미터화 + 전용 실행 모듈)가 아니라 데모용
convergence-gated 임시 재생이다.

## Key learnings

- 무충돌-경로 속성 시험은 비용이 거의 안 드는데 실제 알고리즘 버그를 잡아냈다
  — 이런 종류의 property test를 앞으로도 우선한다.
- planner의 `start`/`goal` configuration은 "정적으로 유효하다"는 것과 "실제
  물리 시뮬레이션의 현재 상태와 일치한다"는 것이 별개다. 실행 전 항상 명시적
  동기화가 필요하다.
- `ArmTorqueController`의 정상 상태 오차는 크지 않지만, 목표까지 도달하는
  실시간 시간 예산은 waypoint당 최대 3초까지도 필요할 수 있다(대형 다관절
  동시 이동일수록). 정식 시간 파라미터화(P2)가 이 부분을 체계적으로 다뤄야 한다.

## Recommended next 1–3 priorities

1. MP-0005 shortcut 평활화
2. MP-0006 시간 파라미터화(사다리꼴 속도 프로파일) — 지금의 convergence-gated
   임시 재생을 대체
3. MP-0013 벤치마크 하네스 — MP-0004의 정식 TSV 기록을 가능하게 함

## Artifacts

- PR: 없음(사람 직접 작업, main 커밋 예정)
- Files touched: src/ffw_sh5_grasp/planning/rrt_connect.py,
  src/ffw_sh5_grasp/planning/__init__.py, scripts/demo_plan_right_arm.py,
  tests/test_planning_rrt.py, tests/test_planning_rrt_scene.py,
  TODO.md, STATE.md, JOURNAL.md
- TSV row appended: no (벤치마크 하네스가 아직 없음 — MP-0013 대기)
