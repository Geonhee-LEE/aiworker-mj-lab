# tests/offline_pose_ik.py를 planning.goals로 위임(중복 제거)

- **Cycle**: 2026-09-12 11:00 KST
- **Branch**: `planning/p4-offline-pose-ik-delegate`
- **TODO**: `MP-0012` [planner] `tests/offline_pose_ik.py`를 `planning.goals`로 위임(중복 제거)
- **Phase**: P4
- **Status**: keep

## What I tried

Cadence gate 통과(PR 큐 1건, stuck Doing 없음, 최근 24시간 planning 브랜치 2개,
Backlog 비어있지 않음) 후 STATE.md/journal 상위 5개를 리뷰. STATE.md는
2026-09-07 스냅샷으로 낡아 있었지만 `gh pr list`로 실제로는 오늘 오전 사이클
(PR #17/#18)이 PR #13~#16을 모두 병합 상태로 동기화해 둔 걸 확인 — MP-0012가
막혀 있던 이유("PR #14가 병합돼 `planning/goals.py`가 main에 들어와야 착수
가능")가 이미 해소됐음을 재확인하고 이 TODO를 골랐다.

`tests/offline_pose_ik.py`의 `solve_offline_pose`/`solve_offline_pose_multistart`를
읽어보니 `planning/goals.py`의 `solve_pose_goal`/`solve_pose_goal_multistart`와
DLS/backtracking 반복 로직이 사실상 한 글자도 다르지 않게 복붙돼 있었다(PR #14
당시 "test_offline_pose_ik.py를 골자로 하되 clip/sample을 RightArmSpace로
위임"이라고 명시했던 그 사본). 차이는 딱 두 곳: (1) `_clamp`가 `solver.joint_limited`/
`joint_ranges`를 직접 쓰는 반면 `planning.goals`는 `RightArmSpace.clip()`을 씀,
(2) multistart 재시도 샘플링이 `rng.uniform(lower, upper)`를 직접 vs
`space.sample(rng)`. 실제 오른팔 7관절(`arm_r_joint1..7`)이 전부 `limited=True`
실기 관절이라 두 경로가 수치적으로 동치임을 `RIGHT_ARM_JOINTS`/`ARM_JOINTS`/
`ARM_R` 세 호출부 모두에서 확인.

`tests/offline_pose_ik.py`를 solver의 관절 범위로 임시 `RightArmSpace`를 만들어
`planning.goals`에 위임하는 얇은 어댑터로 교체(108줄 → 28줄). 호출부
(`test_phase_3.py`/`test_phase_4.py`/`tests/record_demo.py`)가 기대하는 4-튜플
반환과 키워드 인자 시그니처는 그대로 유지해 호출부는 한 줄도 건드리지 않았다.

## What worked / what failed

`planning.goals`가 이미 `space` 매개변수를 받는 구조라 어댑터가 아주 얇았다
(`RightArmSpace`를 원시 생성자로 바로 만들면 됨 — `from_limits()`는 `limited`를
전부 True로 강제해 못 씀). 회귀 위험은 "실제 관절이 전부 limited인가"였는데
`grep`으로 세 호출부의 관절 이름 목록이 전부 `arm_r_joint{1..7}`/`arm_l_joint{1..7}`
류의 실기 관절임을 먼저 확인한 뒤 진행해 실패 없이 한 번에 통과했다.

## North-star delta

P4 Cartesian pose goal 트랙의 마지막 정리 조각 — 같은 IK 알고리즘이 프로덕션
경로(`planning.goals`)와 테스트 전용 경로(`tests/offline_pose_ik.py`)에 중복
존재하던 걸 닫아, 앞으로 DLS/backtracking 로직을 고칠 때 한 곳만 고치면 된다.

## Key learnings

- **"solver만 있고 space가 없는" 호출부를 `planning.goals`에 위임하려면 굳이
  전체 모델을 다시 열 필요 없이 solver가 이미 들고 있는
  `joint_ranges`/`joint_limited`/`n`으로 임시 `RightArmSpace`를 즉석
  구성하면 충분하다** — `RightArmSpace.from_limits()`는 단위 테스트용으로
  `limited=all-True`를 강제하므로 이런 경우엔 원시 생성자를 직접 써야 한다.
- **"두 구현이 동치인지"는 실제 데이터(관절이 진짜 limited인지)를 코드에서
  확인하고 나서 위임해야 한다** — 코드가 보기엔 비슷해도 unlimited 관절이
  섞여 있었다면 샘플링 분포가 달라져 조용한 회귀가 될 뻔했다.

## Recommended next 1–3 priorities

1. PR #17/#18/#19 사람 리뷰·병합.
2. `MP-0022`(P4) — `aggregate_results.py`에 Wilson CI 계산 추가, 의존성 없음.
3. `MP-0037`(P1) — `_solve_valid_ik` q_init 반경 확장 샘플링(1차 후보, R-F-009).

## Artifacts

- PR: https://github.com/Geonhee-LEE/aiworker-mj-lab/pull/19
- Files touched: `tests/offline_pose_ik.py`, `results/p4-offline-pose-ik-delegate.tsv`
- TSV row appended: yes
