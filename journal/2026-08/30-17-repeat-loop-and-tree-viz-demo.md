# 데모 스크립트에 반복 목표 방문과 RRT 트리 시각화 추가

- **Cycle**: 2026-08-30 17:00 KST
- **Branch**: (사람 주도, main 직접 작업)
- **TODO**: (신규 TODO 없음 — `scripts/demo_plan_right_arm.py` 확장, 사용자 직접 요청)
- **Phase**: P1 (데모 도구 개선)
- **Status**: keep

## What I tried

두 가지를 추가했다.

1. **반복 목표 방문(`--loop N`)**: 한 목표에 도착하면 그 configuration을 다음
   cycle의 시작점으로 삼아 새 무작위 목표를 다시 계획·실행한다. `--goal`은
   첫 cycle에만 적용되고 이후는 항상 `_sample_valid_goal`로 뽑는다. `N<=0`이면
   뷰어 창이 닫히거나 인터럽트될 때까지 무한 반복한다.
2. **RRT 트리 시각화(`--show-tree`)**: `plan_rrt_connect`가 반환하는
   `PlannerResult`에 `TreeSnapshot`(노드 배열 + 부모 인덱스 배열) 두 개
   (`start_tree`, `goal_tree`)를 추가했다. 각 노드(7-DOF configuration)를
   `grasp_target_r` site의 world 좌표로 FK 순전파해 3D 점으로 투영하고,
   `mujoco.mjv_initGeom`/`mjv_connector`로 `viewer.user_scn`에 구체(노드)와
   선분(edge)을 그려 실제 탐색 트리를 눈으로 볼 수 있게 했다.

## What worked / what failed

`TreeSnapshot`을 추가하면서 `plan_rrt_connect`의 5곳이던 반환문을 `_result()`
헬퍼 하나로 통합했다. 트리 A/B가 매 반복 서로 swap되기 때문에, 반환 시점에
`swapped` 플래그로 "시작 쪽 트리/목표 쪽 트리" 순서를 항상 복원해야 한다 —
`node_counts`는 이미 이 로직이 있었으므로 그대로 재사용해 일관성을 지켰다.

`_tree_site_positions`로 계산한 site 위치가 NaN 없이 물리적으로 타당한
범위(작업대 근처 x∈[-0.2,0.3], z∈[0.87,1.35])에 나오는 것을 직접 검증했다.
실제 디스플레이(`DISPLAY=:0`)에서 `--loop 2 --execute --show-tree`를 두 번
돌려 세그폴트 없이 exit 0으로 끝나는 것을 확인했다(직전 cycle에서 고친
`with` 컨텍스트 매니저 + `os._exit(0)` 패턴이 반복 실행에도 그대로 안전하게
적용됨).

## North-star delta

데모 도구의 사용성이 개선됐을 뿐, 알고리즘 자체(P1)에는 변화가 없다.

## Key learnings

- `PlannerResult`에 새 필드를 추가할 때 생성 지점이 여러 곳(성공/실패
  경로마다 별도 return문)에 흩어져 있으면 헬퍼 함수로 모으는 게 낫다 —
  안 그러면 필드 하나 추가할 때마다 N곳을 손으로 맞춰야 한다.
- MuJoCo의 `viewer.user_scn`은 `mjv_initGeom` + `mjv_connector`로 임의의
  디버그 지오메트리를 씬 지오메트리와 별개로 그릴 수 있는 표준 방법이다
  (`maxgeom` 기본 100000, 여유 충분).

## Recommended next 1–3 priorities

1. MP-0005 shortcut 평활화
2. MP-0006 시간 파라미터화
3. (선택) 트리 시각화를 pytest 회귀로 고정할지 검토 — 현재는 수동 검증만 함

## Artifacts

- PR: 없음(사람 직접 작업)
- Files touched: src/ffw_sh5_grasp/planning/rrt_connect.py,
  src/ffw_sh5_grasp/planning/__init__.py, scripts/demo_plan_right_arm.py,
  docs/guide/motion-planning.md
- TSV row appended: no (데모 도구 변경, 정량 지표 없음)
