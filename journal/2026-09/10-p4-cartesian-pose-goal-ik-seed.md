# P4 착수: Cartesian pose 목표 → IK 다중 재시도 (`planning/goals.py`)

- **Cycle**: 2026-09-06 11:05 KST
- **Branch**: `planning/p4-cartesian-pose-goal-ik-seed`
- **TODO**: `MP-0011` `planning/goals.py` Cartesian pose goal → IK 시드 다중 재시도
- **Phase**: P4
- **Status**: keep

## What I tried

PR 큐가 비어 있고 Today가 빈 상태라 `STATE.md`의 "Next claude-actionable" 1순위
(MP-0007/MP-0017 벤치마크 측정)를 확인했더니 이미 실측 완료·PR #13 리뷰 대기
(`Blocked`)였다. 2순위인 `MP-0011`(P1, P4)을 골랐다 — P4(Cartesian goal)는
로드맵상 완전 미착수 단계였고, RRT-Connect/RRT*가 관절공간 목표만 받는다는
제약을 확인한 뒤 그 첫 조각(pose→q_goal 변환)을 구현했다.

기존 `tests/offline_pose_ik.py`(주석에 "실시간 제품 API가 아니다"라고 명시된
test-only 헬퍼)에 이미 position-우선 DLS + backtracking + 다중 재시도 로직이
있었다. 이걸 그대로 복사하지 않고, 관절범위 클리핑/샘플링만 `RightArmSpace`
(플래닝 패키지 전역에서 쓰는 관절공간 추상화)로 위임하도록 다시 짜서
`planning/goals.py`에 프로덕션 API로 옮겼다 — `RightArmSpace.sample()`이
"unlimited 관절은 0" 관례를 갖고 있다는 걸 STATE.md 이전 학습에서 미리 알고
있었지만, 오른팔 7관절은 전부 범위가 있어(`limited=True`) 문제가 되지 않음을
확인했다.

## What worked / what failed

- `JointSpaceKinematics`(`kinematics/joint_space.py`)가 이미 "오프라인 FK/Jacobian
  체크용" 프로덕션 어댑터로 존재해, 새 FK 코드 없이 그대로 재사용 가능했다.
- 실제 can-sort 장면(`full_scene.xml`, site `grasp_target_r`)에서 근접 시드
  단일 수렴, 원거리 시드(START_Q→GOAL_Q pose) multistart 수렴, 도달 불가능한
  목표(x=10)에서 `converged=False`+best-effort 해가 관절범위 안에 남는지까지
  3개 신규 테스트로 검증 — 전부 통과, 실패한 시도 없음.
- `RightArmSpace`를 재사용하니 별도 관절범위 배열 재구성 코드가 필요 없어
  `tests/offline_pose_ik.py` 대비 오히려 짧아졌다.

## North-star delta

P4(Cartesian goal + 벤치마크)는 이번 cycle 전까지 완전 미착수였다. 이제
pose→q_goal 변환이 프로덕션 API로 존재하므로, 다음 조각(`MP-0012` 중복 제거,
이후 `MP-0014` 20-seed 성공률 측정)이 막힘 없이 이어질 수 있다.

## Key learnings

- `tests/offline_pose_ik.py`의 "test-only" 경고는 문자 그대로였다 — 실제로
  프로덕션 플래닝 코드에서 재사용 가능한 로직이었지만 import 경계상 테스트
  트리에만 있었다. 이런 "테스트에 갇힌 프로덕션 로직"은 앞으로도 눈여겨볼
  가치가 있다(다음엔 `MP-0012`가 반대 방향 — 테스트 쪽을 이 모듈로 위임).
- `RightArmSpace.clip`/`sample`을 재사용하면 IK 헬퍼가 플래너의 나머지
  부분(EdgeChecker, RRT 등)과 같은 관절범위 규칙을 자동으로 따르게 된다 —
  별도 배열을 만들면 나중에 두 규칙이 갈라질 위험이 있었다.

## Recommended next 1–3 priorities

1. `MP-0012` — `tests/offline_pose_ik.py`를 `planning.goals`로 위임(중복 제거).
2. `MP-0014` — pose goal 20 seed 성공률 측정(사람 확인 필요, `UserTest=☑`).
3. PR #13(MP-0007/MP-0017 벤치마크)·#14(이번 PR) 사람 리뷰/병합.

## Artifacts

- PR: https://github.com/Geonhee-LEE/aiworker-mj-lab/pull/14
- Files touched: `src/ffw_sh5_grasp/planning/goals.py` (신규), `src/ffw_sh5_grasp/planning/__init__.py`, `tests/test_planning_goals.py` (신규), `results/p4-cartesian-pose-goal-ik-seed.tsv` (신규)
- TSV row appended: yes
