# `kinematics.solver`

이름 기반 관절 벡터를 모델 전체 `qpos`에 끼워 넣고, 한 site의 FK와 반복 DLS IK를
제공한다.

## `KinematicsSolver`

### `KinematicsSolver.from_mjcf(path, site_name, joint_names, **kwargs)`

- **기능:** MJCF 로드부터 solver 생성까지 한 번에 수행한다.
- **입력:** XML 경로, 끝단 site 이름, 순서가 고정된 관절 이름, 선택적 damping 등.
- **반환:** 초기화된 `KinematicsSolver`.
- **오류:** site/joint 누락 또는 지원하지 않는 관절 구성.

### `KinematicsSolver.forward(q, context_qpos=None)`

- **기능:** 선택한 관절각의 world pose와 Jacobian을 계산한다.
- **입력:** solver 관절 순서의 `q`, 선택적 전체 모델 `context_qpos`.
- **반환:** `SiteKinematics`.
- **주의:** 전달 배열과 live 물리 상태를 수정하지 않는다.

### `KinematicsSolver.forward_kinematics(q, context_qpos=None)`

- **기능:** Jacobian이 필요 없는 기존 호출을 위한 호환 메서드.
- **반환:** `(world_position, world_quaternion)`.

### `KinematicsSolver.solve_pose(...)`

```python
solve_pose(
    q_init, target_pos, target_quat,
    max_iter=..., pos_tol=..., ori_tol=..., ori_weight=...,
    context_qpos=None,
)
```

- **기능:** 위치 우선 DLS step을 반복해 한 초기 자세에서 목표 pose로 수렴한다.
- **반환:** `(q_solution, position_error_norm, orientation_error_norm)`.
- **종료:** 두 오차가 tolerance 안이거나 `max_iter` 도달.
- **공통 규칙:** 위치·자세 오차는 [`kinematics.tasks`](kinematics-tasks.md)의
  `pose_error()`를 사용한다.

### `KinematicsSolver.solve_pose_multistart(...)`

- **기능:** 여러 초기 자세에서 다시 풀어 가장 좋은 해를 선택한다.
- **추가 입력:** 난수 생성기, 재시작 횟수, 성공 tolerance.
- **반환:** `(q, position_error, orientation_error, success)`.
- **사용:** offline 목표 생성이나 큰 pose 점프.

`InverseKinematics`는 `kinematics.legacy`에 남긴 호환 별칭이다. DLS 유도는
[DLS와 위치 우선 IK](../guide/ik-math.md), 반복 절차는
[단일 팔 IK](../guide/ik.md)를 참고한다.
