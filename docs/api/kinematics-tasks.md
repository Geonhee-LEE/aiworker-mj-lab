# `kinematics.tasks`

단일 팔, 전신, 양손 상대 자세 경로가 같은 pose 오차 부호·좌표계와 Cartesian 속도
제한 규칙을 사용하도록 만든 순수 계산 모듈이다. MuJoCo 모델이나 live 상태를 직접
읽거나 수정하지 않는다.

## `PoseError`

| 필드 | 의미 |
|---|---|
| `position` | `target_position - current_position`, world frame 3-vector, m |
| `orientation` | `target * inverse(current)`의 최단 축각, world frame 3-vector, rad |
| `position_norm` | 위치 오차 크기, m |
| `orientation_norm` | 회전 오차 크기, rad |

## `pose_error(current_position, current_quaternion, target_position, target_quaternion)`

- **기능:** 현재 pose에서 목표 pose까지의 공통 `PoseError`를 계산한다.
- **Quaternion:** MuJoCo와 같은 `(w, x, y, z)` 순서이며 `q`와 `-q`를 같은 회전으로 처리한다.
- **반환:** `PoseError`.
- **부작용:** 없음. 입력 배열을 수정하지 않는다.

## `pose_velocity_command(error, **options)`

```python
pose_velocity_command(
    error,
    position_gain=...,
    orientation_gain=...,
    current_twist=None,
    linear_velocity_damping=0.0,
    angular_velocity_damping=0.0,
    max_linear_speed=np.inf,
    max_angular_speed=np.inf,
)
```

- **기능:** `gain * error - damping * current_twist`를 계산하고 선속도·각속도 norm을 각각 제한한다.
- **입력 twist 순서:** `[linear_xyz, angular_xyz]`, world frame.
- **반환:** world-frame 6차원 목표 twist.
- **사용:** `WholeBodyIK`의 손 pose task와 `rigid_grasp_task()`의 상대 drift 보정.

단일 팔 `KinematicsSolver.solve_pose()`도 `pose_error()`를 사용하지만 결과가 속도가 아닌
관절각이어야 하므로 `pose_velocity_command()` 대신 위치 우선 DLS와 backtracking을
유지한다. 세 경로의 역할 차이는 [전신 IK와 충돌 회피](../guide/whole_body_ik.md#solver-comparison)를
참고한다.
