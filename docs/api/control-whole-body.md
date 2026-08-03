# `control.whole_body`

Base x/y/yaw, lift와 양팔 14축을 하나의 bounded differential IK로 조립한다.

## `WholeBodyIK(model, site_names, arm_joint_names, **weights)`

- **입력:** `MjModel`, 손별 site·관절 이름, 선택적 task/CBF 가중치와 gain.
- **상태:** 공유 `KinematicTree`, 속도 제한, base·손 reference, rigid-grasp reference.
- **설정:** `config/default.yaml`의 `whole_body_ik`.

## `WholeBodyIK.solve(...)`

```python
solve(
    data, target_poses, dt,
    *, active_sides=("r", "l"), arm_nominal=None,
    lift_nominal=None, rigid_grasp=False,
    whole_body_enabled=True,
)
```

- **기능:** 현재 상태와 world 손 목표에서 다음 frame 명령을 계산한다.
- **반환:** `WholeBodyCommand`.
- **OFF:** `whole_body_enabled=False`이면 base·lift 속도를 0으로 hard pin한다.
- **부작용:** actuator와 live `data.qpos`를 쓰지 않는다.
- **공통 규칙:** 손 pose 오차와 목표 twist는
  [`kinematics.tasks`](kinematics-tasks.md)의 공통 함수로 만든다.

## `WholeBodyCommand`

| 필드 | 의미 |
|---|---|
| `base_twist` | 스워브 제어기에 전달할 차체 좌표 속도 |
| `arm_positions` | 손별 다음 목표 관절각 |
| `lift_position` | 다음 리프트 목표 위치 |
| `position_errors`, `orientation_errors` | 손별 pose 오차 norm |
| `generalized_velocity` | `[base(3), lift, right(7), left(7)]` 해 |
| `minimum_collision_distance` | 활성 pair의 최소 signed distance |
| `active_collision_pairs` | 이번 frame의 CBF 입력 |
| `collision_constraint_violation` | 최대 soft-barrier 위반량 |

## 상태와 진단 메서드

### `WholeBodyIK.rebase(data, target_poses=None)`

현재 상태를 새 출발점으로 저장하고 이전 base 속도·solve 시간을 초기화한다.

### `WholeBodyIK.set_rigid_grasp(data, active)`

현재 양손 상대 pose를 저장하거나 기존 rigid-grasp reference를 해제한다.

### `WholeBodyIK.site_state(data, side, current_q=None)`

WBIK와 같은 관절 열 순서의 한 손 `SiteKinematics`를 반환한다.

### `WholeBodyIK.collision_distances(data, max_distance=None)`

CBF와 같은 pair 정의를 사용해 가까운 `CollisionConstraint` tuple을 반환한다.

수식과 task 조립은 [전신 IK와 충돌 회피](../guide/whole_body_ik.md)를 참고한다.
