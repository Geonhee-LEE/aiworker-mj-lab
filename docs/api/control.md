# 제어 API

제어 모듈은 기구학 결과를 전신, 팔, 모바일 베이스와 손가락 명령으로 바꾼다.

<div class="grid cards" markdown>

-   **전신 IK** · `control.whole_body`

    손 목표에서 base·lift·양팔 명령을 계산한다. [바로 보기](#whole-body)

-   **팔과 양손** · `control.arm`, `control.bimanual`

    관절 목표를 torque로 바꾸고 양손 상대 pose task를 만든다. [바로 보기](#arm)

-   **모바일 베이스** · `control.base`

    `BodyTwist`를 세 스워브 모듈의 조향·구동 명령으로 바꾼다. [바로 보기](#base)

-   **손가락** · `control.grasp`

    synergy 명령을 적용하고 실제 접촉력으로 파지를 판정한다. [바로 보기](#grasp)

</div>

## 데이터 변경 여부

| API | 반환 | `data.ctrl` 변경 |
|---|---|:---:|
| `WholeBodyIK.solve()` | `WholeBodyCommand` | 아니요 |
| `rigid_grasp_task()` | 상대 Jacobian과 보정 속도 | 아니요 |
| `ArmTorqueController.apply()` | 제한 전 torque | 예 |
| `SwerveDrive.update_twist()` | wheel 명령 사전 | 아니요 |
| `apply_grasp()` | 없음 | 예 |

## 전신 IK { #whole-body }

### 생성

```python
solver = WholeBodyIK(
    model,
    site_names={"r": "grasp_target_r", "l": "grasp_target_l"},
    arm_joint_names={"r": right_joints, "l": left_joints},
)
```

`site_names`와 `arm_joint_names`는 `"r"`, `"l"` 키를 사용한다. 생성자는 joint/site
주소, `KinematicTree`, 제한값과 collision pair를 준비한다. 생략한 설정은
`config/default.yaml`의 `whole_body_ik` 값을 사용한다.

### 명령 계산

```python
command = solver.solve(
    data,
    target_poses,       # {side: (world_position, world_quaternion)}
    dt,
    active_sides=("r", "l"),
    arm_nominal=None,
    lift_nominal=None,
    rigid_grasp=False,
    whole_body_enabled=True,
)
```

`solve()`는 현재 상태를 읽고 `WholeBodyCommand`를 반환한다. actuator와 `data.qpos`는
변경하지 않는다. `whole_body_enabled=False`이면 base와 lift 속도를 0으로 고정한다.

??? note "`WholeBodyCommand` 반환 필드"

    | 필드 | 내용 |
    |---|---|
    | `base_twist` | `SwerveDrive`에 전달할 `BodyTwist` |
    | `arm_positions` | 손별 다음 관절 목표각 |
    | `lift_position` | 다음 lift 목표 위치 |
    | `position_errors`, `orientation_errors` | 손별 pose 오차 크기 |
    | `generalized_velocity` | `[base(3), lift, right(7), left(7)]` 속도 |
    | `minimum_collision_distance` | 활성 pair의 최소 signed distance |
    | `active_collision_pairs` | 이번 계산의 `CollisionConstraint` tuple |
    | `collision_constraint_violation` | soft barrier의 최대 속도 위반량 |

### 설정과 진단

| API | 반환 또는 변경 |
|---|---|
| `solver_method` | 현재 해법 이름 |
| `set_solver_method(method)` | Pseudoinverse, DLS 또는 QP 선택 |
| `set_dls_damping(value)` | 양수 DLS damping 설정 |
| `qp_weights()` | 편집 가능한 weight 사전의 복사본 |
| `set_qp_weight(name, value)` | 지정한 weight 변경 |
| `rebase(data, target_poses=None)` | 현재 상태를 기준으로 저장하고 base 속도 이력 초기화 |
| `set_rigid_grasp(data, active)` | 현재 양손 상대 pose 저장 또는 해제 |
| `site_state(data, side, current_q=None)` | 한 손의 `SiteKinematics` |
| `collision_distances(data, max_distance=None)` | 가까운 `CollisionConstraint` tuple |

수식과 계산 순서는 [전신 IK와 충돌 회피](../guide/whole_body_ik.md)에 있다.

## 양손 상대 pose { #bimanual }

`control.bimanual`의 두 함수는 입력 상태를 변경하지 않는다.

| 함수 | 반환 |
|---|---|
| `capture_reference(right, left)` | 오른손 기준 왼손 position/rotation 사전 |
| `rigid_grasp_task(reference, site_states, dt, max_linear_speed, max_angular_speed)` | `(6×N relative_jacobian, 6-vector correction_velocity)` |

입력 `right`, `left`와 `site_states`의 값은 `SiteKinematics`다.

## 팔 torque { #arm }

### `ArmTorqueController(model, joint_names, kp=..., kd=...)`

`joint_names` 순서대로 qpos·DOF·motor actuator 주소와 torque 범위를 저장한다. 연결된
actuator가 없으면 `ValueError`를 발생시킨다.

### `apply(data, q_des, kp_scale=1.0)`

`qpos`, `qvel`, `qfrc_bias`로 bias 보상 PD torque를 계산한다. 제한 전 torque 벡터를
반환하고, actuator 범위로 제한한 값을 `data.ctrl`에 기록한다.

[제어식과 적용 흐름](../guide/arm_control.md)

## 모바일 베이스 { #base }

### 차체 속도와 키 입력

| API | 반환 또는 변경 |
|---|---|
| `BodyTwist(vx=0, vy=0, wz=0)` | 차체 좌표계 평면 속도 |
| `BodyTwist.is_zero()` | 설정 deadband 안이면 `True` |
| `BaseTeleop.update_body(keys, dt, measured_twist=None)` | 평활화된 `BodyTwist` |
| `BaseTeleop.update(keys, dt, yaw=0.0)` | 호환용 월드 좌표계 `(vx, vy, wz)` tuple |
| `BaseTeleop.reset_motion()` | 입력 평활화 값 초기화 |

`measured_twist`는 기존 호출부를 위한 호환 인자이며 현재 계산에는 사용하지 않는다.

### 스워브 기구학과 제어

| API | 반환 |
|---|---|
| `SwerveKinematics.inverse(...)` | `({wheel: (steer, drive)}, saturation_scale)` |
| `SwerveKinematics.forward(steering_positions, wheel_velocities)` | 최소제곱 `BodyTwist` |
| `SwerveDrive.update_twist(...)` | 최종 `{wheel: (steer, drive)}` 명령 |
| `SwerveDrive.update(...)` | 키 입력과 `update_twist()`를 연결한 호환 경로 |

전체 시그니처:

```python
SwerveKinematics.inverse(
    twist, steering_positions=None, preferred_directions=None
)
SwerveDrive.update_twist(
    twist, dt, steering_positions=None, wheel_velocities=None
)
```

`ReversalPhase`는 `NORMAL`, `DECELERATING`, `STEERING`, `ACCELERATING` 상태를 갖는다.
[스워브 기구학과 정렬 gate](../guide/base_teleop.md)

## 손가락과 파지 판정 { #grasp }

`side`는 `"r"` 또는 `"l"`이다.

| 함수 | 반환 또는 변경 |
|---|---|
| `apply_grasp(model, data, grasp, thumb, side="r")` | `[0,1]` synergy를 손가락 `data.ctrl`에 기록 |
| `get_finger_can_contacts(model, data, side="r")` | `{finger_group: summed_normal_force}` 사전, 단위 N |
| `is_grasped(model, data, ..., side="r")` | 접촉 그룹 수와 힘 조건을 만족하면 `True` |

`is_grasped()`의 선택 항목은 `min_fingers`, `min_total_force`, `require_thumb`이며 생략하면
YAML 설정값을 사용한다. [손가락 매핑과 판정 조건](../guide/grasp.md)
