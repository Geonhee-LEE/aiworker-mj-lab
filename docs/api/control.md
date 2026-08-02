# 제어 API

제어 계층은 기구학 결과를 매 frame의 generalized velocity, 관절 목표, 바퀴 명령,
actuator 입력으로 바꾼다. `WholeBodyIK`는 명령을 **반환**하고, 팔·손 controller는
`data.ctrl`에 실제 값을 **기록**한다는 경계를 먼저 구분한다.

## 전신 IK

### `WholeBodyIK(model, site_names, arm_joint_names, **weights)`

**직관:** base x/y/yaw, lift, 오른팔 7축, 왼팔 7축을 하나의 속도 벡터로 놓고 손
목표·관절 한계·충돌 안전을 함께 만족시키는 frame별 문제를 준비한다.

- **입력:** `MjModel`, 손별 site 이름, 손별 순서가 고정된 관절 이름, 선택적 task/CBF
  가중치와 gain.
- **상태:** 공유 `KinematicTree`, 손별 solver, 속도 제한, base reference, rigid-grasp
  reference.
- **설정:** 기본값은 `config/default.yaml`의 `whole_body_ik`에서 온다.

### `WholeBodyIK.solve(...)`

```python
solve(
    data, target_poses, dt,
    *, active_sides=("r", "l"), arm_nominal=None,
    lift_nominal=None, rigid_grasp=False,
    whole_body_enabled=True,
)
```

- **직관:** 현재 로봇 상태와 양손 목표를 보고 다음 frame에 보낼 base·lift·팔 명령
  한 묶음을 계산한다.
- **입력:** 현재 `MjData`, 손별 `(world_position, world_quaternion)`, frame 간격,
  활성 IK 팔, 자세 복원 기준, rigid-grasp/Whole-body mode.
- **반환:** `WholeBodyCommand`.
- **OFF 동작:** `whole_body_enabled=False`이면 base x/y/yaw와 lift velocity bound를
  0으로 고정한다. 팔의 pose task, 관절 한계와 충돌 CBF는 그대로 동작한다.
- **부작용:** actuator나 live `data.qpos`를 쓰지 않는다.

### `WholeBodyCommand`

| 필드 | 직관적인 의미 |
|---|---|
| `base_twist` | 스워브 제어기로 보낼 차체 좌표 속도 |
| `arm_positions` | 손별 다음 목표 관절각 |
| `lift_position` | 다음 리프트 목표 위치 |
| `position_errors` / `orientation_errors` | UI·진단용 손별 오차 norm |
| `generalized_velocity` | `[base(3), lift, right(7), left(7)]` 순서의 해 |
| `minimum_collision_distance` | 활성 감시 쌍 중 최소 signed distance |
| `active_collision_pairs` | 이번 frame의 CBF 입력 |
| `collision_constraint_violation` | soft barrier를 만족하지 못한 최대량 |

### `WholeBodyIK.rebase(data, target_poses=None)`

- **직관:** 수동 주행이나 mode 전환 직후 현재 상태를 새 출발점으로 삼아 이전 자동
  명령으로 되돌아가는 것을 막는다.
- **입력:** 현재 `MjData`, 선택적 world 손 목표.
- **변경:** base/손 reference, 이전 base 속도와 solve 시간.
- **반환:** 없음.

### `WholeBodyIK.set_rigid_grasp(data, active)`

- **직관:** 켤 때 현재 두 손의 상대 pose를 사진처럼 저장하고, 끌 때 그 제약을 버린다.
- **입력:** 현재 상태와 boolean.
- **변경:** 내부 rigid-grasp reference.

### `WholeBodyIK.site_state(data, side, current_q=None)`

- **직관:** WBIK와 정확히 같은 관절 순서로 한 손의 현재 pose/Jacobian을 읽는다.
- **입력:** `MjData`, 손, 선택적 18축 관절 벡터.
- **반환:** `SiteKinematics`.
- **사용 시점:** target 초기화, rigid-grasp 캡처, solver 진단.

### `WholeBodyIK.collision_distances(data, max_distance=None)`

- **직관:** 현재 CBF가 실제로 보고 있는 가까운 충돌 쌍만 다시 계산한다.
- **입력:** `MjData`, 선택적 최대 검색 거리.
- **반환:** `CollisionConstraint` tuple.
- **사용 시점:** 충돌 overlay와 상태 표시. 제어와 같은 pair 정의를 공유한다.

수식과 task 조립 순서는 [전신 IK와 충돌 회피](../guide/whole_body_ik.md)에 있다.

## 순수 최적화

### `bounded_least_squares(matrix, vector, lower, upper)`

- **직관:** `Ax ≈ b`를 가장 잘 맞추되 각 변수는 지정된 최솟값과 최댓값 밖으로
  절대 나가지 않게 푼다.
- **입력:** `A` shape `(M,N)`, `b` shape `(M,)`, lower/upper shape `(N,)`.
- **반환:** box 안의 해 `x`, shape `(N,)`.
- **오류:** 행렬·벡터·bound shape 불일치 또는 `lower > upper`.
- **사용 시점:** 속도 제한과 hard-fixed DOF가 있는 least-squares.

### `bounded_least_squares_with_barriers(...)`

```python
bounded_least_squares_with_barriers(
    A, b, lower, upper,
    barrier_matrix, barrier_lower, slack_weight,
)
```

- **직관:** box 제한에 더해 `Gx ≥ h` 안전선을 가능한 한 지키고, 불가능할 때만
  비용이 큰 slack으로 조금 위반한다.
- **입력:** 기본 least-squares와 barrier `G`, 하한 `h`, 양의 slack weight.
- **반환:** box를 항상 만족하고 활성 barrier 위반을 벌점 처리한 해.
- **사용 시점:** collision CBF처럼 hard infeasibility보다 작은 진단 가능한 위반이
  안전한 제약.

두 함수는 robot이나 MuJoCo를 모르는 순수 NumPy 함수이므로 독립 수치 테스트에도 쓸
수 있다.

## 양손 상대 pose

### `capture_reference(right, left)`

- **직관:** 오른손에서 보았을 때 왼손이 어디 있고 어떻게 돌아가 있는지 저장한다.
- **입력:** 오른손·왼손 `SiteKinematics`.
- **반환:** 오른손 frame의 상대 위치와 회전행렬을 담은 mapping.

### `rigid_grasp_task(reference, site_states, dt, max_linear_speed, max_angular_speed)`

- **직관:** 저장한 양손 간격이 흐트러졌을 때 두 손을 다시 맞추는 상대 속도 task를
  만든다.
- **입력:** 캡처 reference, `{"r": state, "l": state}`, frame 간격, 보정 속도 제한.
- **반환:** `(relative_jacobian, correction_velocity)`; shape은 `(6,N)`, `(6,)`.
- **부작용:** 없음. WBIK가 반환 task에 가중치를 붙여 전체 문제에 추가한다.

## 팔 토크 { #arm-torque-api }

### `ArmTorqueController(model, joint_names, kp=..., kd=...)`

- **직관:** 관절 이름을 실제 motor와 연결하고, 팔이 자체 무게에 처지지 않도록 토크
  계산에 필요한 주소·제한을 미리 모은다.
- **입력:** `MjModel`, 순서가 고정된 팔 관절 이름, PD gain.
- **오류:** motor가 없는 관절을 요청하면 `ValueError`.

### `ArmTorqueController.apply(data, q_des, kp_scale=1.0)`

- **직관:** 현재 자세를 버티는 bias force에 목표 오차 PD를 더해 이번 physics step의
  토크를 만든다.
- **입력:** 현재 `MjData`, controller 관절 순서의 목표각, 선택적 P gain 배율.
- **반환:** clipping 전 torque 벡터.
- **변경:** actuator range로 clipping한 값을 해당 팔의 `data.ctrl`에 기록한다.
- **주의:** `data.qpos`는 읽기만 한다.

수식 `τ = qfrc_bias + Kp(q_des-q) - Kd q̇`의 항별 의미는
[팔 토크 제어](../guide/arm_control.md)에 있다.

## 이동 베이스

### `BodyTwist(vx=0, vy=0, wz=0)`

차체 좌표계의 평면 속도 값 객체다. `vx`, `vy`는 m/s, `wz`는 rad/s다.

#### `BodyTwist.is_zero()`

- **직관:** 센서·수치 잡음 수준을 제외하면 차체가 정지 명령인지 판정한다.
- **반환:** 설정된 선속도·각속도 deadband 안이면 `True`.

### `BaseTeleop`

#### `BaseTeleop.update_body(keys, dt, measured_twist=None)`

- **직관:** WASD와 좌우 화살표를 갑자기 튀지 않는 차체 속도로 바꾼다.
- **입력:** key boolean mapping, frame 간격. `measured_twist`는 기존 호출 호환 인자다.
- **반환:** smoothed `BodyTwist`.

#### `BaseTeleop.update(keys, dt, yaw=0.0)`

- **직관:** 예전 호출부를 위해 body-frame 병진 명령만 world x/y로 회전한다.
- **반환:** `(world_vx, world_vy, wz)` tuple.
- **권장:** 새 스워브 경로에서는 `update_body()`를 사용한다.

#### `BaseTeleop.reset_motion()`

- **직관:** 물리적으로 정지한 뒤 남아 있는 입력 smoothing 속도를 0으로 지운다.

### `SwerveKinematics`

#### `SwerveKinematics.inverse(twist, steering_positions=None, preferred_directions=None)`

- **직관:** 원하는 차체 이동을 각 wheel module이 향할 각도와 굴러갈 속도로 분해한다.
- **입력:** `BodyTwist`, 현재 조향각, 이전 구동 방향.
- **반환:** `({wheel: (steer_rad, drive_rad_s)}, saturation_scale)`.
- **특징:** `angle + kπ`와 반대 구동을 함께 탐색해 조향 이동이 짧은 실행 가능 상태를
  고르고, 포화 시 모든 wheel 속도를 같은 비율로 줄인다.

#### `SwerveKinematics.forward(steering_positions, wheel_velocities)`

- **직관:** 실제 wheel 각도·속도로 차체가 어떻게 움직이고 있는지 역으로 추정한다.
- **입력:** wheel별 조향각과 회전속도 mapping.
- **반환:** 최소제곱 추정 `BodyTwist`.

### `SwerveDrive`

#### `SwerveDrive.update_twist(twist, dt, steering_positions=None, wheel_velocities=None)`

- **직관:** 수동 또는 WBIK의 차체 속도를 조향 정렬·방향 반전·가감속까지 고려한
  실제 actuator 목표로 바꾼다.
- **입력:** `BodyTwist` 또는 `(vx,vy,wz)`, frame 간격, 선택적 wheel feedback.
- **반환:** `{wheel: (steer_angle, drive_angular_velocity)}`.
- **안전 동작:** 모든 module이 정렬되기 전에는 drive speed를 0으로 둔다.

#### `SwerveDrive.update(keys, dt, yaw=0.0, steering_positions=None, wheel_velocities=None)`

- **직관:** 키보드 smoothing과 `update_twist()`를 연달아 호출하는 호환 경로다.
- **권장:** 앱처럼 수동/WBIK 우선순위를 외부에서 정하면 `update_twist()`를 사용한다.

`ReversalPhase`는 wheel 방향 전환 상태인 `NORMAL → DECELERATING → STEERING →
ACCELERATING`을 표현한다. 스워브 기하와 FSM은
[모바일 스워브 제어](../guide/base_teleop.md)에 있다.

## 손 파지

### `apply_grasp(model, data, grasp, thumb, side="r")`

- **직관:** 두 개의 0~1 슬라이더를 여러 손가락 관절 목표로 동시에 펼친다.
- **입력:** model/data, 손가락 curl `grasp`, 엄지 curl `thumb`, `"r"` 또는 `"l"`.
- **반환:** 없음.
- **변경:** 해당 손가락 position actuator의 `data.ctrl`.
- **특징:** 입력은 `[0,1]`로 제한되고 model별 joint/actuator 계수는 캐싱된다.

### `get_finger_can_contacts(model, data, side="r")`

- **직관:** 캔을 실제로 누르는 손가락 그룹과 각 그룹의 법선 힘을 읽는다.
- **입력:** 현재 model/data와 손.
- **반환:** `{finger_group: summed_normal_force}` mapping, N.

### `is_grasped(...)`

```python
is_grasped(
    model, data, min_fingers=2, min_total_force=0.05,
    require_thumb=True, side="r",
)
```

- **직관:** 목표 슬라이더가 아니라 실제 접촉 그룹 수와 힘으로 파지 성공을 판정한다.
- **입력:** 최소 손가락 그룹 수, 최소 합력, 엄지 접촉 필요 여부, 손.
- **반환:** 조건을 모두 만족하면 boolean `True`.

Synergy affine 식과 좌우 mirror 관절 처리는
[손 파지와 접촉 판정](../guide/grasp.md)에 있다.
