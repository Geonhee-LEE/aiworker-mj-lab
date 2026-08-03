# `control.base`

차체 좌표 속도 표현, 키보드 smoothing, 스워브 기구학과 방향 반전 FSM을 제공한다.

## `BodyTwist(vx=0, vy=0, wz=0)`

`vx`, `vy`는 m/s, `wz`는 rad/s인 차체 좌표 평면 속도 값 객체다.

### `BodyTwist.is_zero()`

선·각속도가 설정 deadband 안이면 `True`를 반환한다.

## `BaseTeleop`

| 메서드 | 기능 | 반환 |
|---|---|---|
| `BaseTeleop.update_body(keys, dt, measured_twist=None)` | 키 입력을 smoothed body 속도로 변환 | `BodyTwist` |
| `BaseTeleop.update(keys, dt, yaw=0.0)` | 호환용 world 병진 속도 변환 | `(vx, vy, wz)` |
| `BaseTeleop.reset_motion()` | 남아 있는 입력 smoothing 속도 제거 | 없음 |

새 스워브 경로는 `update_body()`를 사용한다.

## `SwerveKinematics`

### `SwerveKinematics.inverse(twist, steering_positions=None, preferred_directions=None)`

- **기능:** 차체 속도를 wheel별 조향각과 회전속도로 분해한다.
- **반환:** `({wheel: (steer_rad, drive_rad_s)}, saturation_scale)`.
- **특징:** `angle+kπ`와 반대 구동을 함께 탐색하고 포화 시 공통 비율로 줄인다.

### `SwerveKinematics.forward(steering_positions, wheel_velocities)`

실제 wheel 상태에서 최소제곱 body `BodyTwist`를 추정한다.

## `SwerveDrive`

### `SwerveDrive.update_twist(twist, dt, steering_positions=None, wheel_velocities=None)`

- **기능:** 조향 정렬·방향 반전·가감속을 고려한 actuator 목표를 만든다.
- **반환:** `{wheel: (steer_angle, drive_angular_velocity)}`.
- **안전:** 모든 module이 정렬되기 전 drive speed는 0이다.

### `SwerveDrive.update(keys, dt, yaw=0.0, steering_positions=None, wheel_velocities=None)`

키보드 smoothing과 `update_twist()`를 연결하는 호환 경로다.

`ReversalPhase`는 `NORMAL → DECELERATING → STEERING → ACCELERATING` 상태를
표현한다. 상세 기하는 [모바일 스워브 제어](../guide/base_teleop.md)를 참고한다.
