# `control.arm`

팔 목표 관절각을 MuJoCo motor torque로 변환한다.

## `ArmTorqueController(model, joint_names, kp=..., kd=...)`

- **기능:** 관절 이름을 motor와 연결하고 qpos/qvel/DOF/actuator 주소를 캐시한다.
- **입력:** `MjModel`, 순서가 고정된 팔 관절 이름, PD gain.
- **오류:** motor가 없는 관절이면 `ValueError`.

## `ArmTorqueController.apply(data, q_des, kp_scale=1.0)`

- **기능:** bias force에 목표 오차 PD를 더해 torque를 계산한다.
- **입력:** `MjData`, controller 관절 순서의 목표각, 선택적 P gain 배율.
- **반환:** clipping 전 torque 벡터.
- **변경:** actuator 범위로 제한한 값을 해당 팔 `data.ctrl`에 기록.
- **주의:** `data.qpos`는 읽기만 한다.

식 `τ = qfrc_bias + Kp(q_des-q) - Kd q̇`는
[팔 토크 제어](../guide/arm_control.md)를 참고한다.
