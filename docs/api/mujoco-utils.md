# `mujoco_utils`

여러 계층에서 공유하는 작은 MuJoCo 이름·actuator 조회 함수다.

## `find_actuator_for_joint(model, joint_id)`

- **기능:** 한 관절을 실제로 구동하는 actuator를 찾는다.
- **입력:** `MjModel`, joint id.
- **반환:** actuator id 또는 연결된 actuator가 없으면 `None`.
- **주의:** 반환값을 검사하지 않고 `data.ctrl[None]`에 쓰면 NumPy broadcasting으로
  전체 control 배열이 바뀔 수 있다.
