# `control.bimanual`

양손 사이의 상대 pose를 유지하는 순수 기구학 task를 만든다.

## `capture_reference(right, left)`

- **기능:** 오른손 frame에서 본 왼손 위치와 회전을 저장한다.
- **입력:** 오른손·왼손 `SiteKinematics`.
- **반환:** 상대 위치와 회전행렬 mapping.

## `rigid_grasp_task(reference, site_states, dt, max_linear_speed, max_angular_speed)`

- **기능:** 저장된 양손 관계의 오차를 줄이는 상대 속도 task를 만든다.
- **입력:** reference, `{"r": state, "l": state}`, frame 간격, 속도 제한.
- **반환:** `(relative_jacobian, correction_velocity)`, shape `(6,N)`과 `(6,)`.
- **부작용:** 없음.
- **공통 규칙:** drift pose 오차와 제한 속도는
  [`kinematics.tasks`](kinematics-tasks.md)의 `pose_error()`와
  `pose_velocity_command()`를 사용한다.

[전신 IK API](control-whole-body.md)는 반환 task에 가중치를 붙여 전체 문제에 추가한다.
