# `kinematics.tasks`

MuJoCo 상태를 모르는 soft objective/residual 카탈로그다.

| API | 의미 |
|---|---|
| `PoseError` | world-frame 위치 m, 최단 회전 rad 오차 |
| `pose_error(...)` | 현재 pose와 목표 pose의 공통 오차 계산 |
| `pose_velocity_command(...)` | gain/damping과 norm 제한으로 목표 twist 생성 |
| `normalized_weights(...)` | `strength / velocity_scale²` 계산 |
| `velocity_task(...)` | Jacobian task를 무차원 weighted residual로 변환 |
| `regularization_task(...)` | damping/posture identity-Jacobian task 생성 |
| `stack_velocity_tasks(...)` | 여러 `VelocityTask`를 solver 행렬/벡터로 결합 |

`VelocityTask`는 `||matrix @ qdot - target||²` 하나를 표현한다. hard bound와 CBF는
이 목록에 섞지 않고 [`kinematics.constraints`](kinematics-constraints.md)에 둔다.
