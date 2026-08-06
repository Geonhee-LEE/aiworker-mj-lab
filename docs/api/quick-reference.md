# 빠른 API 찾기

```python
from ffw_sh5_grasp.control.whole_body import WholeBodyIK
from ffw_sh5_grasp.kinematics import KinematicTree
from ffw_sh5_grasp.kinematics.solver import DifferentialIKSolver
```

| 하고 싶은 일 | API | 상세 문서 |
|---|---|---|
| 텔레옵 앱 실행 | `application.teleop.main()` | [앱](application-teleop.md) |
| 설정 읽기 | `load_settings(path)` | [설정](config.md) |
| Site FK/Jacobian | `KinematicTree.forward_site()` | [Tree](kinematics-tree.md) |
| pose 오차/속도 task | `pose_error()`, `velocity_task()` | [Task](kinematics-tasks.md) |
| joint/collision 제약 | `joint_velocity_bounds()`, `collision_velocity_barriers()` | [Constraint](kinematics-constraints.md) |
| pinv/DLS/QP 계산 | `DifferentialIKSolver.solve()` | [Solver](kinematics-solver.md) |
| 전신/arm-only 명령 | `WholeBodyIK.solve()` | [전신 IK](control-whole-body.md) |
| 차체 속도→바퀴 | `SwerveDrive.update_twist()` | [베이스](control-base.md) |
| 팔 목표각→토크 | `ArmTorqueController.apply()` | [팔](control-arm.md) |
| 손가락 명령 | `grasp.apply_grasp()` | [파지](control-grasp.md) |

## 전신 IK

```python
solver = WholeBodyIK(model, site_names, arm_joint_names, solver_method="qp")
solver.set_solver_method("dls")
command = solver.solve(data, target_poses, dt, active_sides=("r", "l"))
```

`WholeBodyIK.solve()`는 live `qpos`와 actuator를 직접 바꾸지 않는다. 반환된
`WholeBodyCommand`를 application 계층이 base, lift, arm actuator controller로 전달한다.

## QP 가중치

```python
weights = solver.qp_weights()
solver.set_qp_weight("damping_arm", 0.5)
solver.set_qp_weight("collision_slack", 1000.0)
```

모든 task strength는 대응 velocity scale로 정규화된 무차원 값이다.
