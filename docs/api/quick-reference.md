# 빠른 API 찾기

자주 쓰는 공개 함수와 객체만 기능별로 모은 색인이다. 자세한 입력 shape, 좌표계,
부작용과 예외는 오른쪽의 상세 문서에서 확인한다.

## 가장 자주 쓰는 진입점

```python
from ffw_sh5_grasp.config import load_settings
from ffw_sh5_grasp.kinematics import KinematicTree, KinematicsSolver
from ffw_sh5_grasp.control.arm import ArmTorqueController
from ffw_sh5_grasp.control.base import BodyTwist, SwerveDrive
from ffw_sh5_grasp.control.whole_body import WholeBodyIK
```

| 하고 싶은 일 | 첫 호출 | 핵심 결과 | 상세 문서 |
|---|---|---|---|
| 텔레옵 앱 실행 | `application.teleop.main()` | frame loop 시작 | [텔레옵 앱](application-teleop.md) |
| 사용자 YAML 읽기 | `load_settings(path)` | 검증된 `Settings` | [설정](config.md) |
| Site FK/Jacobian | `KinematicTree.forward_site()` | `SiteKinematics` | [Tree와 FK](kinematics-tree.md) |
| 단일 팔 pose IK | `KinematicsSolver.solve_pose()` | 관절각과 최종 오차 | [단일 팔 Solver](kinematics-solver.md) |
| 전신 또는 arm-only IK | `WholeBodyIK.solve()` | `WholeBodyCommand` | [전신 IK](control-whole-body.md) |
| 차체 속도를 바퀴 명령으로 변환 | `SwerveDrive.update_twist()` | 모듈별 조향각·회전속도 | [모바일 베이스](control-base.md) |
| 팔 목표각을 토크로 적용 | `ArmTorqueController.apply()` | `data.ctrl` 변경 | [팔 토크](control-arm.md) |
| 손가락 닫기 | `grasp.apply_grasp()` | 손가락 actuator target | [손 파지](control-grasp.md) |
| 렌더 frame 실행 | `begin_frame()` → `render_scene()` → `end_frame()` | 화면과 입력 갱신 | [렌더링](visualization-render.md) |

## 설정

| API | 기능 | 반환 |
|---|---|---|
| `load_settings(path=None)` | 기본 YAML에 사용자 YAML을 병합하고 검증 | `Settings` |
| `Settings.get(path)` | 임의 설정값을 복사해 읽기 | 설정값 |
| `Settings.number(path, ...)` | 실수와 범위 검증 | `float` |
| `Settings.integer(path, ...)` | 정수와 최솟값 검증 | `int` |

[설정 API 전체 보기](config.md)

## 목표와 애플리케이션

| API | 기능 | 반환 또는 변경 |
|---|---|---|
| `TeleopApp.set_arm_mode(side, mode)` | 한 팔의 IK/FK 제어권 전환 | 앱 mode·target |
| `TeleopApp.set_whole_body_enabled(enabled)` | world 목표를 보존하며 전신/arm-only 전환 | solver와 target 기준 |
| `target_world_pose(app, side)` | UI 손 목표를 solver용 world pose로 변환 | `(position, quaternion)` |
| `world_to_target_pos(app, side, position)` | world 위치를 UI target 값으로 역변환 | 3-element list |
| `capture_grasp(app)` | 가상 물체 기준 양손 상대 pose 저장 | capture 상태 |
| `apply_virtual_object_target(app)` | 가상 물체 pose를 두 손 목표로 전개 | 양손 target |

[목표 좌표 API 전체 보기](application-targets.md)

## 기구학과 회전

| API | 기능 | 반환 |
|---|---|---|
| `rpy_deg_to_quat(rpy_deg)` | UI RPY를 MuJoCo quaternion으로 변환 | `wxyz` 4-vector |
| `quat_to_rpy_deg(quaternion)` | Quaternion을 UI RPY로 변환 | degree 3-vector |
| `shortest_orientation_error(target, current)` | 최단 world-frame 자세 오차 | axis-angle 3-vector |
| `pose_error(current_pos, current_quat, target_pos, target_quat)` | 모든 IK가 공유하는 pose 오차 | `PoseError` |
| `pose_velocity_command(error, ...)` | 오차 피드백과 norm 제한으로 목표 twist 생성 | world-frame 6-vector |
| `KinematicTree.forward_site(qpos, site_id, joint_ids)` | Tree FK와 geometric Jacobian 계산 | `SiteKinematics` |
| `KinematicTree.point_jacobian(...)` | Body 위 한 점의 선속도 Jacobian 계산 | `(3, N)` 행렬 |
| `KinematicsSolver.solve_pose(...)` | 위치 우선 DLS 단일 팔 IK | 해와 위치·자세 오차 |
| `collision_distance_gradient(...)` | signed distance와 관절 gradient 계산 | `CollisionConstraint` 또는 `None` |

회전 함수는 `ffw_sh5_grasp.kinematics.rotations`, pose task 함수는
`ffw_sh5_grasp.kinematics.tasks`에서 가져온다.
[기구학 API 전체 안내](kinematics.md)

## 제어

| API | 기능 | 반환 또는 변경 |
|---|---|---|
| `WholeBodyIK.solve(...)` | base·lift·양팔 bounded differential IK | `WholeBodyCommand` |
| `WholeBodyIK.set_rigid_grasp(data, active)` | 양손 상대 pose 기준 설정·해제 | solver 상태 |
| `bounded_least_squares(A, b, lower, upper)` | Box-constrained least squares | 제한된 해 벡터 |
| `ArmTorqueController.apply(data, q_desired)` | Bias 보상 PD 팔 토크 기록 | `data.ctrl` |
| `BodyTwist(vx, vy, wz)` | 차체 좌표 속도 명령 표현 | 불변 명령 객체 |
| `SwerveDrive.update_twist(twist, dt, ...)` | 차체 속도를 모듈 명령으로 변환 | `{wheel: (steer, speed)}` |
| `apply_grasp(model, data, grasp, thumb, side="r")` | 손가락 synergy 적용 | `data.ctrl` |
| `is_grasped(model, data, ...)` | 실제 접촉력 기반 파지 판정 | `bool` |

[제어 API 전체 안내](control.md)

## 시각화

| API | 기능 | 반환 또는 변경 |
|---|---|---|
| `setup_render(app, width, height)` | GLFW·ImGui·MuJoCo renderer 준비 | 앱 렌더 상태 |
| `begin_frame(app)` | OS 입력을 받고 ImGui frame 시작 | ImGui IO |
| `draw_panel(app)` | Control Center와 Diagnostics 렌더 | 앱 UI 상태 |
| `pose_to_imguizmo_matrix(position, quaternion)` | world pose를 Gizmo 행렬로 변환 | `Matrix16` |
| `render_scene(app)` | marker·충돌 overlay·UI를 framebuffer에 렌더 | 화면 상태 |
| `shutdown(app)` | 렌더 backend와 창 정리 | 없음 |

[시각화 API 전체 안내](visualization.md)

## 대표 호출 흐름

### 단일 팔 IK

```python
solver = KinematicsSolver.from_mjcf(
    model_path,
    site_name="grasp_target_r",
    joint_names=arm_joint_names,
)
q_solution, position_error, orientation_error = solver.solve_pose(
    q_initial,
    target_position,
    target_quaternion,
)
```

### 전신 IK

```python
solver = WholeBodyIK(model, site_names, arm_joint_names)
command = solver.solve(
    data,
    target_poses,
    dt,
    active_sides=("r", "l"),
    whole_body_enabled=True,
)

base_twist = command.base_twist
right_arm_target = command.arm_positions["r"]
```

`WholeBodyIK.solve()`는 live `qpos`나 actuator를 직접 바꾸지 않는다. 반환된 명령이
실제 actuator까지 전달되는 순서는 [전신 IK API](control-whole-body.md)와
[앱 frame 흐름](../guide/teleop_app.md)에서 확인한다.
