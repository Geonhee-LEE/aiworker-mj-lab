# API 치트시트

## 상황별 첫 함수

새 코드는 아래의 공개 진입점에서 시작한다. 이름이 `_`로 시작하는 함수는 앱 내부
구현 세부사항이므로, 테스트에서 특정 frame 단계를 검증하는 경우가 아니면 직접
호출하지 않는다.

### 앱과 target

| 하려는 일 | 파일과 함수 | 결과·다음 단계 | 상세 설명 |
|---|---|---|---|
| 앱 실행 | `teleop_app.main(argv=None)` | `TeleopApp().run()`으로 전체 루프 시작 | [앱 조립](teleop_app.md) |
| 전신/팔 전용 모드 전환 | `TeleopApp.set_whole_body_enabled(enabled)` | world target 보존, WBIK reference 재설정 | [전신 IK](whole_body_ik.md) |
| 손별 IK/FK 모드 전환 | `TeleopApp.set_arm_mode(side, mode)` | FK→IK일 때 현재 손 pose를 target으로 동기화 | [앱 조립](teleop_app.md) |
| 현재 손 목표를 solver 입력으로 변환 | `teleop_targets.target_world_pose(app, side)` | `(world_position, world_quaternion)` | [목표와 좌표 변환](teleop_targets.md) |
| world 위치를 UI target 값으로 역변환 | `teleop_targets.world_to_target_pos(app, side, world_pos)` | 현재 mode의 home-relative offset | [목표와 좌표 변환](teleop_targets.md) |
| RPY target을 world 자세로 변환 | `teleop_targets.target_rpy_to_world_quat(app, side, rpy_deg)` | 정규화된 world quaternion | [목표와 좌표 변환](teleop_targets.md) |
| 양손 상대 pose 캡처/해제 | `teleop_targets.capture_grasp(app)` / `release_grasp(app)` | Bimanual MoveL 상태 변경 | [목표와 좌표 변환](teleop_targets.md) |

### 기구학과 회전 수학

| 하려는 일 | 파일과 함수 | 결과·다음 단계 | 상세 설명 |
|---|---|---|---|
| quaternion 정규화 | `kinematics_math.normalize_quaternion(q)` | 단위 quaternion, 0-norm은 오류 | [Quaternion](quaternion-math.md) |
| 두 자세의 최단 world-frame 오차 | `kinematics_math.shortest_orientation_error(target, current)` | 3D axis-angle error | [Quaternion](quaternion-math.md) |
| quaternion↔회전행렬 | `rotation_from_quaternion(q)` / `quaternion_from_rotation(R)` | 3×3 행렬 또는 wxyz quaternion | [Quaternion](quaternion-math.md) |
| 모델 전체의 불변 tree 구성 | `kinematic_tree.KinematicTree(model)` | body/joint/site lookup과 `qpos0` cache | [Kinematic Tree](kinematic-tree.md) |
| raw qpos의 site FK/Jacobian | `KinematicTree.forward_site(qpos, site_id, joint_ids)` | `SiteKinematics(position, quaternion, jacobian)` | [FK와 Jacobian](forward-kinematics.md) |
| body 고정점의 선형 Jacobian | `KinematicTree.point_jacobian(qpos, body_id, point_world, joint_ids)` | 3×N world Jacobian | [Collision distance](collision-kinematics.md) |
| joint 이름 기반 site FK/Jacobian | `KinematicsSolver.forward(q, context_qpos=None)` | 정규화된 world pose와 6×N Jacobian | [기구학 전체 안내](kinematics.md) |
| MJCF에서 solver 바로 구성 | `KinematicsSolver.from_mjcf(path, site_name, joint_names)` | `KinematicsSolver` | [단일 팔 IK](ik.md) |
| 단일 초기값 IK | `KinematicsSolver.solve_pose(q_init, target_pos, target_quat)` | `(q, position_error, orientation_error)` | [DLS 수학](ik-math.md), [단일 팔 IK](ik.md) |
| local minimum 재시도 포함 IK | `KinematicsSolver.solve_pose_multistart(..., rng)` | 위 결과와 `success` boolean | [단일 팔 IK](ik.md) |
| geometry 거리와 미분 | `collision_kinematics.collision_distance_gradient(...)` | `CollisionConstraint` 또는 거리 밖이면 `None` | [Collision distance](collision-kinematics.md) |

`kinematics.py`는 기존 호출 호환을 위해 tree·collision 공개 이름을 다시 노출한다.
새 세부 구현은 책임이 드러나는 `kinematic_tree`, `kinematics_math`,
`collision_kinematics` 모듈에서 import한다.

### IK와 순수 최적화

| 하려는 일 | 파일과 함수 | 결과·다음 단계 | 상세 설명 |
|---|---|---|---|
| 한 frame의 전신 명령 계산 | `WholeBodyIK.solve(data, target_poses, dt, ...)` | `WholeBodyCommand`의 base twist, lift/arm 목표, 진단값 | [전신 IK](whole_body_ik.md) |
| 현재 상태를 WBIK 기준으로 재설정 | `WholeBodyIK.rebase(data, target_poses=None)` | 전환 직후 되돌림 방지 | [전신 IK](whole_body_ik.md) |
| 양손 rigid-grasp 제약 설정 | `WholeBodyIK.set_rigid_grasp(data, active)` | 상대 pose reference 캡처 또는 해제 | [전신 IK](whole_body_ik.md) |
| live 손 pose/Jacobian 조회 | `WholeBodyIK.site_state(data, side)` | `SiteKinematics` | [전신 IK](whole_body_ik.md) |
| 활성 충돌 거리 조회 | `WholeBodyIK.collision_distances(data)` | `CollisionConstraint` tuple | [Collision distance](collision-kinematics.md) |
| box-constrained least-squares | `bounded_optimization.bounded_least_squares(A, b, lower, upper)` | bound를 만족하는 해 벡터 | [전신 IK](whole_body_ik.md) |
| CBF soft barrier 포함 solve | `bounded_least_squares_with_barriers(...)` | box와 활성 barrier를 반영한 해 | [전신 IK](whole_body_ik.md) |
| 양손 상대 pose reference/task | `bimanual_kinematics.capture_reference(...)` / `rigid_grasp_task(...)` | reference 또는 `(Jacobian, correction_velocity)` | [전신 IK](whole_body_ik.md) |

### actuator와 이동 베이스

| 하려는 일 | 파일과 함수 | 결과·다음 단계 | 상세 설명 |
|---|---|---|---|
| 키 입력을 body twist로 만들기 | `BaseTeleop.update_body(keys, dt, measured_twist=None)` | `BodyTwist(vx, vy, wz)` | [모바일 스워브](base_teleop.md) |
| body twist↔wheel 상태 변환 | `SwerveKinematics.inverse(...)` / `forward(...)` | module 목표 또는 추정 body twist | [모바일 스워브](base_teleop.md) |
| feedback 포함 wheel 명령 | `SwerveDrive.update_twist(twist, dt, steering_positions, wheel_velocities)` | steer/drive command mapping | [모바일 스워브](base_teleop.md) |
| 팔 목표각을 torque로 적용 | `ArmTorqueController.apply(data, q_des, kp_scale=1.0)` | torque를 반환하고 arm `data.ctrl`에 기록 | [팔 토크 제어](arm_control.md) |
| 손가락 synergy 적용 | `grasp.apply_grasp(model, data, grasp, thumb, side)` | finger `data.ctrl`에 기록 | [손 파지](grasp.md) |
| 캔 접촉력 조회 | `grasp.get_finger_can_contacts(model, data, side)` | 손가락 그룹별 법선력 dict | [손 파지](grasp.md) |
| 파지 성공 판정 | `grasp.is_grasped(model, data, ..., side)` | 접촉 그룹·합력 조건 boolean | [손 파지](grasp.md) |

### 한 frame에서의 소유권

`TeleopApp.run()`이 frame loop를 소유하고, `_step_physics()`가 명령 우선순위를,
`_step_actuators()`가 유일한 actuator 기록과 `mujoco.mj_step()` 호출을 담당한다.
UI와 renderer를 확장할 때는 각각 `teleop_ui.draw_panel(app)`과
`teleop_render.render_scene(app)`에서 시작하되, actuator를 직접 쓰지 않고 app의
target/state만 변경한다. 전체 호출 순서는 [런타임 아키텍처](ros2/04-runtime-architecture.md)와
[앱 조립](teleop_app.md)에 한 번만 설명한다.

## 현재 기본값

| 설정 | 값 | 위치 |
|---|---:|---|
| UI loop | 25 Hz | `teleop_app.LOOP_HZ` |
| target position ramp | 0.03 m/frame | `MAX_POS_STEP_PER_FRAME` |
| target orientation ramp | 8°/frame | `MAX_RPY_STEP_PER_FRAME_DEG` |
| lift range | -0.5~0.0 m | `LIFT_RANGE` |
| collision buffer | 0.03 m | `WholeBodyIK` 기본값 |
| collision safe distance | 0.01 m | `WholeBodyIK` 기본값 |
| base linear velocity limit | 0.55 m/s | `DEFAULT_VELOCITY_LIMITS` |
| base yaw velocity limit | 1.2 rad/s | `DEFAULT_VELOCITY_LIMITS` |
| lift velocity limit | 0.25 m/s | `DEFAULT_VELOCITY_LIMITS` |
| arm velocity limit | 2.0 rad/s | `DEFAULT_VELOCITY_LIMITS` fallback |

## MuJoCo Python API

| API | 사용 위치 | 역할 |
|---|---|---|
| `MjModel.from_xml_path()` | `teleop_app.py`, tests | XML 모델 로드 |
| `MjData(model)` | app, physics tests | 시뮬레이션 상태 생성 |
| `mj_forward()` | physics 검증용 tests | 물리 엔진 결과를 독립 검증할 때만 사용; 앱 런타임과 자체 기구학 solver는 사용하지 않음 |
| `mj_step()` | app, tests | 물리 timestep 진행 |
| `mj_resetData()` | tests | data 초기화 |
| `mj_resetDataKeyframe()` | app, tests | keyframe으로 초기화 |
| `mj_name2id()` | 대부분 모듈 | 이름을 id로 변환 |
| `mj_id2name()` | `grasp.py` | id를 이름으로 변환 |
| `mj_contactForce()` | `grasp.py` | contact force 읽기 |
| `mju_mat2Quat()` | target/render/IK | matrix를 quaternion으로 변환 |
| `mju_quat2Mat()` | `teleop_targets.py` | quaternion을 matrix로 변환 |
| `mju_mulQuat()` | target/IK | quaternion 곱 |
| `mju_negQuat()` | target | quaternion inverse/conjugate |
| `MjvScene` | `teleop_render.py` | 렌더 scene |
| `MjvCamera` | `teleop_render.py` | 카메라 |
| `MjvOption` | `teleop_render.py` | 렌더 옵션 |
| `MjvPerturb` | `teleop_render.py` | perturb 구조체 |
| `MjrContext` | `teleop_render.py` | 렌더 context |
| `mjv_updateScene()` | `teleop_render.py` | scene 갱신 |
| `mjr_render()` | `teleop_render.py` | scene 렌더 |
| `mjv_moveCamera()` | `teleop_render.py` | mouse camera 조작 |

## MJCF 요소

| 요소/속성 | 역할 |
|---|---|
| `<body>` | 강체 |
| `<joint>` | 자유도 |
| `<freejoint>` | 6DOF 자유물체 |
| `<geom>` | 시각/충돌 형상 |
| `<site>` | 참조 좌표계 |
| `<actuator>` | joint 구동 |
| `<position>` | 위치 actuator |
| `<motor>` | 토크 actuator |
| `<velocity>` | 속도 actuator |
| `<keyframe>` | 초기 상태 저장 |
| `<equality><weld>` | body 간 제약 |
| `mocap="true"` | 외부에서 pose를 지정하는 kinematic marker body |
| `solref`, `solimp` | 접촉 solver 파라미터 |
| `friction` | 마찰 |
| `condim` | contact 차원 |
| `priority` | 접촉 파라미터 우선순위 |
| `<exclude>` | body pair collision 제외 |
| `<pair>` | geom pair 접촉 파라미터 |
