# 애플리케이션 API

## `application.teleop` { #teleop }

### 실행

| API | 입력·반환 | 동작 |
|---|---|---|
| `main(argv=None)` | CLI 인자 목록, 반환 없음 | `TeleopApp`을 생성하고 종료할 때까지 실행 |
| `TeleopApp.run()` | 반환 없음 | 입력 → UI → 제어·물리 → 렌더링 반복 |
| `KeyEdge.pressed(window, key)` | GLFW window/key → `bool` | 키를 누른 첫 프레임에만 `True` |

`--config PATH`는 `python3 src/teleop_app.py` 실행기가 패키지를 import하기 전에 적용한다.
`application.teleop.main()`을 직접 호출할 때는 같은 설정 경로가 이미 선택되어 있어야 한다.

### 상태 변경

| 메서드 | 동작 |
|---|---|
| `set_arm_mode(side, mode)` | 손별 `"ik"`/`"fk"` 전환; FK→IK는 현재 site pose로 target 동기화 |
| `set_whole_body_enabled(enabled)` | 손과 virtual object의 world pose를 보존하며 전신/팔 전용 모드 전환 |
| `toggle_whole_body_control()` | 현재 전신 제어 상태 반전 |
| `capture_grasp()` / `release_grasp()` | 양손 상대 pose와 solver rigid-grasp 기준 설정·해제 |
| `apply_virtual_object_target()` | virtual object pose에서 양손 target 갱신 |
| `reset_can()` / `reset_active_object()` | 캔 또는 캔·파지 상태 초기화 |
| `cycle_camera()` | camera preset 변경 |
| `toggle_collision_visualization()` | collision overlay 표시 변경 |

상세 실행 순서는 [애플리케이션과 목표 좌표](../guide/teleop_app.md)를 참고한다.

## `application.targets` { #targets }

이 함수들은 `app`에 저장된 좌표 기준과 target을 읽거나 바꾼다. IK solve와 actuator
기록은 하지 않는다. `side`는 `"r"` 또는 `"l"`이다.

### 기준 좌표

| 함수 | 반환 또는 변경 |
|---|---|
| `set_home_references(app)` | 현재 양손·base pose를 home과 anchor로 저장 |
| `base_pose(app)` | `(x, y, yaw, cos_yaw, sin_yaw, yaw_quaternion)` |
| `local_to_world_pos(app, p_local)` | live-base 점 → world 3-vector |
| `world_to_base_pos(app, p_world)` | world 점 → live-base 3-vector |
| `anchor_local_to_world_pos(app, p_local)` | anchor 점 → world 3-vector |
| `world_to_anchor_local_pos(app, p_world)` | world 점 → anchor 3-vector |
| `carry_world_targets_with_base(app, previous, current)` | 측정된 base SE(2) 이동만큼 home·anchor 이동 |

### 손과 virtual object

| 함수 | 반환 또는 변경 |
|---|---|
| `target_pos_to_world_pos(app, side, pos_target)` | 내부 위치 target → world 위치 |
| `world_to_target_pos(app, side, world_pos)` | world 위치 → 내부 위치 target |
| `target_rpy_to_world_quat(app, side, rpy_deg)` | 내부 RPY → world quaternion |
| `target_world_quat(app, side)` | 저장된 손 target의 world quaternion |
| `world_quat_to_target_rpy(app, side, quat)` | world quaternion → 손 target RPY |
| `world_quat_to_virtual_rpy(app, quat)` | world quaternion → virtual object RPY |
| `target_world_pose(app, side)` | `(world_position, world_quaternion)` |
| `virtual_object_world_pose(app)` | virtual object의 world pose |
| `capture_grasp(app)` / `release_grasp(app)` | 양손 상대 transform 저장·해제 |
| `apply_virtual_object_target(app)` | 저장된 transform으로 양손 target 갱신 |

### Marker와 Gizmo

| 함수 | 반환 또는 변경 |
|---|---|
| `active_gizmo_target(app)` | `"r"`, `"l"`, `"virtual"` 중 활성 대상 |
| `gizmo_target_world_pose(app, target)` | 활성 대상의 world pose |
| `set_gizmo_target_world_pose(app, target, pos, quat)` | Gizmo world pose를 내부 target으로 저장 |
| `sync_ik_mocaps_from_targets(app)` | 손·virtual target을 MuJoCo mocap에 복사 |
| `sync_marker_visibility(app)` | capture 상태에 따라 virtual marker alpha 변경 |

좌표 변환식과 ON/OFF 기준은 [목표 좌표계](../guide/teleop_app.md#target-frames)에 있다.
