# 애플리케이션 API

`application.teleop`은 앱을 조립하고, `application.targets`는 UI의 홈 기준 값을 IK와
렌더러가 쓰는 world pose로 바꾼다. 여기서 quaternion 배열 순서는 모두
`(w, x, y, z)`다.

## 앱 실행과 모드

### `main(argv=None)`

- **직관:** 명령행 인자를 읽고 `TeleopApp`을 만들어 종료할 때까지 실행한다.
- **입력:** `--config PATH`를 포함할 수 있는 문자열 목록. `None`이면 실제 CLI를 쓴다.
- **반환:** 없음. 창과 물리 loop의 생명주기를 소유한다.
- **사용 시점:** `python3 src/teleop_app.py`와 같은 프로그램 진입점이 필요할 때.

### `KeyEdge.pressed(window, key)`

- **직관:** 키를 계속 누르고 있어도 처음 눌린 frame에만 `True`를 준다.
- **입력:** GLFW 창과 key code.
- **반환:** 눌림 edge 여부.
- **사용 시점:** reset·toggle처럼 한 번만 실행되어야 하는 키 처리.

### `TeleopApp.run()`

- **직관:** 입력 → UI → IK/제어 → 물리 → 렌더링 순서를 매 frame 반복한다.
- **입력/반환:** 없음.
- **변경:** 앱 상태, MuJoCo `data.ctrl`, 물리 상태, 렌더 상태.
- **사용 시점:** 직접 만든 `TeleopApp`을 실행할 때. 보통은 `main()`을 쓴다.

### `TeleopApp.set_arm_mode(side, mode)`

- **직관:** 한쪽 팔을 pose를 따라가는 `ik`와 관절 슬라이더를 따라가는 `fk` 중
  하나로 바꾼다.
- **입력:** `side`는 `"r"` 또는 `"l"`, `mode`는 `"ik"` 또는 `"fk"`.
- **변경:** 팔 모드와 전환 기준. FK→IK에서는 현재 손 pose를 새 목표로 잡아 튐을 막는다.

### `TeleopApp.set_whole_body_enabled(enabled)`

- **직관:** 손 목표를 그대로 둔 채 base·lift까지 IK에 참여시킬지 결정한다.
- **입력:** boolean.
- **변경:** target 표현과 WBIK reference. `False`이면 solver가 base·lift 속도를 정확히
  0으로 고정하고 팔만 푼다.

### 상태 변경 메서드

| 메서드 | 직관적인 기능 | 주요 변경 |
|---|---|---|
| `TeleopApp.reset_can()` | 캔 하나를 홈 주변의 임의 자세로 되돌린다 | 캔 free-joint 상태 |
| `TeleopApp.reset_active_object()` | 캔과 양손 캡처 상태를 함께 초기화한다 | 캔, grasp/Cyclo 상태 |
| `TeleopApp.cycle_camera()` | overview와 hand-closeup 카메라를 번갈아 선택한다 | 카메라 preset index |
| `TeleopApp.toggle_collision_visualization()` | CBF가 감시하는 충돌 형상·최근접점을 보이거나 숨긴다 | 표시 flag |
| `TeleopApp.toggle_whole_body_control()` | 현재 Whole-body boolean을 뒤집는다 | `set_whole_body_enabled()`와 동일한 안전 전환 |
| `TeleopApp.sync_virtual_object_to_hand_targets()` | 가상 물체를 두 손 목표의 중간으로 옮긴다 | virtual target |
| `TeleopApp.capture_grasp()` | 가상 물체와 양손의 상대 pose를 기록한다 | rigid-grasp reference |
| `TeleopApp.release_grasp()` | 기록한 양손 상대 pose를 해제한다 | capture 상태 |
| `TeleopApp.apply_virtual_object_target()` | 가상 물체의 이동을 양손 목표로 펼친다 | 좌우 손 target |

위 네 개의 양손 메서드는 같은 이름의 `application.targets` 함수로 전달하는 앱용
진입점이다.

## 기본 회전·위치 변환

| 함수 | 직관적인 기능 | 입력 | 반환 |
|---|---|---|---|
| `rpy_deg_to_quat(rpy_deg)` | 사람이 읽는 Roll/Pitch/Yaw를 solver 자세로 바꾼다 | degree 3-vector | `wxyz` quaternion |
| `quat_to_rpy_deg(q)` | solver 자세를 UI 각도로 바꾼다 | `wxyz` quaternion | degree `[roll, pitch, yaw]` |
| `quat_to_mat(quat)` | quaternion 회전을 행렬 계산에 쓴다 | `wxyz` quaternion | (3\times3) 회전행렬 |
| `mat_to_quat(mat)` | 행렬 회전을 MuJoCo 자세로 바꾼다 | (3\times3) 회전행렬 | `wxyz` quaternion |
| `base_pose(app)` | 현재 이동 베이스의 평면 pose와 회전 보조값을 한 번에 읽는다 | 초기화된 앱 | `(x, y, yaw, cos, sin, quaternion)` |
| `local_to_world_pos(app, p_local)` | 현재 베이스 기준 점을 world 점으로 옮긴다 | base-frame 3-vector | world 3-vector |
| `world_to_base_pos(app, p_world)` | world 점을 현재 베이스 좌표로 되돌린다 | world 3-vector | base-frame 3-vector |
| `anchor_local_to_world_pos(app, p_local)` | Whole-body 시작 기준점의 local 점을 world로 옮긴다 | anchor-frame 3-vector | world 3-vector |
| `world_to_anchor_local_pos(app, p_world)` | world 점을 Whole-body 시작 기준으로 되돌린다 | world 3-vector | anchor-frame 3-vector |

`local_*`은 움직이는 현재 base를, `anchor_*`는 Whole-body 목표를 world에 고정하기
위한 기준 pose를 사용한다. 두 표현의 이유는 [목표와 좌표 변환](../guide/teleop_targets.md)에
있다.

## 손 목표 변환

| 함수 | 직관적인 기능 | 입력 | 반환/변경 |
|---|---|---|---|
| `set_home_references(app)` | 현재 양손과 base를 UI의 영점으로 기억한다 | 앱 | home/anchor 상태 변경 |
| `carry_world_targets_with_base(app, previous_base_pose, current_base_pose)` | 수동 주행으로 로봇을 옮길 때 목표도 같은 SE(2) 이동만큼 운반한다 | 이전·현재 `(x,y,yaw)` | world home/anchor 변경 |
| `target_pos_to_base_pos(app, side, pos_target)` | 손의 홈 오프셋을 base-frame 절대 위치로 만든다 | 손, UI 3-vector | base-frame 위치 |
| `target_pos_to_world_pos(app, side, pos_target)` | 현재 ON/OFF 규칙에 맞춰 UI 위치를 IK world 위치로 만든다 | 손, UI 3-vector | world 위치 |
| `world_to_target_pos(app, side, world_pos)` | world 위치를 UI 슬라이더 값으로 역변환한다 | 손, world 위치 | 3-element list |
| `target_rpy_to_world_quat(app, side, rpy_deg)` | 홈 기준 RPY를 현재 ON/OFF 규칙의 world 자세로 만든다 | 손, degree RPY | world quaternion |
| `target_world_quat(app, side)` | 앱에 저장된 한 손의 RPY를 world 자세로 읽는다 | 앱, 손 | world quaternion |
| `world_quat_to_target_rpy(app, side, world_quat)` | world 자세를 한 손의 UI RPY로 되돌린다 | 앱, 손, quaternion | degree RPY |
| `world_quat_to_virtual_rpy(app, world_quat)` | world 자세를 가상 물체의 기준 RPY로 되돌린다 | 앱, quaternion | degree RPY |
| `target_world_pose(app, side)` | 한 손의 위치와 자세를 solver 입력 한 쌍으로 만든다 | 앱, 손 | `(world_position, world_quaternion)` |

`side`는 항상 `"r"` 또는 `"l"`이다. Whole-body ON에서는 startup/carried anchor,
OFF에서는 움직이는 현재 base가 target frame이다.

## 양손 가상 물체와 마커

| 함수 | 직관적인 기능 | 입력 | 반환/변경 |
|---|---|---|---|
| `virtual_object_world_pose(app)` | 양손이 함께 운반할 가상 물체의 world pose를 만든다 | 앱 | `(position, quaternion)` |
| `sync_virtual_object_to_hand_targets(app)` | 가상 물체를 현재 두 손 목표의 중점으로 맞춘다 | 앱 | virtual target 변경 |
| `capture_grasp(app)` | 물체에서 본 각 손의 상대 pose를 고정한다 | 앱 | capture offset과 상태 변경 |
| `release_grasp(app)` | 상대 pose 고정을 푼다 | 앱 | capture 상태 초기화 |
| `apply_virtual_object_target(app)` | 가상 물체 pose와 캡처 offset으로 좌우 손 목표를 재계산한다 | 앱 | 양손 target 변경 |
| `bimanual_marker_visible(app)` | 가상 물체 marker를 보여야 하는 상태인지 묻는다 | 앱 | boolean |
| `sync_marker_visibility(app)` | 위 판정을 실제 geom/site alpha에 반영한다 | 앱 | 모델 표시 색상 변경 |
| `active_gizmo_target(app)` | 현재 Gizmo가 오른손·왼손·가상 물체 중 무엇을 잡을지 고른다 | 앱 | `"r"`, `"l"`, `"virtual"` |
| `gizmo_target_world_pose(app, target)` | 선택한 Gizmo 대상의 world pose를 읽는다 | 앱, target 이름 | `(position, quaternion)` |
| `set_gizmo_target_world_pose(app, target, world_pos, world_quat)` | Gizmo 결과를 UI target 표현으로 되돌려 쓴다 | 앱, 대상, world pose | target 변경 |
| `sync_ik_mocaps_from_targets(app)` | 계산된 target pose와 가시성을 MuJoCo marker에 복사한다 | 앱 | `data.mocap_*`, marker alpha 변경 |

이 함수들은 actuator를 쓰지 않는다. 목표 상태만 바꾸며, 실제 관절·바퀴 명령은 다음
physics frame의 제어 계층이 만든다.
