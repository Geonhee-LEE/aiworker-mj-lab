# `application.targets`

UI의 홈 기준 XYZ/RPY, IK의 world pose, 렌더 marker pose 사이를 변환한다. 이 함수들은
actuator를 쓰지 않고 `app.targets`와 marker 상태만 변경한다.

## 기준 좌표

| 함수 | 기능 | 반환 |
|---|---|---|
| `set_home_references(app)` | 현재 양손과 base를 target 영점으로 저장 | 없음; home/anchor 변경 |
| `base_pose(app)` | 현재 base 평면 pose와 보조 회전값 읽기 | `(x, y, yaw, cos, sin, quat)` |
| `local_to_world_pos(app, p_local)` | live base-local 점을 world로 변환 | world 3-vector |
| `world_to_base_pos(app, p_world)` | world 점을 live base-local로 변환 | base 3-vector |
| `anchor_local_to_world_pos(app, p_local)` | startup anchor 점을 world로 변환 | world 3-vector |
| `world_to_anchor_local_pos(app, p_world)` | world 점을 startup anchor로 역변환 | anchor 3-vector |
| `carry_world_targets_with_base(app, previous, current)` | 수동 주행 SE(2) 이동만큼 world 기준 운반 | anchor/home 변경 |

`local_*`은 움직이는 현재 base, `anchor_*`는 Whole-body 목표를 world에 고정하는 시작
기준을 사용한다.

## 손 목표 변환

| 함수 | 기능 | 반환 |
|---|---|---|
| `target_pos_to_world_pos(app, side, pos_target)` | UI 위치를 현재 모드의 world 위치로 변환 | world 3-vector |
| `world_to_target_pos(app, side, world_pos)` | world 위치를 UI offset으로 역변환 | 3-element list |
| `target_rpy_to_world_quat(app, side, rpy_deg)` | 홈 기준 RPY를 world 자세로 변환 | `wxyz` quaternion |
| `target_world_quat(app, side)` | 저장된 한 손 RPY를 world 자세로 읽기 | `wxyz` quaternion |
| `world_quat_to_target_rpy(app, side, world_quat)` | world 자세를 손 RPY로 역변환 | degree RPY |
| `world_quat_to_virtual_rpy(app, world_quat)` | world 자세를 가상 물체 RPY로 역변환 | degree RPY |
| `target_world_pose(app, side)` | 한 손의 최종 solver 입력 생성 | `(position, quaternion)` |

`side`는 `"r"` 또는 `"l"`이다. Whole-body ON은 startup/carried anchor, OFF는 live
base를 target frame으로 사용한다.

## 양손 가상 물체

| 함수 | 기능 | 반환 또는 변경 |
|---|---|---|
| `virtual_object_world_pose(app)` | 가상 물체의 world pose 계산 | `(position, quaternion)` |
| `sync_virtual_object_to_hand_targets(app)` | 가상 물체를 두 손 목표 중점에 배치 | virtual target |
| `capture_grasp(app)` | 물체에서 본 양손 상대 transform 저장 | capture offset·상태 |
| `release_grasp(app)` | 상대 transform 해제 | capture 상태 |
| `apply_virtual_object_target(app)` | 물체 pose와 offset으로 양손 목표 재계산 | 양손 target |

## Gizmo와 marker

| 함수 | 기능 | 반환 또는 변경 |
|---|---|---|
| `sync_marker_visibility(app)` | 캡처 상태에 맞춰 가상 marker alpha 변경 | 모델 RGBA |
| `active_gizmo_target(app)` | 조작할 손 또는 가상 물체 선택 | `"r"`, `"l"`, `"virtual"` |
| `gizmo_target_world_pose(app, target)` | 선택 대상의 world pose 읽기 | `(position, quaternion)` |
| `set_gizmo_target_world_pose(app, target, pos, quat)` | Gizmo 결과를 target 표현으로 저장 | target 변경 |
| `sync_ik_mocaps_from_targets(app)` | 최종 target pose를 MuJoCo mocap에 복사 | `data.mocap_*` |

좌표계의 수식과 모드 전환 이유는 [목표와 좌표 변환](../guide/teleop_targets.md)을
참고한다.
