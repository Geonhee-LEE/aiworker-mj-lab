# `application.teleop`

MuJoCo model/data, UI, 렌더러와 제어기를 조립하고 frame loop를 소유한다.

## 모듈 함수

### `main(argv=None)`

- **기능:** 명령행 인자를 읽고 `TeleopApp`을 생성해 종료할 때까지 실행한다.
- **입력:** `--config PATH`를 포함할 수 있는 문자열 목록. `None`이면 실제 CLI.
- **반환:** 없음.

## `KeyEdge`

### `KeyEdge.pressed(window, key)`

- **기능:** 키를 계속 눌러도 처음 눌린 frame에만 `True`를 반환한다.
- **사용:** reset·toggle처럼 한 번만 실행할 입력.

## `TeleopApp`

### `TeleopApp.run()`

- **기능:** 입력 → UI → IK/제어 → 물리 → 렌더링을 매 frame 반복한다.
- **변경:** 앱 상태, `data.ctrl`, MuJoCo 물리와 렌더 상태.

### `TeleopApp.set_arm_mode(side, mode)`

- **입력:** `side`는 `"r"`/`"l"`, `mode`는 `"ik"`/`"fk"`.
- **기능:** FK→IK에서는 현재 site pose를 목표로 잡아 전환 순간의 튐을 막는다.

### `TeleopApp.set_whole_body_enabled(enabled)`

- **기능:** 손의 world 목표를 보존하며 Whole-body와 arm-only를 전환한다.
- **OFF:** solver의 base·lift 속도를 0으로 고정하고 팔만 푼다.
- **변경:** target 표현, smoothing, solver reference와 이전 base 명령.

### 상태 변경 메서드

| 메서드 | 기능 | 주요 변경 |
|---|---|---|
| `TeleopApp.reset_can()` | 캔을 홈 주변 임의 자세로 되돌림 | 캔 free-joint 상태 |
| `TeleopApp.reset_active_object()` | 캔과 양손 캡처 상태 초기화 | 캔, grasp/Cyclo 상태 |
| `TeleopApp.cycle_camera()` | 카메라 preset 순환 | 카메라 index |
| `TeleopApp.toggle_collision_visualization()` | CBF overlay 표시 전환 | 표시 flag |
| `TeleopApp.toggle_whole_body_control()` | Whole-body boolean 전환 | `set_whole_body_enabled()` 호출 |
| `TeleopApp.capture_grasp()` | 양손 상대 pose와 solver 기준 저장 | rigid-grasp reference |
| `TeleopApp.release_grasp()` | 저장한 상대 pose 해제 | capture 상태 |
| `TeleopApp.apply_virtual_object_target()` | 가상 물체 이동을 두 손 목표로 전개 | 양손 target |

실제 좌표 계산은 [목표 좌표 API](application-targets.md), frame 내부 순서는
[앱 조립과 물리 루프](../guide/teleop_app.md)에서 확인한다.
