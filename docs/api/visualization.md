# 시각화 API

시각화 계층은 controller 계산이나 actuator 명령을 만들지 않는다. UI는 앱의 target과
mode 상태를 바꾸고, renderer는 그 상태와 MuJoCo 물리 결과를 화면에 그린다.

## 렌더링 생명주기

### `setup_render(app, window_w, window_h)`

- **직관:** GLFW 창, ImGui, MuJoCo scene/context와 기본 카메라를 한 번에 준비한다.
- **입력:** model이 준비된 앱, 창의 pixel 너비·높이.
- **반환:** 없음.
- **변경:** `app.window`, `scene`, `cam`, `opt`, `pert`, `context` 등 렌더 상태.
- **오류:** GLFW 창 또는 ImGui backend를 만들지 못하면 `RuntimeError`.

### `begin_frame(app)`

- **직관:** 운영체제 입력 event를 받고 새 ImGui frame을 시작한다.
- **반환:** 현재 ImGui IO. 키보드·마우스 capture 여부를 판단할 때 쓴다.

### `render_scene(app)`

- **직관:** 최신 물리 상태, target marker, 충돌 overlay와 Gizmo를 실제 framebuffer에
  그린다.
- **입력:** 초기화된 앱.
- **변경:** marker pose 동기화, MuJoCo scene, ImGui draw data, window buffer swap.
- **호출 시점:** physics step 뒤, frame당 한 번.

### `end_frame(app, t0)`

- **직관:** 실제 frame 주파수를 지수평균으로 갱신하고 남은 시간만 쉬어 목표 주기를
  맞춘다.
- **입력:** frame 시작 때의 `time.perf_counter()` 값.
- **변경:** `app.freq_ema`; 필요하면 짧게 대기한다.

### `shutdown(app)`

- **직관:** 생성의 역순으로 ImGui backend, 창과 GLFW를 안전하게 닫는다.
- **사용 시점:** `run()`의 `finally` 정리 경로.

## 카메라와 Gizmo

### `set_camera_preset(cam, preset)`

- **직관:** YAML에 저장한 overview 또는 hand-closeup 시점으로 즉시 이동한다.
- **입력:** `MjvCamera`, preset index. 0은 overview, 그 외는 closeup.
- **변경:** 카메라 `lookat`, 거리, 방위각, 고도각.

### `handle_camera_mouse(app, io)`

- **직관:** UI나 Gizmo가 마우스를 사용하지 않을 때만 drag/wheel을 MuJoCo 카메라
  회전·이동·zoom으로 전달한다.
- **입력:** 앱과 현재 ImGui IO.
- **변경:** `app.cam`과 마지막 mouse 위치.

### `pose_to_imguizmo_matrix(world_pos, world_quat)`

- **직관:** 로봇의 world pose를 ImGuizmo가 조작할 수 있는 column-major 4×4 행렬로
  포장한다.
- **반환:** `imguizmo.Matrix16`.

### `imguizmo_matrix_to_pose(matrix)`

- **직관:** 사용자가 끌어 바꾼 Gizmo 행렬을 다시 IK target 위치·quaternion으로 푼다.
- **반환:** `(world_position, world_quaternion)`.

### `draw_transform_gizmo(app, viewport)`

- **직관:** 현재 선택한 손 또는 가상 물체 위에 이동·회전 손잡이를 그리고, 조작된
  pose를 target 상태로 되돌린다.
- **입력:** 앱과 MuJoCo framebuffer viewport.
- **변경:** `app.gizmo_mouse_active`와 활성 target.

## 충돌 표시

### `collision_visualization_data(app)`

- **직관:** 화면 장식용 임의 거리 대신 WBIK CBF가 실제 감시 중인 충돌 결과만 가져온다.
- **반환:** collision 표시가 켜졌으면 `CollisionConstraint` tuple, 아니면 빈 tuple.

색상, 최근접점 sphere와 연결선은 renderer 내부 함수가 추가한다. 이들은 scene 용량을
검사하며 제어기의 거리나 gradient를 수정하지 않는다.

## UI

### `draw_panel(app)`

- **직관:** Control Center와 Diagnostics 창을 그리고 사용자의 widget 입력을 앱의
  target·mode·표시 상태에 반영한다.
- **입력:** 초기화된 `TeleopApp`.
- **반환:** 없음.
- **변경:** target, 팔/Whole-body mode, grasp, jog, window visibility 등 UI 소유 상태.
- **하지 않는 일:** IK solve, `data.ctrl` 기록, `mj_step`, 3D scene rendering.

### `kinematic_tree_body_ids(app, scope=None, show_full=None)`

- **직관:** Diagnostics 트리에 전체 모델을 보일지, 제어 중인 양팔 경로만 보일지에
  맞춰 body id 집합을 고른다.
- **입력:** 앱, 선택적 scope와 전체 표시 boolean. 생략하면 현재 UI 상태를 사용한다.
- **반환:** 표시할 body id의 `set`.
- **사용 시점:** 트리 widget 또는 트리 범위 테스트.

한 frame에서 이 함수들이 호출되는 순서는 [앱 조립과 물리 루프](../guide/teleop_app.md),
화면 좌표와 framebuffer 좌표의 관계는 [렌더링과 Gizmo](../guide/teleop_render.md)에 있다.
