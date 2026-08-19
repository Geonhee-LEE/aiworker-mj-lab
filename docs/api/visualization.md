# 시각화 API

UI와 renderer는 target·표시 상태만 변경한다. IK, actuator 기록과 `mj_step()`은
애플리케이션에서 실행한다.

## UI { #ui }

모듈은 `visualization.ui`다.

| 함수 | 입력·반환 또는 변경 |
|---|---|
| `draw_panel(app)` | Control Center와 Diagnostics를 그리고 app의 target·mode·창 상태 변경 |
| `kinematic_tree_body_ids(app, scope=None, show_full=None)` | 표시할 body id의 `set`; scope는 `"r"`, `"l"`, `"both"` |

`_draw_*` 함수는 widget 내부 구현이다.

## 렌더링 { #render }

모듈은 `visualization.render`다.

### 창과 프레임

| 함수 | 반환 또는 변경 |
|---|---|
| `setup_render(app, window_w, window_h)` | GLFW·ImGui·MuJoCo scene/context를 app에 생성 |
| `begin_frame(app)` | event 처리와 ImGui 프레임 시작; ImGui IO 반환 |
| `render_scene(app)` | marker·scene·overlay·Gizmo·ImGui를 window에 렌더링 |
| `end_frame(app, t0)` | `app.freq_ema` 갱신과 남은 프레임 시간 대기 |
| `shutdown(app)` | ImGui backend와 GLFW 자원 정리 |

`setup_render()`는 창 또는 backend 생성 실패 시 `RuntimeError`를 발생시킨다.

### Camera와 Gizmo

| 함수 | 반환 또는 변경 |
|---|---|
| `set_camera_preset(cam, preset)` | 설정된 camera pose 적용 |
| `handle_camera_mouse(app, io)` | UI가 사용하지 않는 mouse 입력을 `app.cam`에 적용 |
| `pose_to_imguizmo_matrix(world_pos, world_quat)` | world pose → column-major 4×4 값 |
| `imguizmo_matrix_to_pose(matrix)` | Gizmo 행렬 → `(world_position, world_quaternion)` |
| `draw_transform_gizmo(app, viewport)` | 활성 target 표시·조작; 결과를 target 상태에 반영 |
| `collision_visualization_data(app)` | 표시 중이면 WBIK collision constraint tuple, 아니면 빈 tuple |

화면 구성과 호출 순서는 [UI와 시각화](../guide/teleop_ui.md)를 참고한다.
