# `visualization.render`

GLFW, ImGui와 MuJoCo renderer의 생명주기 및 3D Gizmo를 담당한다.

## Frame 생명주기

| 함수 | 기능 | 반환 또는 변경 |
|---|---|---|
| `setup_render(app, window_w, window_h)` | 창·ImGui·scene/context 준비 | 앱 렌더 상태 |
| `begin_frame(app)` | OS event 처리와 새 ImGui frame 시작 | ImGui IO |
| `render_scene(app)` | marker·장면·overlay·Gizmo·UI 렌더 | framebuffer/window |
| `end_frame(app, t0)` | FPS EMA와 남은 frame 시간 조절 | `app.freq_ema` |
| `shutdown(app)` | backend와 GLFW를 생성 역순으로 정리 | 없음 |

`setup_render()`는 창 또는 backend 생성 실패 시 `RuntimeError`를 발생시킨다.

## 카메라와 Gizmo

| 함수 | 기능 | 반환 또는 변경 |
|---|---|---|
| `set_camera_preset(cam, preset)` | YAML의 overview/closeup 시점 적용 | `MjvCamera` |
| `handle_camera_mouse(app, io)` | UI가 쓰지 않는 drag/wheel을 카메라로 전달 | `app.cam` |
| `pose_to_imguizmo_matrix(world_pos, world_quat)` | world pose를 column-major 4×4 형식으로 변환 | `Matrix16` |
| `imguizmo_matrix_to_pose(matrix)` | Gizmo 행렬을 world pose로 복원 | `(position, quaternion)` |
| `draw_transform_gizmo(app, viewport)` | 활성 target 손잡이를 그리고 결과 반영 | target·mouse 상태 |

## 충돌 표시

### `collision_visualization_data(app)`

- **기능:** WBIK CBF가 감시하는 동일한 `CollisionConstraint`만 renderer에 제공한다.
- **반환:** 표시가 켜졌으면 constraint tuple, 아니면 빈 tuple.
- **주의:** overlay는 거리와 gradient를 수정하지 않는다.

한 frame의 호출 순서는 [앱 조립](../guide/teleop_app.md), framebuffer와 desktop
좌표 정렬은 [렌더링과 Gizmo](../guide/teleop_render.md)에 있다.
