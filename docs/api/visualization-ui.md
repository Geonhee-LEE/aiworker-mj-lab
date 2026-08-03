# `visualization.ui`

ImGui widget을 그리고 앱의 target·mode·표시 상태를 변경한다. IK, actuator 기록과
`mj_step`은 수행하지 않는다.

## `draw_panel(app)`

- **기능:** Control Center와 Diagnostics 창을 그리는 frame 진입점.
- **입력:** 초기화된 `TeleopApp`.
- **변경:** target, 팔/Whole-body mode, grasp, jog, 창 표시 상태.
- **반환:** 없음.

## `kinematic_tree_body_ids(app, scope=None, show_full=None)`

- **기능:** Diagnostics 트리에 표시할 body 범위를 고른다.
- **입력:** 앱, 선택적 `"r"`/`"l"`/`"both"` scope와 전체 표시 여부.
- **반환:** body id의 `set`.
- **사용:** 기구학 트리 widget과 표시 범위 테스트.

내부 `_draw_*` 함수는 widget 구성 세부사항이며 공개 API가 아니다. UI 구성과 창 분리
규칙은 [UI 패널](../guide/teleop_ui.md)을 참고한다.
