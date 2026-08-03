# 시각화 API

화면 출력과 사용자 입력을 두 모듈로 나눈다. 두 모듈 모두 IK solve나 physics step을
직접 수행하지 않는다.

| 모듈 | 책임 | 상세 문서 |
|---|---|---|
| `visualization.render` | 창, frame, MuJoCo 장면, 카메라, Gizmo, collision overlay | [렌더링 API](visualization-render.md) |
| `visualization.ui` | Control Center, Diagnostics와 기구학 트리 widget | [UI API](visualization-ui.md) |
