# Journal digest (최근 20 cycle, 최신 위로)

_REVIEW 단계는 이 파일의 상위 5개 항목만 읽는다. 전체 보고서는 `journal/`에 있다._

## 2026-08-30 17:00 — repeat-loop-and-tree-viz-demo
- **Pick**: 사용자 요청 — 데모에 반복 목표 방문(`--loop`)과 RRT 트리 시각화(`--show-tree`) 추가
- **Outcome**: `PlannerResult`에 `TreeSnapshot`(start_tree/goal_tree) 추가, mjv_initGeom/mjv_connector로 트리를 뷰어에 렌더. 실제 디스플레이에서 반복 실행 세그폴트 없이 확인
- **Next**: MP-0005 shortcut 평활화, MP-0006 시간 파라미터화
- **Full**: [journal/2026-08/30-17-repeat-loop-and-tree-viz-demo.md](journal/2026-08/30-17-repeat-loop-and-tree-viz-demo.md)


## 2026-08-30 15:35 — p1-rrt-connect-plus-demo
- **Pick**: MP-0002/0003 RRT-Connect core + property tests, 사용자 요청으로 실행 데모까지
- **Outcome**: CONNECT 로직 버그(속성 시험이 발견) + 실행 재생 시작상태 동기화 버그(사용자 데모로 발견) 둘 다 수정. 25개 planning 테스트 통과
- **Next**: MP-0005 shortcut 평활화, MP-0006 시간 파라미터화
- **Full**: [journal/2026-08/30-15-p1-rrt-connect-plus-demo.md](journal/2026-08/30-15-p1-rrt-connect-plus-demo.md)


## 2026-08-30 14:00 — bootstrap-planning-module-and-automation
- **Pick**: PRD/TODO/자동화 스캐폴딩 + `planning/` P0 기반 구현 (사람 주도, 최초 커밋)
- **Outcome**: `RightArmSpace`, `ArmCollisionChecker`, `EdgeChecker` 골격 및 cron 자동화
  8종 wrapper 배선 완료
- **Next**: RRT-Connect 코어(MP-0002)부터 자동 루프가 이어받는다
- **Full**: [`journal/2026-08/30-14-bootstrap-planning-module-and-automation.md`](journal/2026-08/30-14-bootstrap-planning-module-and-automation.md)
