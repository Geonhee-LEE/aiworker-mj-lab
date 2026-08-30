# Journal digest (최근 20 cycle, 최신 위로)

_REVIEW 단계는 이 파일의 상위 5개 항목만 읽는다. 전체 보고서는 `journal/`에 있다._

## 2026-08-30 21:00 — p2-shortcut-smoothing
- **Pick**: MP-0005 — RRT-Connect 경로 shortcut 평활화(`planning/shortcut.py`)
- **Outcome**: 무작위 두 waypoint 사이 직선 구간을 `EdgeChecker`로 검증해 잘라내는 `shortcut_path` + `path_length_rad` 헬퍼 추가. 신규 6개 포함 31개 planning 테스트 통과(끝점 보존·길이 비증가·무충돌·결정론 속성). 합성 slab 시나리오 median reduction 23%(비공식, 실제 장면 측정은 벤치마크 하네스 대기)
- **Next**: MP-0006 시간 파라미터화, MP-0013 벤치마크 하네스
- **Full**: [journal/2026-08/30-21-p2-shortcut-smoothing.md](journal/2026-08/30-21-p2-shortcut-smoothing.md)


## 2026-08-30 20:00 — interactive-mouse-goal-marker
- **Pick**: 사용자 요청 — teleop_app.py처럼 마우스로 목표를 드래그하는 인터랙티브 모드
- **Outcome**: mocap body + 뷰어 기본 조작으로 커스텀 마우스 코드 없이 드래그 가능. 데모 전용 position-우선 DLS IK 추가. 버그 2개(시작상태 미동기화 재발, 무한 재트리거) 발견·수정, 실제 main() 경로로 세그폴트 없이 검증
- **Next**: MP-0005 shortcut 평활화, MP-0006 시간 파라미터화
- **Full**: [journal/2026-08/30-20-interactive-mouse-goal-marker.md](journal/2026-08/30-20-interactive-mouse-goal-marker.md)


## 2026-08-30 19:00 — qspace-visualization-and-cvd-safe-palette
- **Pick**: 사용자 요청 — Q-space(관절 공간) 시각화. Artifact로 parallel coordinates 페이지 게시
- **Outcome**: dataviz 스킬 팔레트 검증기(Node 없어 Python으로 포팅)가 기존 초록/주황 조합의 protanopia Delta E 2.8 하드 FAIL을 발견. 초록/파랑/마젠타로 교체해 검증 통과, 3D 뷰어 색도 함께 맞춤
- **Next**: MP-0005 shortcut 평활화, MP-0006 시간 파라미터화
- **Full**: [journal/2026-08/30-19-qspace-visualization-and-cvd-safe-palette.md](journal/2026-08/30-19-qspace-visualization-and-cvd-safe-palette.md)


## 2026-08-30 18:00 — obstacle-in-real-reach-region
- **Pick**: 사용자 지적 — 장애물이 오른팔 실제 동작 영역 밖(테이블 위)에 있었음. 구체 3개로 교체 + 위치 재배치 + 크기 확대
- **Outcome**: RightArmSpace.sample() 분포 실측으로 실제 도달 영역 파악, 손끝만 보고 배치했다가 팔 body 충돌로 실패 → is_valid(START) 체계적 검증으로 재배치. 반지름 6cm에서 시작 자세 유효 유지, 차단율 54%
- **Next**: MP-0005 shortcut 평활화, Q-space 시각화(사용자 요청)
- **Full**: [journal/2026-08/30-18-obstacle-in-real-reach-region.md](journal/2026-08/30-18-obstacle-in-real-reach-region.md)


## 2026-08-30 17:30 — add-obstacle-and-persistent-path-viz
- **Pick**: 사용자 요청 — 오른팔 탐색 영역에 장애물 추가, 실행 중 경로 시각화 유지
- **Outcome**: MjSpec으로 모델 파일을 안 건드리고 빨간 기둥(planning_obstacle) 추가, 실제 충돌 판정에 관여함을 검증. 실행 중 주황색 경로 시각화가 지워지지 않고 유지되도록 수정
- **Next**: MP-0005 shortcut 평활화, MP-0006 시간 파라미터화
- **Full**: [journal/2026-08/30-17-30-add-obstacle-and-persistent-path-viz.md](journal/2026-08/30-17-30-add-obstacle-and-persistent-path-viz.md)


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
