# Journal digest (최근 20 cycle, 최신 위로)

_REVIEW 단계는 이 파일의 상위 5개 항목만 읽는다. 전체 보고서는 `journal/`에 있다._

## 2026-09-05 — p7-1-base-pose
- **Pick**: PR #9(PRD)·#10(P7.0 reachability map)이 병합된 뒤 계획대로 **MP-0027** P7.1 `planning/base_pose.py` 착수 — "decoupled" 패턴의 두 번째 조각(베이스를 어디에 둘지 고르기)
- **Outcome**: `world_to_base_frame`(월드→베이스 SE(2) 변환, base_link이 z로는 절대 안 움직인다는 실측에 근거해 z는 통과시킴), `BaseFootprintChecker`(`ArmCollisionChecker`와 같은 scratch-model 아키텍처 재현), `select_base_pose`(reachability 점수·발자국 충돌·현재 위치 근접도로 후보 순위 — yaw는 로봇의 "정면" 축을 가정하지 않고 후보 각도 집합을 위치·방향 양쪽에 재사용). 베이스 주행은 `WholeBodyIK`(손 목표 반응형이라 지점-대-지점엔 안 맞음) 대신 기존 `SwerveDrive`를 목표-오차 비례 루프로 얇게 감싼 `planning/mobile_execution.py`로 처리. **핵심 발견**: `build_reachability_map`이 실은 "베이스 원점 전용"이 아니라 grid point를 그냥 절대 world IK 타겟으로 쓴다는 걸 확인해, 이 함수를 그대로 재사용(새 IK 검증 코드 없이)해 임의 베이스 위치에서의 진짜 IK 도달성을 검증하는 핵심 회귀 테스트를 만들 수 있었다 — 먼 베이스 위치(3,3,0)에서는 도달 불가능하던 타겟이 `select_base_pose`가 고른 위치에서는 도달 가능해짐을 실제 IK로 증명. 실제 장면엔 베이스 발자국 높이대에 겹치는 정적 장애물이 없어(table이 그 위에 있음) 참-충돌 테스트는 합성 MJCF로 대신함. 13개 신규 테스트 + 기존 47개 모두 통과, PR #11 생성
- **Next**: PR #3/#4/#5/#7/#8/#11 사람 리뷰/병합, P7 Tier 2(결합형 `WholeBodySpace`, 우선순위 낮음), 사용자가 정하면 IK 실패(`MP-0011`)/연속 동작(`MP-0021`) 진행
- **Full**: [journal/2026-09/05-p7-1-base-pose.md](journal/2026-09/05-p7-1-base-pose.md)


## 2026-09-04 — p7-reachability-map
- **Pick**: 사용자 요청 — 오른팔 단독이 아니라 모바일 매니퓰레이터(베이스+팔) 전체의 IK·모션 플래닝 설계. PRD Non-Goals("왼팔·베이스·리프트 계획...범위 밖")를 명시적으로 넘어서는 요청이라 조사부터 시작
- **Outcome**: 조사 결과 로봇은 실제로 모바일(수동 평면 가상 관절+스워브 드라이브)이고, 반응형 whole-body IK(`control.whole_body.WholeBodyIK`)와 베이스 실행 계층(`control.base`)이 이미 있어 재구현 불필요 — 빠진 건 전역 모션 플래닝뿐. 실전 시스템 표준인 decoupled 패턴(reachability map 기반 베이스 배치 + 기존 팔 계획기 재사용)을 Tier 1으로 설계, 완전 결합 `WholeBodySpace`는 Tier 2(후속)로 미룸. PRD를 먼저 갱신(PR #9, Non-Goals에서 베이스·리프트 제외+P7 로드맵 추가)한 뒤 P7.0 `planning/reachability.py`(PR #10, MP-0026) 구현 — 새 IK 없이 기존 `_ik_attempt` 패턴 재사용, 격자 경계는 3000-표본 백분위 실측 근거. 기본 격자(504점) 빌드 81초, 도달 가능/불가능 분리 확인. 47개 테스트 통과. `todo_tool.py`의 `PHASES`가 P6까지만 있어 P7 행이 거부되던 걸 같은 브랜치에서 수정
- **Next**: PR #9/#10 포함 7개 PR 사람 리뷰/병합 대기, **MP-0027** P7.1 `planning/base_pose.py`(reachability map 소비+베이스 발자국 충돌+end-to-end 데모)
- **Full**: [journal/2026-09/04-p7-reachability-map.md](journal/2026-09/04-p7-reachability-map.md)


## 2026-09-04 — p3-execution-module
- **Pick**: 사용자 요청 — PRD를 고려해 다음 작업 진행. 로드맵상 다음 미착수 단계는 P3(정식 실행 모듈). `Trajectory`가 필요해 사용자 승인 하에 PR #1(shortcut)·#2(time_parameterize)를 먼저 병합(독립 코드 리뷰+로컬 merge dry-run으로 안전 확인 후)
- **Outcome**: PR #2 병합 시 `__init__.py`에서 PR #1과 충돌 — export 합집합으로 수동 해결. `planning/execution.py`의 `follow_trajectory`가 `ArmTorqueController` 토크만으로 재생하는 폐루프 함수를 제공, 매 표본마다 `ArmCollisionChecker.is_valid` 재확인으로 "침투 없음"을 직접 검증. 실측: 4개 seed 모두 최종 site 오차 0.07~0.09mm(PRD 목표 5mm 대비 60배 여유), 침투 0건 — velocity feedforward 없이도 여유 있게 충족. **작업 중 사고 2건**: `git reset --hard`를 상태 확인 없이 실행해 cron 루프의 미커밋 변경을 날렸다가 대화 컨텍스트로 정확히 복원, `git checkout -b <기존 브랜치>`가 조용히 실패해 main에 남은 채 명령을 실행한 사고가 두 번 더 반복(이 세션 통산 세 번째) — 둘 다 push 전 발견해 원격 영향 없이 복구
- **Next**: PR #3/#4/#5/#7/#8 사람 리뷰/병합(PR #4/#5는 이제 `__init__.py` 충돌 상태), MP-0007/MP-0017 비교 벤치마크, 사용자가 정하면 IK 실패/연속 동작 한계 진행
- **Full**: [journal/2026-09/04-p3-execution-module.md](journal/2026-09/04-p3-execution-module.md)


## 2026-09-03 — benchmark-harness
- **Pick**: STATE.md 병목이 두 사이클 연속 같은 항목(MP-0013 벤치마크 하네스 부재). MP-0004/0007/0014/0017이 전부 대기 중이었고, 자동 researcher가 이미 설계 지침(`research/2026-09/001.md`, `002.md`)을 조사해 둠
- **Outcome**: `scripts/benchmark_planning.py` — `demo_plan_right_arm.py`의 장면·목표 샘플링 재사용, `wall_budget_s` 가드로 2분 예산 보장. 도구를 바로 실제로 돌려 MP-0004도 같은 사이클에서 닫음: 50 seed×2 시나리오(장애물 유/무) 모두 성공률 100%, 계획 시간 중앙값 ~13ms. **작업 중 사고**: 동시에 도는 curator 프로세스와 워킹 디렉토리를 공유하다 브랜치 전환이 꼬여 main에 코드를 두 번 잘못 커밋 — 둘 다 push 전에 발견해 안전 브랜치+cherry-pick으로 완전히 복구, 원격엔 영향 없음. curator의 정당한 상태 커밋(`research/cron_activity.md`)은 별도 보존했으나 raw `git push origin main`이 auto-mode 분류기에 막혀 로컬에 남아 있음(사람 처리 필요)
- **Next**: PR #1~#7 사람 리뷰/병합(7개로 늘어남), 로컬 main의 curator 커밋 push, MP-0007/0014/0017을 이 하네스로 확장
- **Full**: [journal/2026-09/03-benchmark-harness.md](journal/2026-09/03-benchmark-harness.md)


## 2026-09-03 — chomp-posture-smoothing
- **Pick**: 사용자 지적 — IK 계산 자주 실패·연속 동작이 부드럽지 않음·팔 자세가 기괴함 세 한계 중 "자세 기괴함"을 먼저 진행. 원인: RRT-Connect는 비용 함수가 없고 RRT*의 비용은 경로 길이일 뿐, shortcut도 경로를 짧게 할 뿐 매끄럽게 하지 않음
- **Outcome**: `planning/chomp.py` — 시작·끝 고정, 가속도(2차 차분) 최소화 QP를 관절별 독립 1-D QP 7개로 풀어 `kinematics/optimization.py`의 기존 QP 유틸리티를 그대로 재사용. trust region + EdgeChecker 재검증 + 실패 시 원본 반환으로 무효 경로 불변식 유지. 데모 `--posture-smooth` 실측: 매끄러움 비용 0.14~2.06 → 대부분 0에 가깝게 개선, 재생 오차는 기존과 동일(~0.02 rad). 안전 폴백도 실제 장면에서 발동 확인. 5개 테스트 통과, PR #5 생성
- **Next**: PR #1~#5 사람 리뷰/병합, 남은 두 한계(IK 실패·연속 동작) 사용자 우선순위 확인 후 진행
- **Full**: [journal/2026-09/03-chomp-posture-smoothing.md](journal/2026-09/03-chomp-posture-smoothing.md)


## 2026-09-03 — rrt-star-planner
- **Pick**: 사용자 요청 — RRT-Connect 외 "향상된 다른 모션 플래닝 기법" 추가. `TODO.md`의 MP-0015/16/17(P5)이 정확히 이 백로그였음
- **Outcome**: `rrt_connect.py` 자산을 재사용하는 단일 트리 RRT*(`planning/rrt_star.py`) 구현 — 고정 rewiring 반경(이론적 shrinking radius 대신, 7-DOF에서 이론식은 반경이 너무 빨리 줄어 rewiring이 거의 안 일어남), 거부 표집 기반 informed sampling. **중요 발견**: RRT-Connect의 기본 `goal_bias`(0.1)를 그대로 물려주면 실제 can-sort 장면에서 40초·6750회 반복 안에도 목표에 못 닿음 — 단일 트리는 bidirectional CONNECT처럼 한 반복에 여러 스텝을 전진 못 하기 때문(RRT-Connect가 고안된 이유이기도 한 트레이드오프). `goal_bias=0.3`/`goal_tolerance_rad=0.5`/`time_budget_s=30`으로 데모 기본값 조정해 해결. 데모에 `--planner {rrt_connect,rrt_star}` 추가, 31개 planning 테스트 통과, PR #4 생성
- **Next**: PR #1~#4 사람 리뷰/병합, MP-0013 벤치마크 하네스(MP-0017 정식 비교표에 필요), PR #3 병합 후 RRT*에도 shortcut+시간 파라미터화 연결
- **Full**: [journal/2026-09/03-rrt-star-planner.md](journal/2026-09/03-rrt-star-planner.md)


## 2026-09-03 — demo-natural-motion
- **Pick**: 사용자 지적 — `--interactive` 데모에서 팔이 목적지에 도달은 하지만 중간 경로 자세가 부자연스러움. 원인: `_run_cycle`/`_run_interactive`가 RRT-Connect의 raw(지그재그) 경로를 그대로 재생. 사용자가 "RRT-Connect 외 다른 향상된 기법 추가 또는 부자연스러운 모션 개선을 먼저 진행"을 요청, 조사 결과 이미 구현·테스트 끝난 PR #1(MP-0005 shortcut)·PR #2(MP-0006 time_parameterize)가 이 문제를 직접 푸는 코드였음을 확인
- **Outcome**: 두 PR을 병합하지 않고(사용자 선택) 파일 내용만 새 브랜치로 옮겨 데모 실행 경로에 연결. `_execute`를 waypoint 수렴-게이팅에서 `Trajectory` 표본 재생으로 교체. **중요 발견**: config의 하드웨어 관절 속도 한계(4.8 rad/s)를 그대로 재생에 쓰면 이 데모의 오픈루프 PD 토크 컨트롤러가 못 따라가 중간 추종 오차가 1.5 rad까지 벌어짐(`Trajectory`를 실제로 소비하는 첫 코드라 지금까지 발견 안 됐던 문제) — 재생 전용 보수적 속도(`--exec-max-speed-rad-s`, 기본 1.0 rad/s)를 config의 하드웨어 스펙과 분리해 해결. 43개 planning 테스트 통과, PR #3 생성
- **Next**: PR #1/#2/#3 사람 리뷰/병합, `planning/p5-rrt-star-planner`(MP-0016, RRT* 대안 플래너) 착수
- **Full**: [journal/2026-09/03-demo-natural-motion.md](journal/2026-09/03-demo-natural-motion.md)


## 2026-08-31 11:10 — p2-time-parameterize
- **Pick**: MP-0006 — `planning.trajectory.time_parameterize` 사다리꼴 속도 프로파일. `research/feed.md`의 같은 날 researcher 노트가 이 TODO를 정확히 다뤄 설계 출발점으로 삼음
- **Outcome**: 처음 구현한 "웨이포인트에서 안 멈추는 전역 단일 프로파일"이 다중 waypoint 합성 경로 시험에서 가속도 상한 위반(상한 4.0 vs 실측 173 rad/s²)을 냄 — 세그먼트 경계(코너)에서 관절 속도 방향이 불연속으로 바뀌기 때문. research 노트가 권장한 "세그먼트별 독립 사다리꼴 + 매 waypoint 정지"(moveit 계열)로 재구현해 해결. 모든 관절이 같은 스칼라 상한을 쓰므로 표준 다관절 동기화가 Linf 세그먼트 거리와 수학적으로 동치임을 확인. `max_joint_speed_rad_s`는 `imitation.teleop`의 실기 한계(4.8)를 재사용, 신규 11개 포함 37개 planning 테스트 통과
- **Next**: MP-0013 벤치마크 하네스(PR 대기열이 늘고 있어 우선순위 상향), PR #1(MP-0005)·PR #2(MP-0006) 리뷰/병합, P3 실행 연결(MP-0008)
- **Full**: [journal/2026-08/31-11-p2-time-parameterize.md](journal/2026-08/31-11-p2-time-parameterize.md)


## 2026-08-31 07:17 — nullspace-regularization-natural-posture
- **Pick**: 사용자 지적 — 목적지엔 가지만 팔 자세가 부자연스러움, 매니퓰레이터 모션 플래닝을 보통 이렇게 푸는지 질문 + hydrax(sampling MPC) 참조 가능성 문의
- **Outcome**: position-only IK의 남는 4개 자유도에 표준 nullspace redundancy resolution(현재 관절값에 최대한 가깝게 유지) 추가. 분석해보니 "부자연스러운" 사례 하나는 실제로 가장 가까운 해가 장애물과 충돌해서 정당하게 크게 재배치된 것 — 정칙화는 이런 경우 충돌 회피를 우선한다(의도된 동작). hydrax는 RRT 대체가 아니라 P3(정식 실행) 보완 후보로 조사만 하고 구현은 사용자 확인 대기
- **Next**: MP-0006 시간 파라미터화, MP-0013 벤치마크 하네스. hydrax 통합은 사용자 확인 후 TODO 등록
- **Full**: [journal/2026-08/31-07-nullspace-regularization-natural-posture.md](journal/2026-08/31-07-nullspace-regularization-natural-posture.md)


## 2026-08-30 22:16 — drop-orientation-constraint-from-interactive-ik
- **Pick**: 사용자 버그 리포트 — 인터랙티브 모드에서 "IK가 수렴하지 않았습니다"가 반복됨, 모션 플래닝이 이걸 풀어주는 게 아니냐는 질문
- **Outcome**: 세션 시작 자세를 고정한 채 위치+자세를 동시에 풀던 게 원인 — 보고된 실패 지점은 자세 제약 없이 위치만 풀면 즉시 수렴+충돌 없음. `_ik_attempt`/`_solve_valid_ik`를 position-only로 재작성. IK 수렴 실패와 모션 플래닝은 다른 층의 문제라는 설명도 정리(목표 configuration 자체가 없으면 플래닝이 개입할 여지가 없음)
- **Next**: MP-0006 시간 파라미터화, MP-0013 벤치마크 하네스. P4(planning.goals) 설계에 이 발견 반영할 것
- **Full**: [journal/2026-08/30-22-16-drop-orientation-constraint-from-interactive-ik.md](journal/2026-08/30-22-16-drop-orientation-constraint-from-interactive-ik.md)


## 2026-08-30 22:02 — fix-marker-render-below-floor
- **Pick**: 사용자 버그 리포트 — 인터랙티브 모드의 노란 목표 마커가 바닥 밑에 보임
- **Outcome**: `mocap_pos`만 쓰고 `mj_forward`를 안 불러서 렌더링용 `data.xpos`가 컴파일 시점 기본값(월드 원점, 바닥보다 아래)에 남아있던 버그. 마커 초기 배치 직후 `mj_forward` 호출 추가, 실제 뷰어에서 `data.xpos`를 직접 읽어 바닥 위(z=0.99 vs 0.148)임을 확인
- **Next**: MP-0006 시간 파라미터화, MP-0013 벤치마크 하네스
- **Full**: [journal/2026-08/30-22-fix-marker-render-below-floor.md](journal/2026-08/30-22-fix-marker-render-below-floor.md)


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
