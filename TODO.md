# TODO — AIWORKER 오른팔 모션 플래닝

_이 파일이 작업 상태의 유일한 권위다. 사람과 cron 에이전트가 함께 수정한다._
_기계적 수정은 `scripts/todo_tool.py`를 쓴다 (표 정렬·ID 발급·중복 검사 포함)._

- Last update: `2026-09-03 KST`
- Open (Doing + Today + Blocked + Backlog): **21**
- Next ID: `MP-0026`

## Doing
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|

## Today
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|

## Blocked
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|
| MP-0005 | P1 | P2 | claude | [stuck][planner] shortcut 평활화 — 구현·테스트 완료, PR #1 사람 리뷰 대기 | planning/p2-shortcut-smoothing | ☐ |
| MP-0006 | P1 | P2 | claude | [stuck][planner] `time_parameterize` — 구현·테스트 완료, PR #2 사람 리뷰 대기 | planning/p2-time-parameterize | ☐ |
| MP-0023 | P1 | P2 | claude | [planner] shortcut+시간 파라미터화를 데모 실행 경로에 연결 — 구현·테스트 완료, PR #3 사람 리뷰 대기 | planning/p2-demo-natural-motion | ☐ |
| MP-0016 | P2 | P5 | claude | [planner] `planning/rrt_star.py` 초안 — 구현·테스트 완료, PR #4 사람 리뷰 대기 | planning/p5-rrt-star-planner | ☐ |
| MP-0024 | P1 | P2 | claude | [planner] CHOMP류 궤적 최적화 후처리(`planning/chomp.py`) — 구현·테스트 완료, PR #5 사람 리뷰 대기 | planning/chomp-posture-smoothing | ☐ |

## Backlog
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|
| MP-0004 | P1 | P1 | claude | [planner] can-sort 장면 seed 10개 RRT-Connect 성공률 측정 |  | ☑ |
| MP-0007 | P1 | P2 | claude | [bench] shortcut 전/후 경로 길이 비교 속성 시험 |  | ☐ |
| MP-0008 | P1 | P3 | claude | [planner] `planning/execution.py` — `ArmTorqueController` 연결 |  | ☐ |
| MP-0009 | P1 | P3 | claude | [planner] `test_planning_execution.py` 침투·오차 검증 |  | ☑ |
| MP-0010 | P2 | P3 | claude | [docs] `docs/guide/motion-planning.md` 작성 + mkdocs nav 등록 |  | ☐ |
| MP-0011 | P1 | P4 | claude | [planner] `planning/goals.py` Cartesian pose goal → IK 시드 다중 재시도 |  | ☐ |
| MP-0012 | P1 | P4 | claude | [planner] `tests/offline_pose_ik.py`를 `planning.goals`로 위임(중복 제거) |  | ☐ |
| MP-0013 | P0 | P4 | claude | [bench] `scripts/benchmark_planning.py` 작성 — TSV append, 2분 예산 |  | ☐ |
| MP-0014 | P2 | P4 | claude | [bench] pose goal 20 seed 성공률 측정 |  | ☑ |
| MP-0017 | P3 | P5 | claude | [bench] RRT-Connect vs RRT* 50 seed 비교표 → `RESULTS.md` |  | ☐ |
| MP-0018 | P2 | P0 | claude | [infra] `scripts/aggregate_results.py` — TSV→RESULTS.md 집계기 |  | ☐ |
| MP-0019 | P2 | P0 | claude | [infra] `scripts/todo_tool.py` 단위 테스트(파싱·왕복) |  | ☐ |
| MP-0020 | P3 | P0 | user | (user) Telegram 봇 생성 후 `scripts/telegram_setup.sh` 실행 확인 |  | ☑ |
| MP-0021 | P2 | P3 | user | [research] hydrax(GPU sampling MPC) 통합 검토 — P3 실행 레이어에서 RRT-Connect 경로를 매끄럽게 추종하는 저수준 제어로. JAX/MJ... |  | ☐ |
| MP-0022 | P2 | P4 | claude | [research] aggregate_results.py에 성공률 Wilson 신뢰구간 계산 추가 (MP-0013/0018 완료 후) |  | ☐ |
| MP-0025 | P2 | P5 | user | [research] VAMP-MR(SIMD 가속 multi-arm 샘플링 플래너) 통합 검토 — 10-100x 계획/후처리/실행 가속. PRD Non-Goal(외부 플래닝 라이브러리, OMPL 명시 제외)과 충돌 + sudo 시스템 패키지가 필요한 C++ 빌드(cmake/ninja/boost/ompl/protobuf/tbb) 신규 도입 — research/2026-09/003-vamp-mr.md 참고 |  | ☐ |

## Done
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|
| MP-0015 | P2 | P5 | claude | [research] RRT* rewiring + informed sampling 문헌조사 (rrt_star.py 설계 과정으로 충족, journal/2026-09/03-rrt-star-planner.md 참고) |  | ☐ |
| MP-0003 | P0 | P1 | claude | [planner] `tests/test_planning_core.py` — 무충돌 경로 속성 시험 20 seed |  | ☐ |
| MP-0002 | P0 | P1 | claude | [planner] RRT-Connect core: `_Tree`, extend/connect, goal-bias sampler |  | ☐ |
| MP-0001 | P0 | P0 | claude | [planner] `RightArmSpace` + `ArmCollisionChecker` + `EdgeChecker` (P0 기반) |  | ☐ |

> Mirror는 없음 — 이 파일 자체가 canonical이다. 갱신: `python3 scripts/todo_tool.py check`
