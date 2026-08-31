# TODO — AIWORKER 오른팔 모션 플래닝

_이 파일이 작업 상태의 유일한 권위다. 사람과 cron 에이전트가 함께 수정한다._
_기계적 수정은 `scripts/todo_tool.py`를 쓴다 (표 정렬·ID 발급·중복 검사 포함)._

- Last update: `2026-08-31 11:03 KST`
- Open (Doing + Today + Blocked + Backlog): **18**
- Next ID: `MP-0022`

## Doing
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|
| MP-0005 | P1 | P2 | claude | [planner] shortcut 평활화 `planning/shortcut.py` | planning/p2-shortcut-smoothing | ☐ |
| MP-0006 | P1 | P2 | claude | [planner] `time_parameterize` 사다리꼴 속도 프로파일 | planning/p2-time-parameterize | ☐ |

## Today
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|

## Blocked
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|

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
| MP-0015 | P2 | P5 | claude | [research] RRT* rewiring + informed sampling 문헌조사 |  | ☐ |
| MP-0016 | P2 | P5 | claude | [planner] `planning/rrt_star.py` 초안 |  | ☐ |
| MP-0017 | P3 | P5 | claude | [bench] RRT-Connect vs RRT* 50 seed 비교표 → `RESULTS.md` |  | ☐ |
| MP-0018 | P2 | P0 | claude | [infra] `scripts/aggregate_results.py` — TSV→RESULTS.md 집계기 |  | ☐ |
| MP-0019 | P2 | P0 | claude | [infra] `scripts/todo_tool.py` 단위 테스트(파싱·왕복) |  | ☐ |
| MP-0020 | P3 | P0 | user | (user) Telegram 봇 생성 후 `scripts/telegram_setup.sh` 실행 확인 |  | ☑ |
| MP-0021 | P2 | P3 | user | [research] hydrax(GPU sampling MPC) 통합 검토 — P3 실행 레이어에서 RRT-Connect 경로를 매끄럽게 추종하는 저수준 제어로. JAX/MJ... |  | ☐ |

## Done
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|
| MP-0003 | P0 | P1 | claude | [planner] `tests/test_planning_core.py` — 무충돌 경로 속성 시험 20 seed |  | ☐ |
| MP-0002 | P0 | P1 | claude | [planner] RRT-Connect core: `_Tree`, extend/connect, goal-bias sampler |  | ☐ |
| MP-0001 | P0 | P0 | claude | [planner] `RightArmSpace` + `ArmCollisionChecker` + `EdgeChecker` (P0 기반) |  | ☐ |

> Mirror는 없음 — 이 파일 자체가 canonical이다. 갱신: `python3 scripts/todo_tool.py check`
