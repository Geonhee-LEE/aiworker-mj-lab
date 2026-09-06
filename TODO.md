# TODO — AIWORKER 오른팔 모션 플래닝

_이 파일이 작업 상태의 유일한 권위다. 사람과 cron 에이전트가 함께 수정한다._
_기계적 수정은 `scripts/todo_tool.py`를 쓴다 (표 정렬·ID 발급·중복 검사 포함)._

- Last update: `2026-09-06 21:07 KST`
- Open (Doing + Today + Blocked + Backlog): **12**
- Next ID: `MP-0032`

## Doing
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|

## Today
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|

## Blocked
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|
| MP-0007 | P1 | P2 | claude | [bench] shortcut 전/후 경로 길이 비교 — 실측 완료(50 seed, 중앙값 6.82% 감소, 증가 0건), PR #13 사람 리뷰 대기 | planning/p5-planner-comparison | ☐ |
| MP-0017 | P3 | P5 | claude | [bench] RRT-Connect vs RRT* 50 seed 비교표 — 실측 완료: 통계적으로 유의한 경로 품질 차이 없음(Wilcoxon p≈0.86), RRT*가 ~1... | planning/p5-planner-comparison | ☐ |
| MP-0011 | P1 | P4 | claude | [planner] `planning/goals.py` Cartesian pose goal → IK 시드 다중 재시도 — 구현·테스트 완료(83개 통과), PR #14 사람 리... | planning/p4-cartesian-pose-goal-ik-seed | ☐ |

## Backlog
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|
| MP-0012 | P1 | P4 | claude | [planner] `tests/offline_pose_ik.py`를 `planning.goals`로 위임(중복 제거) |  | ☐ |
| MP-0014 | P2 | P4 | claude | [bench] pose goal 20 seed 성공률 측정 |  | ☑ |
| MP-0019 | P2 | P0 | claude | [infra] `scripts/todo_tool.py` 단위 테스트(파싱·왕복) |  | ☐ |
| MP-0020 | P3 | P0 | user | (user) Telegram 봇 생성 후 `scripts/telegram_setup.sh` 실행 확인 |  | ☑ |
| MP-0021 | P2 | P3 | user | [research] hydrax(GPU sampling MPC) 통합 검토 — P3 실행 레이어에서 RRT-Connect 경로를 매끄럽게 추종하는 저수준 제어로. JAX/MJ... |  | ☐ |
| MP-0022 | P2 | P4 | claude | [research] aggregate_results.py에 성공률 Wilson 신뢰구간 계산 추가 (MP-0013/0018 완료 후) |  | ☐ |
| MP-0025 | P2 | P5 | user | [research] VAMP-MR(SIMD 가속 multi-arm 샘플링 플래너) 통합 검토 — 10-100x 계획/후처리/실행 가속. PRD Non-Goal(외부 플래닝 라... |  | ☐ |
| MP-0030 | P2 | P7 | user | [design] P7 Tier 2(결합형 whole-body 플래너) 착수 여부 결정 — 타당성 평가 완료(PR #12), 검증 시나리오(합성 MJCF) 준비 필요 |  | ☐ |
| MP-0031 | P2 | P5 | user | [research] RRT*가 장애물 없는 시나리오·15초 예산에서 RRT-Connect 대비 경로 품질 이득이 없음을 실측(PR #13, Wilcoxon p≈0.86) — ... |  | ☐ |

## Done
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|
| MP-0029 | P2 | P7 | claude | [docs] P7.0/P7.1 가이드 문서화 + P7 Tier 2 타당성 평가 — PR #12 병합 완료(main에 실제로 있음) | docs/p7-mobile-manipulator-guide | ☐ |
| MP-0028 | P2 | P1 | claude | [research] EdgeChecker/ArmCollisionChecker에 safety-certificate 스타일 캐싱 도입 여부 판단 — 바로 구현하지 않고 `stat... | planning/p1-safety-certificate-profiling | ☐ |
| MP-0027 | P1 | P7 | claude | [planner] P7.1 `planning/base_pose.py` — PR #11 병합 완료(main에 실제로 있음) | planning/p7-1-base-pose | ☐ |
| MP-0026 | P1 | P7 | claude | [planner] P7.0 `planning/reachability.py` reachability map — PR #10 병합 완료(main에 실제로 있음) | planning/p7-reachability-map | ☐ |
| MP-0024 | P1 | P2 | claude | [planner] CHOMP류 궤적 최적화 후처리(`planning/chomp.py`) — PR #5 병합 완료(main에 실제로 있음) | planning/chomp-posture-smoothing | ☐ |
| MP-0023 | P1 | P2 | claude | [planner] shortcut+시간 파라미터화를 데모 실행 경로에 연결 — PR #3 병합 완료(main에 실제로 있음) | planning/p2-demo-natural-motion | ☐ |
| MP-0018 | P2 | P0 | claude | [infra] `scripts/aggregate_results.py` — TSV→RESULTS.md 집계기 |  | ☐ |
| MP-0016 | P2 | P5 | claude | [planner] `planning/rrt_star.py` 초안 — PR #4 병합 완료(main에 실제로 있음) | planning/p5-rrt-star-planner | ☐ |
| MP-0015 | P2 | P5 | claude | [research] RRT* rewiring + informed sampling 문헌조사 (rrt_star.py 설계 과정으로 충족, journal/2026-09/03-rrt... |  | ☐ |
| MP-0013 | P0 | P4 | claude | [bench] `scripts/benchmark_planning.py` — PR #7 병합 완료(main에 실제로 있음) | planning/p4-benchmark-harness | ☐ |
| MP-0010 | P2 | P3 | claude | [docs] `docs/guide/motion-planning.md` 작성 + mkdocs nav 등록 |  | ☐ |
| MP-0009 | P1 | P3 | claude | [planner] `test_planning_execution.py` 침투·오차 검증 — PR #8에 포함, 병합 완료 | planning/p3-execution-module | ☑ |
| MP-0008 | P1 | P3 | claude | [planner] `planning/execution.py` — PR #8 병합 완료(main에 실제로 있음), 실측 site 오차 0.07~0.09mm | planning/p3-execution-module | ☐ |
| MP-0006 | P1 | P2 | claude | [planner] `time_parameterize` 사다리꼴 속도 프로파일 — PR #2 병합 완료(main에 실제로 있음) | planning/p2-time-parameterize | ☐ |
| MP-0005 | P1 | P2 | claude | [planner] shortcut 평활화 `planning/shortcut.py` — PR #1 병합 완료(main에 실제로 있음) | planning/p2-shortcut-smoothing | ☐ |
| MP-0004 | P1 | P1 | claude | [planner] can-sort 장면 RRT-Connect 성공률 측정 — 50 seed×2 시나리오 100% 성공, PR #7에 포함, 병합 완료 | planning/p4-benchmark-harness | ☑ |
| MP-0003 | P0 | P1 | claude | [planner] `tests/test_planning_core.py` — 무충돌 경로 속성 시험 20 seed |  | ☐ |
| MP-0002 | P0 | P1 | claude | [planner] RRT-Connect core: `_Tree`, extend/connect, goal-bias sampler |  | ☐ |
| MP-0001 | P0 | P0 | claude | [planner] `RightArmSpace` + `ArmCollisionChecker` + `EdgeChecker` (P0 기반) |  | ☐ |

> Mirror는 없음 — 이 파일 자체가 canonical이다. 갱신: `python3 scripts/todo_tool.py check`
