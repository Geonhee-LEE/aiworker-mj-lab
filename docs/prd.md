# Product Requirements — 오른팔 모션 플래닝

## 1. 북극성 (North Star)

> **AIWORKER 오른팔이 캔 분류 작업대 위에서 정적 장애물(상자, 테이블, 왼팔)을
> 피해 충돌 없는 관절 경로를 계획하고 그대로 실행한다.**

### "완벽"의 운영적 정의 (P4 벤치마크 이후 확정)

| 차원 | 1차 임계 (가설) | 측정 방법 |
|---|---|---|
| 성공률 | 캔 분류 장면 시드 50개 중 ≥ 90% | `scripts/benchmark_planning.py` |
| 계획 시간 | 중앙값 ≤ 500 ms | 같은 벤치마크의 `planning_time_s` |
| 경로 품질 | shortcut 후 원경로 대비 길이 30%↓ | `path_length_rad` vs `smoothed_length_rad` |
| 실행 안전성 | 재생 중 최소 clearance ≥ `whole_body_ik.collision_safe_distance_m` | `tests/test_planning_execution.py` |

### 비-목표 (Non-Goals)

- 왼팔·베이스·리프트 계획 (P6 이후 후보, 이 PRD 범위 밖)
- 동적 장애물 회피(움직이는 물체) — 그 몫은 여전히 whole-body IK의 CBF
- OMPL/cuRobo 등 외부 플래닝 라이브러리 도입 — 저장소 원칙상 직접 구현
- 학습 기반 플래너(러닝 기반 샘플링, 신경망 유도) — 이 로드맵에서는 다루지 않음

## 2. Phased Roadmap

| Phase | 결과물 | Exit criterion | 상태 |
|---|---|---|---|
| P0 | `RightArmSpace` + `ArmCollisionChecker` + `EdgeChecker` | validity 테스트 통과, live `MjData` 무오염 확인 | ✅ 완료·병합 |
| P1 | RRT-Connect 코어 | can-sort 50 seed 성공률 ≥ 90%, 충돌 경로 미반환 속성 시험 통과 | ✅ 코어 완료·병합. 정식 50-seed TSV 측정(MP-0004)은 벤치마크 하네스 대기 |
| P2 | shortcut 평활화 + 시간 파라미터화 + 자세 매끄러움 후처리 | 경로 길이 중앙값 30%+ 단축, 속도/가속도 상한 위반 0 | 🟡 구현·테스트 완료, 리뷰 대기(PR #1 shortcut, #2 time_parameterize, #3 데모 연결, #5 CHOMP 자세 후처리) |
| P3 | MuJoCo 실행 연결 | headless 재생 시 침투 없음, 최종 site 오차 ≤ 5 mm | ⬜ 미착수 — `planning.execution` 모듈 없음. 데모 스크립트가 데모/디버그용 재생만 제공 |
| P4 | Cartesian goal + 벤치마크 하네스 | pose 목표 성공률 ≥ 85%, 벤치마크 2분 이내 완주 | ⬜ 미착수 |
| P5 | RRT* / informed sampling 비교 연구 | 동일 seed에서 baseline 대비 경로 길이·시간 trade-off 표 산출 | 🟡 `planning.rrt_star` 구현·테스트 완료, 리뷰 대기(PR #4). 정식 50-seed 비교표(MP-0017)는 벤치마크 하네스 대기 |
| P6 | (비-목표 재검토) 왼팔·양팔 협조 계획 후보 | 사람 결정 대기 | ⬜ 미착수 |

**자세 매끄러움 후처리를 P2로 편입한 이유**: 원래 로드맵에 없던 항목이다.
사용자가 실제 데모(`--interactive`)로 확인한 세 가지 남은 한계 — IK 계산
실패, 연속 동작 부자연스러움, **팔 자세의 기괴함** — 중 마지막 것은
shortcut/시간 파라미터화만으로는 풀리지 않는 별도 문제였다(§3
R-F-004b 참고). 나머지 두 한계는 각각 R-F-009(계획), R-F-010(계획)로
로드맵에 새로 추가했다 — 아직 착수 전이다.

## 3. 기능 요구 (Functional Requirements)

### R-F-001 오른팔 7-DOF 관절공간 추상화

`planning.arm_state.RightArmSpace` — 관절 이름·id·qpos 주소·범위를 한 번 계산해
샘플링·보간·클리핑 API를 제공한다. MuJoCo 모델 없이도(`from_limits`) 단위 테스트가
가능해야 한다.

### R-F-002 충돌 유효성 검사기

`planning.collision_state.ArmCollisionChecker` — live `MjData`를 절대 변경하지 않고
scratch 모델·데이터로 boolean `is_valid(q)`와 exact `clearance(q)`를 제공한다.

### R-F-003 RRT-Connect 플래너

`planning.rrt_connect.plan_rrt_connect` — 두 트리 확장/연결, goal-bias 샘플링,
seed 결정론, 시간 예산과 반복 상한을 모두 갖는다.

### R-F-004 경로 후처리

shortcut 평활화(`planning.shortcut`)와 제어 주기 시간 파라미터화(`planning.trajectory`).

### R-F-004b 자세 매끄러움 후처리 (신규)

`planning.chomp.smooth_posture` — RRT-Connect/RRT*는 redundant DOF를 무작위로
표본화하므로, 경로 길이·시간과 무관하게 이웃 waypoint와 비교해 관절값이
튀는("기괴한") 지점이 남을 수 있다. shortcut 평활화(R-F-004)는 경로를
짧게 만들 뿐 매끄럽게 만들지는 않으므로 별도 요구사항으로 분리했다. CHOMP류
가속도(2차 차분) 최소화 QP로 시작·끝 waypoint를 고정한 채 내부 waypoint를
지역적으로 다듬는다. 콜리전 제약이 없는 QP이므로 trust region + 재검증 +
실패 시 원본 반환으로 "절대 무효 경로를 반환하지 않는다"(R-NF-005 인접
원칙)를 지킨다. `kinematics.optimization`의 기존 QP 유틸리티를 재사용하고
수정하지 않는다(§7 기존 자산 재사용 우선 원칙).

### R-F-005 실행 연결

`planning.execution.follow_trajectory` — 기존 `ArmTorqueController`로 궤적을 재생한다.
새 액추에이터 코드를 추가하지 않는다.

### R-F-006 벤치마크 하네스

`scripts/benchmark_planning.py` — seed 목록으로 N회 질의를 실행해 `results/*.tsv`에
append-only 행을 남긴다. 2분 이내 완주가 요구사항이다(자동화 에이전트가 돌릴 수 있어야 함).

### R-F-007 자율 R&D 루프

`scripts/prompts/auto_research.md` + cron — 하루 2회 TODO를 하나 골라 구현·검증·PR을 만들고
Telegram으로 보고한다. `docs/agents.md`에 상세 정의.

### R-F-008 TODO.md 단독 권위

Notion 등 외부 서비스 없이 `TODO.md` 파일이 작업 상태의 유일한 권위다.
`scripts/todo_tool.py`로 기계적으로만 수정한다.

### R-F-009 IK 목표 탐색 개선 (계획, 미착수)

사용자가 관찰: `--interactive` 데모에서 IK 계산이 자주 실패한다. 현재
`_solve_valid_ik`는 순수 무작위 재시도(`n_restarts=25`)에 의존해 성공률이
운에 좌우된다. `planning.goals`(기존 `MP-0011` 백로그)에서 목표 근처의
구조화된 시드(이전 성공 해, 거친 reachability 격자 등)로 개선한다.

### R-F-010 연속 동작 매끄러움 (계획, 미착수)

사용자가 관찰: 목표를 연속으로 바꾸며 조작할 때 매 목표 전환마다 속도가
0으로 끊긴다(계획→정지 재생→재계획 구조 때문). 두 층의 해법 후보가 있다:
(1) `planning.execution`(R-F-005/P3)이 이전 재생 속도를 이어받는 방식으로
설계, (2) receding-horizon 저수준 제어(`MP-0021`, hydrax/MPPI 계열) — P3
실행 레이어가 sampling-based 전역 계획과 별개로 담당. 어느 쪽을 먼저
할지는 P3 설계 시점에 결정한다.

## 4. 비기능 요구 (Non-Functional Requirements)

### R-NF-001 자율성

executor는 사람 개입 없이 조사→구현→테스트→PR까지 완결한다. 머지는 항상 사람.

### R-NF-002 비용/시간 제한

한 cycle ≤ 35분 wall clock. 시뮬레이션 실행 ≤ 2분(초과 시 사람에게 test request로 위임).

### R-NF-003 안전

`main`에 코드 직접 push 금지, 하드 리밋(§ `docs/agents.md`) 위반 시 조용히 거절.
기존 `src/ffw_sh5_grasp/{kinematics,control,imitation}` 수정은 별도 심의(Q-NNN) 필요.

### R-NF-004 관측성

모든 cron 실행은 로그 파일 + stdout 센티널(`EXECUTOR_DONE`/`EXECUTOR_SKIP` 등)을 남긴다.

### R-NF-005 재현성

모든 계획·벤치마크는 seed 기반으로 결정론적이어야 한다. `np.random.default_rng`만 쓴다.

## 5. 성공 지표 (Success Metrics)

### 단기 (P0-P1 마무리)

- [x] `ArmCollisionChecker`가 live `MjData`를 오염시키지 않음을 테스트로 증명
  (`tests/test_planning_collision.py::test_live_data_is_never_mutated`)
- [x] can-sort 장면에서 RRT-Connect가 seed 10개 중 10개 성공 — 합성 공간
  100/100 seed + 실제 장면 seeded 질의로 확인(`tests/test_planning_rrt_scene.py`).
  정식 TSV 벤치마크(seed 50개, MP-0004/MP-0013)는 아직 대기
- [ ] 자동화 cron 8종이 모두 최소 1회 수동 스모크 통과 — researcher/brief/
  executor/wrap은 `research/cron_activity.md` 로그로 반복 확인됨. curator·
  weekly_rollup·telegram_poll·urgent_agent는 미확인

### 중기 (P3-P4 마무리)

- [ ] 계획 궤적을 MuJoCo에서 실행해 최종 오차 5 mm 이내
- [ ] Cartesian goal 성공률 85% 이상
- [ ] 벤치마크가 자동화 루프에서 정기적으로 `results/`에 행을 남김

### 장기 (P5 마무리)

- [ ] RRT* 등 대안 플래너와의 정량 비교표를 `RESULTS.md`에 게시
- [ ] PR throughput ≥ 2/week, 사용자 평균 리뷰 시간 ≤ 20분/week

### Project (P6 검토)

- [ ] 왼팔·양팔 협조 계획으로 범위를 넓힐지 결정

## 6. 위험 + 완화 (Risks)

| 위험 | 영향 | 완화 |
|---|---|---|
| 상자 geom이 raw 모델에서 충돌 비활성 (`contype=2 conaffinity=0`) | 플래너가 상자를 관통하는 경로를 "안전"으로 오판 | `ArmCollisionChecker` 생성 시 `require_contact_geoms` 가시성 가드로 즉시 예외 |
| `config/default.yaml` 스키마 위반 | CI 전체 실패 | 모든 `planning.*` 키를 코드에서 리터럴 경로로 읽고 한글 주석 유지 |
| PR 큐 적체로 야간 알림 남발 | 사용자 수면 방해 | skip 시 Telegram 무음, PR 큐 상한 게이트 |
| 홈 Claude 설정의 `bypassPermissions`로 방어선 약화 | 의도치 않은 위험 명령 실행 | 하드 리밋 + `state_push.sh` 화이트리스트 + 프로젝트 deny 규칙 3중 방어 |
| 백로그 고갈로 executor가 매번 조용히 skip | 진척 정체를 사람이 못 알아챔 | researcher가 매일 후속 TODO 보충, 주간 롤업으로 가시화 |

## 7. 의사결정 헌법 (Decision Constitution)

- **북극성 정렬 우선** — 어떤 작업도 "이게 목표 거리를 어떻게 줄이나" 답 못하면 우선순위를 낮춘다.
- **단순성 기준** — `+50 LOC` 이상 추가 시 측정 가능한 이득 1개를 명시한다. 순수 삭제는 환영.
- **append-only 기록** — `results/*.tsv`, `journal/`, `research/feed.md`의 과거 행은 절대 수정하지 않는다.
- **기존 자산 재사용 우선** — `kinematics/collision.py`, `tree.py`, `solver.py`를 재구현하지 않는다.
- **양방향 동기화** — `TODO.md`와 실제 작업 상태(브랜치·PR)가 어긋나면 다음 cycle에서 즉시 정정한다.
- **live 시뮬레이션 상태 불가침** — 플래너는 caller의 `MjData`를 절대 변경하지 않는다.

## 8. 변경 관리

이 문서는 사람이 갱신한다. cron 에이전트가 임의로 수정하지 않는다. 단:

- `STATE.md`, `JOURNAL.md`, `RESULTS.md` — 에이전트가 매 cycle 재작성/append한다.
- `docs/agents.md`, `docs/skills.md` — Curator가 stale 감지 시 PR을 올리고 사람이 머지한다.

_Last manual update: 2026-09-03 KST_
