# Research State — auto-generated each cycle

_Last updated: 2026-09-04 · cycle p3-execution-module_

## North star distance

P0·P1이 완료·병합됐다. **PR #1(shortcut)·#2(time_parameterize)를 이번
cycle에 사용자 승인으로 병합**해 `main`이 실제로 `shortcut_path`/
`time_parameterize`/`Trajectory`를 갖는다. P2 나머지 두 조각(데모 실행
경로 연결 MP-0023, CHOMP류 궤적 최적화 MP-0024)과 P5(RRT* 대안 플래너
MP-0016), P4(벤치마크 하네스+P1 성공률 MP-0013/MP-0004)는 구현·테스트
완료 상태로 리뷰 대기 중.

**P3(정식 실행 모듈)도 이번 cycle에 착수·완료됐다** —
`planning/execution.py`의 `follow_trajectory`(PR #8, MP-0008/0009).
`ArmTorqueController` 토크만으로 재생하는 폐루프 함수이고, 매 표본마다
`ArmCollisionChecker.is_valid`로 재확인해 "침투 없음"을 직접 검증한다.
실측(seed 4개): 최종 site 오차 0.07~0.09mm(PRD 목표 5mm 대비 60배 여유),
침투 0건 — velocity feedforward 없이도 여유 있게 충족해 이번 스코프에
안 넣기로 한 연구 노트의 판단이 확인됐다.

**여덟 PR(#1~#8) 중 여섯(#3/#4/#5/#7/#8, #1·#2는 방금 병합)이 리뷰
대기 중**이다. 북극성까지는 P4 나머지(Cartesian goal 성공률) → 사용자가
지적한 나머지 두 한계(IK 실패, 연속 동작)가 남는다.

## Current bottleneck

**PR #3/#4/#5/#7/#8 사람 리뷰/병합 대기** — 다섯 다 테스트 통과·구현
완료 상태다. **PR #4·#5는 PR #1/#2 병합 여파로 `__init__.py`에서 새
충돌이 생겼다** — 병합 시 export 합집합으로 수동 해결 필요(이번 cycle에
PR #2에서 같은 패턴을 이미 두 번 해결한 적 있어 절차는 확립돼 있다).

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p2-demo-natural-motion | 2026-09-03 | MP-0023 데모 실행 경로 연결, PR #3 리뷰 대기 | 1 |
| planning/p5-rrt-star-planner | 2026-09-03 | MP-0016 RRT* 대안 플래너, PR #4 리뷰 대기(`__init__.py` 충돌 발생) | 1 |
| planning/chomp-posture-smoothing | 2026-09-03 | MP-0024 CHOMP류 궤적 최적화, PR #5 리뷰 대기(`__init__.py` 충돌 발생) | 1 |
| planning/p4-benchmark-harness | 2026-09-03 | MP-0013+MP-0004 벤치마크 하네스+첫 측정, PR #7 리뷰 대기 | 1 |
| planning/p3-execution-module | 2026-09-04 | MP-0008+MP-0009 P3 실행 모듈, PR #8 리뷰 대기 | 0 |

## Recent learnings (last 3 cycles)

- **공유 워킹 디렉토리에서 `git checkout -b`는 실패해도 조용하다 — 반드시
  직후에 `git branch --show-current`로 확인해야 한다.** 이 세션에서 같은
  실수가 세 번 반복됐다(마지막 두 번은 이번 cycle). 실패 메시지("이미
  존재함")를 놓치고 다음 명령을 실행하면 의도와 다른 브랜치(보통 `main`)에
  작업하게 된다. `git reset --hard`/`checkout`류 파괴적 명령 전에는 항상
  `git status`를 먼저 봐야 한다는 기존 규율도 이번에 한 번 어겼다가
  대화 컨텍스트로 복구했다 — 시스템 프롬프트 규율이라고 자동으로
  지켜지는 게 아니라 매번 의식적으로 확인해야 한다.
- **`checker.is_valid`를 재생 후처리에도 재활용하면 "침투 없음"을 계획
  때와 같은 계약으로 검증할 수 있다** — 새 contact 필터링 로직을 안
  만들어도, 계획·실행 두 층에서 "충돌"의 정의가 어긋나는 위험도 없앤다.
- **관절 간 결합이 없는 비용+제약이면 D개의 독립 1-D QP가 1개의 D·K차원
  QP보다 낫다.** CHOMP류 가속도 최소화를 관절별 7개 작은 QP로 풀었다.
- **단일 트리 sampling 플래너의 하이퍼파라미터는 bidirectional 플래너의
  기본값을 그대로 물려받으면 안 된다.** RRT*에 RRT-Connect의
  `goal_bias=0.1`을 그대로 썼다가 실제 장면에서 목표에 못 닿았다.

## Next claude-actionable

1. PR #3/#4/#5/#7/#8이 병합되면 `benchmark_planning.py`에
   `--planner`/`--postprocess` 플래그를 추가해 MP-0007/MP-0017 비교
   벤치마크로 확장.
2. 사용자가 우선순위를 정하면 나머지 두 한계(IK 실패 개선 — `MP-0011`
   `planning/goals.py` 스마트 시딩; 연속 동작 부드러움 — `MP-0021`
   hydrax/MPPI와 연관) 중 하나를 이어서 진행.
3. `MP-0010` `docs/guide/motion-planning.md` — P3까지 반영해 갱신.

## Next user-blocked

1. **PR #3/#4/#5/#7/#8 사람 리뷰/병합** — PR #4·#5는 `__init__.py` 충돌
   해결이 필요(export 합집합으로 수동 병합, 절차는 이번 cycle에 확립됨).
2. **MP-0020** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만 가능).

## Cycles to date

14 (2026-08-30~09-04 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
반복/트리 시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스
목표+버그 수정 3건, nullspace 정칙화+hydrax 조사, 데모 실행 경로에 shortcut+
시간 파라미터화 연결, RRT* 대안 플래너, CHOMP류 궤적 최적화 후처리, 벤치마크
하네스+P1 성공률 첫 측정, PR #1/#2 병합+P3 실행 모듈; 자율 루프: shortcut
평활화, 시간 파라미터화)
