# Research State — auto-generated each cycle

_Last updated: 2026-08-30 15:35 KST · cycle p1-rrt-connect-plus-demo_

## North star distance

P0(관절공간 추상화 + 충돌 검사기)와 P1(RRT-Connect 코어)이 모두 구현·검증되었다.
실제 can-sort 장면에서 계획한 경로를 `ArmTorqueController`로 재생해 목표에
수렴하는 것까지 확인했다(데모 스크립트, 정식 P3는 아직 아님). 북극성까지는
P2(평활화·시간화) → P3(정식 실행 모듈) → P4(Cartesian goal·벤치마크) 세 단계가
남았다.

## Current bottleneck

P2(shortcut 평활화 + 시간 파라미터화)가 없어 현재 경로는 waypoint를 곧바로
"수렴할 때까지 대기"하는 방식으로만 재생 가능하다(데모 스크립트의 임시방편).
정식 시간 파라미터화가 있어야 관절 속도 상한을 지키는 매끄러운 궤적을 만들 수
있다. MP-0005/MP-0006이 다음 최우선 후보다.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|

## Recent learnings (last 3 cycles)

- **CONNECT 버그**: 최초 `_connect` 구현이 target을 향해 반복 확장하는 대신
  방금 추가한 노드를 향해 확장하는 오류가 있었다. 무충돌 경로 속성 시험
  (20 seed)이 이를 즉시 잡아냈다 — property test가 실제로 버그를 잡은 사례.
- **실행 재생 버그**: `--execute` 데모에서 live `MjData`를 planner의 `start`
  configuration으로 먼저 맞추지 않고 재생을 시작해, 여전히 상자와 겹치는
  `home` 키프레임에서 출발해 수렴하지 못했다. 시작 상태 동기화를 추가하고
  회귀 시험(`test_plan_then_execute_converges`)으로 고정했다.
- **테스트 설계 교훈**: 순수 numpy 성질 시험에서 "완전히 막힌" slab 장애물은
  RRT-Connect가 원리적으로 풀 수 없다(로컬 검사 해상도 이하로 벽을 통과할 수
  없음). "틈이 있는 벽" 형태로 바꿔야 진짜 우회 경로 시험이 된다.

## Next claude-actionable

1. **MP-0005** shortcut 평활화(`planning/shortcut.py`)
2. **MP-0006** `time_parameterize` 사다리꼴 속도 프로파일
3. **MP-0018** `aggregate_results.py`는 이미 있음 — 벤치마크 하네스(MP-0013)와
   연결해 P1/P2 결과를 `results/*.tsv`에 남기는 작업

## Next user-blocked

1. **MP-0020** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만 가능)
2. **MP-0004** can-sort 10 seed 성공률 정식 측정 — 벤치마크 하네스(MP-0013)가
   먼저 있어야 TSV로 기록 가능. 데모 스크립트로는 이미 여러 seed에서 성공을
   비공식 확인함(`scripts/demo_plan_right_arm.py --seed N --execute`)

## Cycles to date

1 (2026-08-30 사람 주도: P0 부트스트랩 + P1 RRT-Connect 구현·버그 수정·실행 데모)
