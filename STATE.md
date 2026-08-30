# Research State — auto-generated each cycle

_Last updated: 2026-08-30 14:00 KST · cycle bootstrap_

## North star distance

P0 기반(관절 공간 추상화 + 충돌 검사기)이 사람 손으로 막 구현되었다. RRT-Connect
코어는 아직 없다. 북극성(캔 분류 장면 오른팔 충돌-없는 경로 계획+실행)까지는
P1~P4 네 단계가 남았다.

## Current bottleneck

RRT-Connect 코어(`planning/rrt_connect.py`)가 없어 P1이 시작되지 못했다. MP-0002가
다음 executor cycle의 최우선 후보다.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|

## Recent learnings (last 3 cycles)

- (부트스트랩) 상자 geom(`target_bin_collision` class)은 raw 모델에서
  `contype=2 conaffinity=0`이라 오른팔과 충돌하지 않는다. `enable_task_collisions`가
  이를 승격시킨다 — 플래너의 충돌 검사기는 이 승격된 상태를 복사해야 하며,
  생성 시 가시성 가드로 이를 강제한다.

## Next claude-actionable

1. **MP-0002** RRT-Connect core — P0 기반 위에서 바로 시작 가능
2. **MP-0003** 무충돌 경로 속성 시험 — MP-0002와 병행 설계 가능
3. **MP-0018** `aggregate_results.py` — 벤치마크가 생기기 전에 먼저 준비해도 무해

## Next user-blocked

1. **MP-0020** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만 가능)
2. **MP-0004** can-sort 10 seed 성공률 측정 — RRT-Connect 완성 후 사람이 결과 확인

## Cycles to date

0 (부트스트랩 — 사람이 직접 P0을 구현하고 자동화를 배선한 첫 커밋)
