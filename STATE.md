# Research State — auto-generated each cycle

_Last updated: 2026-09-03 · cycle benchmark-harness_

## North star distance

P0(관절공간 추상화 + 충돌 검사기)와 P1(RRT-Connect 코어)이 모두 구현·검증됐다.
P2(경로 후처리)는 shortcut 평활화(MP-0005, PR #1), 시간 파라미터화(MP-0006,
PR #2), 데모 실행 경로 연결(MP-0023, PR #3), CHOMP류 궤적 최적화(MP-0024,
PR #5)까지 네 조각 모두 구현·테스트 완료. P5(RRT* 대안 플래너)도
착수·완료됐다(MP-0016, PR #4).

**P4 벤치마크 하네스(MP-0013)도 이번 cycle에 완성됐고, 바로 실제로 돌려
P1 성공률 측정(MP-0004)까지 같은 cycle에서 닫았다** — `scripts/
benchmark_planning.py`, PR #7. can-sort 실제 장면에서 50 seed×2 시나리오
(장애물 유/무) 모두 **성공률 100%(50/50)**, 계획 시간 중앙값 ~13ms(PRD 목표
500ms 대비 크게 여유), 전체 실행 각 ~1~1.4초(2분 예산 대비 여유 큼). 이제
`results/*.tsv`에 처음으로 실제 벤치마크 raw 데이터가 쌓였다.

**일곱 PR(#1~#7) 모두 사람 리뷰/병합 대기 중**이다. 북극성까지는 P3(정식
실행 모듈) → P4 나머지(Cartesian goal 성공률, MP-0007/0014/0017 비교
벤치마크) → 사용자가 지적한 나머지 두 한계(IK 실패, 연속 동작)가 남는다.

## Current bottleneck

**PR #1~#7(MP-0005/0006/0023/0016/0024/0013/0004) 사람 리뷰/병합 대기** —
일곱 다 테스트 통과·구현 완료 상태로 막혀 있다. PR 큐가 7개까지 늘었다 —
새 코드 작업보다 기존 PR 소화가 우선순위라는 경고가 이제 세 cycle째
반복되고 있다.

**로컬 main에 push 안 된 정당한 커밋 1개가 남아 있다** — 이번 cycle 작업
중 curator의 상태 커밋(`research/cron_activity.md`)을 raw `git push origin
main`으로 밀려다 auto-mode 안전 분류기에 막혔다(`state_push.sh`를 거치지
않은 게 원인으로 보임). 원격엔 영향 없지만, 사람이 직접 push하거나 다음
정상 state_push 시도 때 반영돼야 한다.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p2-shortcut-smoothing | 2026-08-30 21:04 KST | MP-0005 shortcut 평활화, PR #1 리뷰 대기 | 4 |
| planning/p2-time-parameterize | 2026-08-31 11:10 KST | MP-0006 time_parameterize, PR #2 리뷰 대기 | 3 |
| planning/p2-demo-natural-motion | 2026-09-03 | MP-0023 데모 실행 경로 연결, PR #3 리뷰 대기 | 0 |
| planning/p5-rrt-star-planner | 2026-09-03 | MP-0016 RRT* 대안 플래너, PR #4 리뷰 대기 | 0 |
| planning/chomp-posture-smoothing | 2026-09-03 | MP-0024 CHOMP류 궤적 최적화, PR #5 리뷰 대기 | 0 |
| planning/p4-benchmark-harness | 2026-09-03 | MP-0013+MP-0004 벤치마크 하네스+첫 측정, PR #7 리뷰 대기 | 0 |

## Recent learnings (last 3 cycles)

- **동시에 도는 자율 루프와 워킹 디렉토리를 공유할 때는 `git checkout -b`
  직후에도 실제로 그 브랜치에 있는지 검증해야 한다.** 이번 cycle에 브랜치
  생성 직후 뭔가(추정: 동시에 돈 curator)가 working tree를 `main`으로
  되돌려놔서, 그걸 모른 채 커밋해 main에 코드가 두 번 잘못 들어갔다(둘 다
  push 전에 발견, 안전 브랜치+cherry-pick으로 완전 복구, 원격 영향 없음).
  브랜치 작업 중에는 `git branch --show-current`나 커밋 직후 `git log
  --oneline -1`으로 확인하는 습관이 필요하다.
- **`git push origin main`은 항상 `state_push.sh`를 거쳐야 한다** —
  화이트리스트 검증뿐 아니라 auto-mode 분류기가 raw main push 자체를
  별도로 막는 것으로 보인다(이번 cycle에 실제로 두 번 거부당함).
- **관절 간 결합이 없는 비용+제약이면 D개의 독립 1-D QP가 1개의 D·K차원
  QP보다 낫다.** CHOMP류 가속도 최소화를 관절별 7개 작은 QP로 풀어
  `kinematics/optimization.py`를 전혀 수정하지 않고 재사용했다.
- **단일 트리 sampling 플래너의 하이퍼파라미터는 bidirectional 플래너의
  기본값을 그대로 물려받으면 안 된다.** RRT*에 RRT-Connect의 `goal_bias=0.1`을
  그대로 썼다가 실제 장면에서 목표에 못 닿는 경우가 있었다.

## Next claude-actionable

1. 사용자가 우선순위를 정하면 나머지 두 한계(IK 실패 개선 — `MP-0011`
   `planning/goals.py` 스마트 시딩; 연속 동작 부드러움 — 실행/제어 층,
   `MP-0021` hydrax/MPPI와 연관) 중 하나를 이어서 진행.
2. **MP-0008** `planning/execution.py` — `ArmTorqueController` 연결(P3).
   MP-0023에서 발견한 "재생 컨트롤러 대역폭 ≠ 하드웨어 스펙" 구분을 설계에
   반영해야 한다.
3. PR #1(shortcut)·PR #4(RRT*)가 병합되면 `benchmark_planning.py`에
   `--planner`/`--postprocess` 플래그를 추가해 MP-0007/MP-0017 비교
   벤치마크로 확장 — 지금은 존재하지 않는 모듈이라 미리 만들지 않았다.

## Next user-blocked

1. **PR #1~#7 사람 리뷰/병합** — 일곱 다 테스트 통과, 병합 대기 중. PR
   큐가 계속 늘고 있어 최우선순위.
2. **로컬 main의 curator 상태 커밋 push** — auto-mode 분류기가 이번
   cycle에 raw push를 두 번 막았다. 사람이 직접 push하거나 확인 필요.
3. **MP-0020** Telegram 봇 생성 및 `telegram_setup.sh` 실행 (사람만 가능).

## Cycles to date

13 (2026-08-30~09-03 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
반복/트리 시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스
목표+버그 수정 3건, nullspace 정칙화+hydrax 조사, 데모 실행 경로에 shortcut+
시간 파라미터화 연결, RRT* 대안 플래너, CHOMP류 궤적 최적화 후처리, 벤치마크
하네스+P1 성공률 첫 측정; 자율 루프: shortcut 평활화, 시간 파라미터화)
