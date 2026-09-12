# Research State — auto-generated each cycle

_Last updated: 2026-09-12 · cycle p4-offline-pose-ik-delegate_

## North star distance

**P4(Cartesian pose goal) 트랙의 마지막 정리 조각을 닫았다.** `MP-0011`(PR #14,
`planning/goals.py` 다중 재시도 IK)이 병합된 뒤, 같은 DLS/backtracking 로직이
`tests/offline_pose_ik.py`에 그대로 복붙돼 남아있던 중복을 발견 — solver의
관절 범위로 임시 `RightArmSpace`를 만들어 `planning.goals`에 위임하는 얇은
어댑터로 교체했다(108→28줄, `MP-0012`, PR #19). 호출부(`test_phase_3.py`/
`test_phase_4.py`/`tests/record_demo.py`)의 4-튜플 반환 시그니처는 그대로
유지해 회귀 위험 없이 끝냈다 — `tests/test_planning_*.py` 93개 + 실제
`test_phase_3.py` 물리 게이트(IK 100/100, pick 10/10) 재검증 통과.

이 전 cycle(오늘 오전, 사람 주도)에서는 PR #13~#16이 실제로 병합됐음을
TODO.md/PRD에 동기화하고(`docs/prd-roadmap-sync`, PR #17), `todo_tool.py`의
파이프 이스케이프 버그를 수정(`MP-0034`, PR #18)했다. 그 전에는 `todo_tool.py`
단위 테스트(PR #16, MP-0019)가 들어왔다.

## Current bottleneck

**PR 리뷰 큐가 3건(#17/#18/#19) — 다음 cycle이 이 셋을 다 만나면 cadence
gate(≥4)에 근접한다.** #17(PRD/TODO 실제 병합 상태 동기화)·#18(MP-0034
todo_tool.py 파이프 버그 수정)·#19(MP-0012 offline_pose_ik 위임) 전부 사람
리뷰 대기.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p4-offline-pose-ik-delegate | 2026-09-12 | PR #19, MP-0012 offline_pose_ik.py를 planning.goals로 위임 — 사람 리뷰 대기 | 0 |
| planning/p0-todo-tool-pipe-escape-fix | 2026-09-12 | PR #18, MP-0034 todo_tool.py 파이프 이스케이프 파싱 버그 수정 — 사람 리뷰 대기 | 0 |
| docs/prd-roadmap-sync | 2026-09-12 | PR #17, PRD 로드맵/Success Metrics를 실제 병합 상태로 동기화 — 사람 리뷰 대기 | 0 |

## Recent learnings (last 3 cycles)

- **"solver만 있고 space가 없는" 호출부를 `planning.goals`에 위임하려면
  solver가 이미 들고 있는 `joint_ranges`/`joint_limited`/`n`으로 임시
  `RightArmSpace`를 즉석 구성하면 충분하다** — `RightArmSpace.from_limits()`
  는 단위 테스트용으로 `limited=all-True`를 강제하므로 원시 생성자를 직접
  써야 한다. 위임 전엔 실제 호출부의 관절이 전부 limited인지 코드로
  확인해야 조용한 샘플링 분포 회귀를 피한다.
- **"PR 큐 포화"로 멈춰 있던 자율 루프가 사람이 한꺼번에 리뷰·병합하면
  즉시 재개 가능해야 한다** — 로컬 main이 원격보다 뒤처진 상태로 push를
  시도해 거부당하면 `git fetch` → `git rebase origin/main`으로 해결한다.
  PR 큐가 오래 쌓였다가 한꺼번에 풀리는 패턴에서는 항상 먼저 fetch하는
  습관이 중요하다.
- **PRD 같은 "가설 임계" 문서는 실측이 나온 시점에 반드시 대조해야 한다**
  — 임계를 못 넘겼다는 사실(P2 exit criterion 30%↓ vs 실측 6.82%)이
  숨겨지면 다음 사람이 잘못 넘겨짚을 수 있다. 실측이 임계에 못 미쳐도
  그대로 적고 판단은 사람에게 넘긴다.

## Next claude-actionable

1. `MP-0022`(P4) — `aggregate_results.py`에 Wilson CI 계산 추가. 의존성 없음,
   main에서 바로 착수 가능.
2. `MP-0037`(P1) — `_solve_valid_ik` 무작위 재시도를 `q_init` 반경 확장
   샘플링으로 개선(R-F-009 1차 후보). main에서 바로 착수 가능.
3. `MP-0033`(P2) — `time_parameterize`에 LSPB via-point 블렌딩 opt-in 추가
   검토(R-F-010 1차 후보).

## Next user-blocked

1. **PR #17/#18/#19** 사람 리뷰/병합 대기.
2. **`MP-0030`** P7 Tier 2(결합형 whole-body 플래너) 착수 여부 결정 — 타당성
   평가 완료, 검증 시나리오(합성 MJCF)가 먼저 필요하다는 점까지 문서화됨.
3. **`MP-0020`** Telegram 봇 생성 및 `telegram_setup.sh` 실행(사람만 가능).
4. **`MP-0021`**(hydrax)/`MP-0025`(VAMP-MR)/`MP-0036`(BIT*) 우선순위 결정 —
   셋 다 조사 끝, 구현 여부만 사용자 판단 대기.
5. **`MP-0014`**(pose goal 20 seed 성공률 측정) — `UserTest=☑`, 사람 확인 필요.
6. **`MP-0038`** ReachabilityMap에 격자별 `representative_q` 저장 확장 검토 —
   스키마 변경 승인 필요.

## Cycles to date

21 (2026-08-30~09-06 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
반복/트리 시각화, 장애물 재배치, Q-space 시각화+CVD 팔레트, 인터랙티브 마우스
목표+버그 수정 3건, nullspace 정칙화+hydrax 조사, 데모 실행 경로에 shortcut+
시간 파라미터화 연결, RRT* 대안 플래너, CHOMP류 궤적 최적화 후처리, 벤치마크
하네스+P1 성공률 첫 측정, PR #1/#2 병합+P3 실행 모듈, PRD를 P7(모바일
매니퓰레이터)까지 확장 + P7.0 reachability map, P7.1 베이스 자세 선택,
P7 Tier 2 타당성 평가 + PR #3/#4/#5/#7/#8/#12 전체 병합(문서 중복 수동
정리 포함)로 PR 큐 완전 소진; 자율 루프: shortcut 평활화, 시간
파라미터화, MP-0007/MP-0017 벤치마크 비교, P4 Cartesian pose goal IK
다중 재시도, safety-certificate 캐싱 순이득 프로파일링(도입 보류),
todo_tool.py 단위 테스트; 2026-09-12 사람 주도: TODO/PRD를 실제 병합
상태로 동기화 + todo_tool.py 파이프-이스케이프 버그 수정; 자율 루프:
tests/offline_pose_ik.py를 planning.goals로 위임)
