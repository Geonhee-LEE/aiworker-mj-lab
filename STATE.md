# Research State — auto-generated each cycle

_Last updated: 2026-09-12 · cycle p4-wilson-ci-aggregate_

## North star distance

**P4(Cartesian pose goal) 트랙 정리는 지난 cycle에 닫혔고, 이번 cycle은 계측
인프라를 한 조각 보강했다.** `scripts/aggregate_results.py`에 성공률 Wilson
95% CI 계산(`MP-0022`, PR #20)을 추가 — `results/*.tsv`의 `bench:` 메트릭을
`planner`별로 그룹핑해 `RESULTS.md`에 신뢰구간을 자동 표시한다. 실제
재생성 결과 `p5-planner-comparison`의 rrt_star narrow_passage/cluttered 혼합
성공률이 `0.830 [0.745, 0.891] (n=100)`으로 기존 MP-0031 실측(78-88%)과
일치함을 확인해 검증 완료. `RESULTS.md`/`TODO.md`는 별도 state-push
파이프라인이 갱신하는 산출물이라 PR 커밋에서 제외했다(과거 커밋 히스토리로
관례 확인).

이 전 cycle(오늘 오전)에서는 `tests/offline_pose_ik.py`를 `planning.goals`로
위임하는 어댑터로 교체(`MP-0012`, PR #19, 108→28줄)해 P4 IK 로직 중복을
닫았다. 그 전에는 PR #13~#16 병합 상태를 TODO.md/PRD에 동기화(PR #17)하고
`todo_tool.py`의 파이프 이스케이프 버그를 수정(`MP-0034`, PR #18)했다.

## Current bottleneck

**PR 리뷰 큐가 4건(#17/#18/#19/#20)으로 cadence gate(≥4) 임계값에 도달했다.**
다음 executor cycle은 `gh pr list --head "planning/" --state open`(exact-match
함정 주의, `--json headRefName`으로 재확인 필요)이 4건을 보면 조용히
`EXECUTOR_SKIP reason=pr-queue-full`로 종료해야 한다. #17(PRD/TODO 동기화)·
#18(MP-0034 파이프 버그)·#19(MP-0012 offline_pose_ik 위임)·#20(MP-0022 Wilson
CI) 전부 사람 리뷰 대기.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p4-wilson-ci-aggregate | 2026-09-12 | PR #20, MP-0022 aggregate_results.py에 Wilson 95% CI 추가 — 사람 리뷰 대기 | 0 |
| planning/p4-offline-pose-ik-delegate | 2026-09-12 | PR #19, MP-0012 offline_pose_ik.py를 planning.goals로 위임 — 사람 리뷰 대기 | 0 |
| planning/p0-todo-tool-pipe-escape-fix | 2026-09-12 | PR #18, MP-0034 todo_tool.py 파이프 이스케이프 파싱 버그 수정 — 사람 리뷰 대기 | 0 |
| docs/prd-roadmap-sync | 2026-09-12 | PR #17, PRD 로드맵/Success Metrics를 실제 병합 상태로 동기화 — 사람 리뷰 대기 | 0 |

## Recent learnings (last 3 cycles)

- **`results/*.tsv`의 `bench:` 메트릭은 이미 `planner=`/`success=` 필드를
  일관되게 써 왔으므로, 성공률 신뢰구간 계산은 새 계측 코드 없이 순수
  집계 레이어(문자열 파싱+Wilson score interval)만으로 끝났다** — 향후
  P4(MP-0014, 20-seed pose goal 성공률)·P5 비교연구 모두 이 CI 표시를
  자동으로 물려받는다.
- **자동생성 산출물(`RESULTS.md`)과 메타 상태 파일(`TODO.md`)은 PR
  브랜치가 아니라 별도 state-push 파이프라인이 main에 직접 반영한다** —
  과거 커밋의 파일 stat(`bfe5af8` 등)을 먼저 확인해 이 관례를 재확인하지
  않았다면 불필요한 파일을 PR에 끼워넣을 뻔했다. 다음 cycle도 같은 방식
  유지할 것.
- **"solver만 있고 space가 없는" 호출부를 `planning.goals`에 위임하려면
  solver가 이미 들고 있는 `joint_ranges`/`joint_limited`/`n`으로 임시
  `RightArmSpace`를 즉석 구성하면 충분하다** — `RightArmSpace.from_limits()`
  는 단위 테스트용으로 `limited=all-True`를 강제하므로 원시 생성자를 직접
  써야 한다.

## Next claude-actionable

1. `MP-0037`(P1) — `_solve_valid_ik` 무작위 재시도를 `q_init` 반경 확장
   샘플링으로 개선(R-F-009 1차 후보). main에서 바로 착수 가능. **단, PR 큐가
   4건이라 다음 cycle은 gate에 걸려 착수하지 못할 가능성이 높음.**
2. `MP-0033`(P2) — `time_parameterize`에 LSPB via-point 블렌딩 opt-in 추가
   검토(R-F-010 1차 후보).
3. `MP-0035`(P1) — `collision_distance_gradient`에 `need_gradient=False`
   조기 반환 분기 추가(호출부가 안 쓰는 Jacobian 계산 낭비 제거).
4. `MP-0032`(P5) — narrow_passage/cluttered RRT* bridge-test 편향 샘플링
   검토(MP-0031 실측 78-88% 성공률 개선 여지).

## Next user-blocked

1. **PR #17/#18/#19/#20** 사람 리뷰/병합 대기(4건, gate 임계값 도달).
2. **`MP-0030`** P7 Tier 2(결합형 whole-body 플래너) 착수 여부 결정 — 타당성
   평가 완료, 검증 시나리오(합성 MJCF)가 먼저 필요하다는 점까지 문서화됨.
3. **`MP-0020`** Telegram 봇 생성 및 `telegram_setup.sh` 실행(사람만 가능).
4. **`MP-0021`**(hydrax)/`MP-0025`(VAMP-MR)/`MP-0036`(BIT*) 우선순위 결정 —
   셋 다 조사 끝, 구현 여부만 사용자 판단 대기.
5. **`MP-0014`**(pose goal 20 seed 성공률 측정) — `UserTest=☑`, 사람 확인 필요.
6. **`MP-0038`** ReachabilityMap에 격자별 `representative_q` 저장 확장 검토 —
   스키마 변경 승인 필요.

## Cycles to date

22 (2026-08-30~09-06 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
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
tests/offline_pose_ik.py를 planning.goals로 위임, aggregate_results.py에
Wilson 95% CI 추가)
