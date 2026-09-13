# Research State — auto-generated each cycle

_Last updated: 2026-09-13 · cycle p1-ik-retry-radius-seeding_

## North star distance

**이번 cycle은 P1 트랙의 부수적 개선(인터랙티브 데모 IK 시딩 효율)이고,
North star(충돌 없는 관절 경로 계획·실행) 자체는 지난 cycle들에서 이미
P4(Cartesian pose goal)까지 정리·계측 인프라 보강이 끝난 상태다.**
`_solve_valid_ik`(`scripts/demo_plan_right_arm.py`)의 재시도 시드를 관절범위
전체 균등 무작위 대신 `q_init` 주변 반경 10%→30%→100% 단계적 확장
(`_staged_retry_seeds`)으로 바꿨다(`MP-0037`, R-F-009 1차 후보). 임시 A/B
벤치마크로 "q_init 첫 시도가 실패하는" 사례(3000개 스캔 중 50개, 1.7%)만
따로 골라 비교한 결과 성공률은 이전/이후 모두 100%로 동일, 평균 재시도
횟수가 5.84→5.46(~6.5% 감소)으로 방향은 맞았지만 개선폭이 크지 않다는 점을
`results/p1-ik-retry-radius-seeding.tsv`에 투명하게 기록했다. 완전 무작위
원거리 타겟으로 처음 측정했을 때는 오히려 역전되는 걸 확인해(가정한
"인접 타겟" 분포에서만 이득) 잘못된 벤치마크 설계였음도 journal에 남겼다.

이 전 cycle(오늘 오전, PR #20)에서는 `scripts/aggregate_results.py`에
성공률 Wilson 95% CI 계산(`MP-0022`)을 추가했고, 그 전(PR #19)에는
`tests/offline_pose_ik.py`를 `planning.goals`로 위임하는 어댑터로 교체해
P4 IK 로직 중복을 닫았다(108→28줄).

## Current bottleneck

**PR 리뷰 큐가 이번 cycle(PR #21 생성)로 4건(#18/#19/#20/#21)이 되어
cadence gate(≥4) 임계값에 도달했다.** 다음 executor cycle은
`gh pr list --state open --json headRefName`(`--head "planning/"`는
exact-match라 항상 빈 결과를 내는 함정 — prefix 필터링은 반드시
`--json`으로 받은 뒤 직접 걸러야 한다)으로 4건을 확인하면 조용히
`EXECUTOR_SKIP reason=pr-queue-full`로 종료해야 한다. #18(MP-0034 파이프
버그)·#19(MP-0012 offline_pose_ik 위임)·#20(MP-0022 Wilson CI)·#21(MP-0037
IK 재시도 반경 확장) 전부 사람 리뷰 대기.

## Open experiments

| Branch | Last update | Last description | Days open |
|---|---|---|---|
| planning/p1-ik-retry-radius-seeding | 2026-09-13 | PR #21, MP-0037 `_solve_valid_ik` 재시도 시드를 q_init 반경 확장 샘플링으로 개선 — 사람 리뷰 대기 | 0 |
| planning/p4-wilson-ci-aggregate | 2026-09-12 | PR #20, MP-0022 aggregate_results.py에 Wilson 95% CI 추가 — 사람 리뷰 대기 | 1 |
| planning/p4-offline-pose-ik-delegate | 2026-09-12 | PR #19, MP-0012 offline_pose_ik.py를 planning.goals로 위임 — 사람 리뷰 대기 | 1 |
| planning/p0-todo-tool-pipe-escape-fix | 2026-09-12 | PR #18, MP-0034 todo_tool.py 파이프 이스케이프 파싱 버그 수정 — 사람 리뷰 대기 | 1 |

## Recent learnings (last 3 cycles)

- **반경 확장 재시도 샘플링의 이득은 "q_init이 이미 거의 통하는" 인접
  타겟 분포에서만 나타난다** — 완전 무작위 원거리 타겟으로 벤치마크하면
  좁은 반경 단계가 낭비되어 오히려 역효과가 보일 수 있다(실측: 새 방식
  평균 재시도 2.62 vs 구 방식 1.02, 둘 다 대부분 idx=0에서 즉시 성공).
  이런 함수를 벤치마크할 때는 반드시 "q_init 첫 시도 실패" 사례로
  필터링한 타겟 집합을 써야 의도한 개선 방향과 일치하는 신호를 얻는다
  ([[journal/2026-09/13-11-p1-ik-retry-radius-seeding]] 참고).
- **`gh pr list --head "planning/"`는 exact-match라 prefix 필터로는 항상
  빈 결과를 낸다** — cadence gate 판정은 반드시 `--json headRefName`으로
  전체를 받은 뒤 문자열 prefix로 직접 걸러야 한다. 최근 3개 cycle 연속
  이 함정이 재확인됐다.
- **`results/*.tsv`의 `bench:` 메트릭은 이미 `planner=`/`success=` 필드를
  일관되게 써 왔으므로, 성공률 신뢰구간 계산은 새 계측 코드 없이 순수
  집계 레이어(문자열 파싱+Wilson score interval)만으로 끝났다** — 향후
  P4(MP-0014, 20-seed pose goal 성공률)·P5 비교연구 모두 이 CI 표시를
  자동으로 물려받는다.

## Next claude-actionable

1. `MP-0033`(P2) — `time_parameterize`에 LSPB via-point 블렌딩 opt-in 추가
   검토(R-F-010 1차 후보). **단, PR 큐가 4건이라 다음 cycle은 gate에 걸려
   착수하지 못할 가능성이 높음.**
2. `MP-0035`(P1) — `collision_distance_gradient`에 `need_gradient=False`
   조기 반환 분기 추가(호출부가 안 쓰는 Jacobian 계산 낭비 제거).
3. `MP-0032`(P5) — narrow_passage/cluttered RRT* bridge-test 편향 샘플링
   검토(MP-0031 실측 78-88% 성공률 개선 여지). `research/2026-09/013.md`가
   고전 Hsu et al. 알고리즘(무효점 2개 중점 채택)까지 이미 구체화해 둠.

## Next user-blocked

1. **PR #18/#19/#20/#21** 사람 리뷰/병합 대기(4건, gate 임계값 도달).
2. **`MP-0030`** P7 Tier 2(결합형 whole-body 플래너) 착수 여부 결정 — 타당성
   평가 완료, 검증 시나리오(합성 MJCF)가 먼저 필요하다는 점까지 문서화됨.
3. **`MP-0020`** Telegram 봇 생성 및 `telegram_setup.sh` 실행(사람만 가능).
4. **`MP-0021`**(hydrax)/`MP-0025`(VAMP-MR)/`MP-0036`(BIT*) 우선순위 결정 —
   셋 다 조사 끝, 구현 여부만 사용자 판단 대기.
5. **`MP-0014`**(pose goal 20 seed 성공률 측정) — `UserTest=☑`, 사람 확인 필요.
6. **`MP-0038`** ReachabilityMap에 격자별 `representative_q` 저장 확장 검토 —
   스키마 변경 승인 필요.

## Cycles to date

23 (2026-08-30~09-06 사람 주도: P0 부트스트랩, P1 RRT-Connect 구현, 데모
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
Wilson 95% CI 추가, `_solve_valid_ik` IK 재시도 시드를 q_init 반경 확장
샘플링으로 개선)
