# aggregate_results.py에 성공률 Wilson 95% CI 계산 추가

- **Cycle**: 2026-09-12 21:00 KST
- **Branch**: `planning/p4-wilson-ci-aggregate`
- **TODO**: `MP-0022` [research] aggregate_results.py에 성공률 Wilson 신뢰구간 계산 추가
- **Phase**: P4
- **Status**: keep

## What I tried

Cadence gate 통과(`gh pr list --head "planning/"` 0건, `Doing` owner=claude
비어있음, 최근 24시간 planning 브랜치 3개 < 6, Backlog 8건 비어있지 않음)
후 STATE.md/JOURNAL.md 상위 5개·`gh pr list --state open`(#17/#18/#19, 3건)을
리뷰. `Doing`/`Today` 모두 비어 결정 트리 3번(백로그 승격)으로 진입 —
STATE.md의 "North star distance"가 P4(offline_pose_ik 위임) 트랙 정리를 막
닫았다고 명시하고 "Next claude-actionable" 1순위로 이미 `MP-0022`(P4, 의존성
없음)를 지목하고 있어 이를 Today로 승격 후 선택. 의존성으로 명시된
`MP-0013`/`MP-0018`이 `TODO.md` `## Done`에 실제로 존재함을 먼저 확인.

`scripts/aggregate_results.py`에 `_wilson_ci(successes, n, z=1.96)`(Wilson
score interval), `_parse_bench_metric(metric)`(`bench:k=v,...` 문자열을 dict로
파싱), `_success_rate_lines(rows)`(bench 행을 `planner` 필드 기준 그룹핑해
성공률+CI 문자열 생성)를 추가하고 `main()`의 각 tsv 섹션 헤더에 삽입.
`tests/test_aggregate_results.py` 신규 8개(경계값·그룹핑·planner 없는 경우
"all" 그룹·bench 아닌 행 무시)로 검증.

## What worked / what failed

전체가 첫 시도에 통과. `results/p5-planner-comparison.tsv`(rrt_star,
narrow_passage/cluttered 혼합)로 실제 재생성해보니 `0.830 [0.745, 0.891]
(n=100)`이 나와, 기존 MP-0031 실측(78-88%)과 정확히 일치함을 확인 —
Wilson CI가 실측 대역을 올바르게 감싸는 걸 실측 데이터로 재검증했다.

`RESULTS.md`가 "자동생성, 손대지 말 것" 산출물이고 과거 PR(`bfe5af8` 등)
어디에도 `RESULTS.md` 자체가 포함되지 않았음을 커밋 히스토리로 먼저 확인한
뒤, 로컬에서 검증용으로 재생성했다가 PR 커밋 전에 `git checkout -- RESULTS.md`
로 되돌려 PR에는 코드/테스트/결과 tsv만 포함시켰다 — 별도 state-push
파이프라인이 갱신하는 파일을 PR로 건드리지 않기 위함. 같은 이유로
`todo_tool.py set --status Doing`이 만든 `TODO.md` 작업 트리 변경도 이번
feature 커밋에서 제외(과거 커밋 stat에 `TODO.md`가 없었던 패턴과 일치) —
main 쪽 TODO/STATE 갱신은 이 REPORT 단계에서 별도로 처리한다.

## North-star delta

P4 트랙 자체를 진전시키진 않지만, P4/P5 벤치마크 결과(`results/*.tsv`)를 다음
사람/에이전트가 신뢰구간 없이 point estimate만 보고 성급하게 결론 내리는 걸
막는 계측 인프라 조각 — 향후 P4(20-seed pose goal 성공률, MP-0014)·P5
비교연구 모두 이 CI 표시를 그대로 얻는다.

## Key learnings

- **`results/*.tsv`의 `bench:` 메트릭 필드는 이미 `planner=` 키로 실험군을
  구분해 왔으므로, 그룹핑 로직은 새 스키마 없이 기존 문자열 파싱만으로
  충분했다** — `success=0/1` 필드도 이미 있어 신규 계측 코드가 전혀 필요
  없었고, 순수 집계 스크립트 레이어에서만 구현이 끝났다.
- **자동생성 산출물(`RESULTS.md`)과 메타 상태 파일(`TODO.md`)은 PR 브랜치가
  아니라 별도 state-push 파이프라인이 main에 직접 반영한다** — 과거 커밋의
  파일 stat을 먼저 확인해 이 관례를 재확인하지 않았다면 불필요한 파일을
  PR에 끼워넣을 뻔했다.

## Recommended next 1–3 priorities

1. PR #17/#18/#19/#20 사람 리뷰/병합(큐 4건 — 다음 cycle 진입 전 gate
   임계값(≥4)에 이미 도달, 다음 cron 실행은 `EXECUTOR_SKIP
   reason=pr-queue-full`로 조용히 종료될 가능성이 높음을 STATE.md에
   명시해 둔다).
2. `MP-0037`(P1) — `_solve_valid_ik` q_init 반경 확장 샘플링(R-F-009 1차
   후보), 의존성 없음.
3. `MP-0033`(P2) — `time_parameterize` LSPB via-point 블렌딩(R-F-010 1차
   후보).

## Artifacts
- PR: https://github.com/Geonhee-LEE/aiworker-mj-lab/pull/20
- Files touched: scripts/aggregate_results.py, tests/test_aggregate_results.py, results/p4-wilson-ci-aggregate.tsv
- TSV row appended: yes
