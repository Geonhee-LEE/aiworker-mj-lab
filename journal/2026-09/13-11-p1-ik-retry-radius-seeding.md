# IK 재시도 시드를 q_init 반경 확장 샘플링으로 개선

- **Cycle**: 2026-09-13 11:00 KST
- **Branch**: `planning/p1-ik-retry-radius-seeding`
- **TODO**: `MP-0037` [research] R-F-009: `_solve_valid_ik` 무작위 재시도를 `q_init` 반경 확장 샘플링으로 개선
- **Phase**: P1
- **Status**: keep

## What I tried

Cadence gate 통과 확인 — `gh pr list --head "planning/"`는 exact-match 함정
(STATE.md가 이미 지적)이라 빈 결과를 냈고, `--json headRefName`으로 재확인하니
실제 open PR은 3건(#18/#19/#20, docs/prd-roadmap-sync #17은 이미 병합됨)으로
gate 임계값(4) 미만이었다. `Doing`/`Today` owner=claude 모두 비어 있어 결정
트리 3번(백로그 승격)으로 진입, STATE.md "Next claude-actionable" 1순위인
`MP-0037`을 선택 — `scripts/demo_plan_right_arm.py`에 코드가 이미 main에
있어(unmerged 브랜치 의존 없음) 실행가능성 필터도 통과.

`_solve_valid_ik`의 재시도 시드를 `rng.uniform(joint_ranges)`(전 구간 균등)
대신 `_staged_retry_seeds(q_init, joint_ranges, rng, n_restarts)`로 교체 —
`q_init` 주변 반경을 10%→30%→100%(전체 범위) 3단계로 나눠 `n_restarts`를
균등 분배해 뽑는다(마지막 단계가 나머지를 흡수). 순수 함수로 분리해
`_solve_valid_ik` 본문은 후보 생성 한 줄만 바뀌었다.

## What worked / what failed

`tests/test_planning_ik_retry_seeding.py` 신규 3개(합성 4-DOF 관절범위로
단계별 반경 준수 확인, 관절한계 클리핑 확인, 실제 can-sort 장면에서
`_solve_valid_ik`가 충돌 없는 IK 해로 수렴하는지 통합 확인) 포함 96개
planning 테스트 전부 첫 시도에 통과.

측정 가능한 이득을 확인하려고 임시 벤치마크를 돌렸다 — 처음에는
`space.sample(rng)`로 뽑은 완전 무작위 원거리 타겟 60개로 비교했더니
오히려 새 방식이 평균 재시도 횟수가 더 많이 나왔다(새 2.62 vs 구 1.02,
둘 다 median=0). 원인은 명확했다: 대부분의 타겟이 `q_init` 자체에서 이미
풀려(median=0) 재시도 로직 자체가 거의 발동하지 않았고, 드물게
`q_init`이 아예 안 통하는 "멀리 있는" 타겟에서는 좁은 반경 단계가 그냥
낭비되고 100% 단계로 넘어가야 했기 때문 — 이건 R-F-009가 가정한 "연속
질의 사이 목표가 조금씩만 바뀌는" 인터랙티브 드래그 시나리오와 다른
분포였다. 이 첫 벤치마크는 신호가 뒤집혀 있어 결과에 기록하지 않고
버렸다. `CAN_SORT_START_Q` 기준 ±0.5rad 무작위 관절 오프셋으로 타겟을
만들어 "q_init 첫 시도가 실패(미수렴 또는 충돌)하는" 사례만 골라(3000개
스캔 중 50개, 1.7%) 재비교하니 성공률은 이전/이후 모두 100%로 동일, 평균
재시도 횟수 5.84 → 5.46(약 6.5% 감소)으로 방향은 맞았다.
`results/p1-ik-retry-radius-seeding.tsv`에 이 재비교 결과를 기록.

## North-star delta

North star(충돌 없는 관절 경로 계획·실행)에 직접 닿진 않지만, 인터랙티브
데모(`--interactive`)의 IK 시딩 효율을 소폭 개선 — 개선폭(~6.5%)이 크지
않다는 점을 투명하게 남긴다. "hard" 사례 자체가 드물어(1.7%) 대부분의
실사용 패턴에서는 체감 차이가 거의 없을 것으로 예상된다.

## Key learnings

- **반경 확장 샘플링의 이득은 "q_init이 이미 거의 통하는" 인접 타겟
  분포에서만 나타난다** — 완전 무작위 원거리 타겟으로 벤치마크하면 좁은
  반경 단계가 낭비되어 오히려 역효과가 보일 수 있다. 앞으로 이 함수를
  벤치마크할 때는 반드시 "첫 시도(`q_init`) 실패" 사례로 필터링한 타겟
  집합을 써야 실제 개선 방향과 일치하는 신호를 얻는다.
- `_ik_attempt`/`checker.is_valid`를 직접 호출해 "이전 방식"을 인라인으로
  재구현하는 A/B 벤치마크는 새 모듈 없이도 짧게(스크립트 40줄 이내) 짤 수
  있었다 — 이번엔 임시 스크립트로 실행만 하고 저장소에 남기지 않았다(결과는
  tsv에 기록).

## Recommended next 1–3 priorities

1. PR #18/#19/#20/#21 사람 리뷰/병합 — 이번 PR(#21)로 open planning PR이
   4건이 되어 다음 executor cycle은 gate 임계값(≥4)에 도달, `EXECUTOR_SKIP
   reason=pr-queue-full`로 조용히 종료될 가능성이 높다.
2. `MP-0033`(P2) — `time_parameterize`에 LSPB via-point 블렌딩 opt-in
   추가(R-F-010 1차 후보), 의존성 없음.
3. `MP-0035`(P1) — `collision_distance_gradient`에 `need_gradient=False`
   조기 반환 분기 추가(호출부가 안 쓰는 Jacobian 계산 낭비 제거).

## Artifacts
- PR: https://github.com/Geonhee-LEE/aiworker-mj-lab/pull/21
- Files touched: scripts/demo_plan_right_arm.py, tests/test_planning_ik_retry_seeding.py, results/p1-ik-retry-radius-seeding.tsv
- TSV row appended: yes
