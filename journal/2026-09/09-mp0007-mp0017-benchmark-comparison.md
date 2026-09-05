# MP-0007/MP-0017: shortcut·RRT* 실측 비교 (PR #13)

- **Cycle**: 2026-09-06
- **Branch**: `planning/p5-planner-comparison` (PR #13)
- **TODO**: MP-0007, MP-0017, MP-0031(신규)
- **Phase**: P2/P5
- **Status**: keep

## What I tried

사용자가 "MP-0007/MP-0017 벤치마크 비교 측정 진행해주세요"라고 요청 —
직전 cycle에서 내가 STATE.md에 스스로 남긴 다음 우선순위였다. Plan mode로
`scripts/benchmark_planning.py`(PR #7)/`tests/test_planning_benchmark.py`/
`scripts/aggregate_results.py`/`results/README.md`를 다시 읽고 설계한 뒤:

1. `benchmark_planning.py`에 `--planner {rrt_connect,rrt_star}`와
   `--postprocess {none,shortcut}`를 추가. `format_metric()`에 선택적
   `planner=`/`path_len_after=` 필드를 넣되, 기존 두 exact-match 테스트가
   새 인자를 안 넘기므로 문자열이 한 글자도 안 바뀌게 설계(하위호환).
2. `tests/test_planning_benchmark.py`에 새 필드 직렬화 테스트 4개 추가.
3. 실제 can-sort 장면(장애물 없음, `p4-benchmark-harness.tsv`의 기존
   baseline과 완전히 같은 seed 0-49)로 두 번 실측:
   - RRT-Connect + shortcut 후처리 (2.2초, MP-0007용)
   - RRT* (seed당 최대 15초 예산, 총 750.1초 ≈ 12.5분, 백그라운드 실행,
     MP-0017용)

작업 시작 직전 워킹 디렉토리에 자율 researcher 루프가 동시에 남긴
`research/2026-09/006.md`(미커밋)를 발견 — 정확히 이 작업(MP-0017 통계
검정 방법)을 미리 조사해 뒀었다. 브랜치를 만들기 전에 `git stash push --
<그 파일들>`로 분리해 main에 먼저 push하고, 내 코드 브랜치는 그 위에서
새로 시작했다(이 세션에 확립된 "동시 작업 분리" 패턴 그대로).

## What worked / what failed

**MP-0007은 깔끔하게 확인됐다** — 실제 장면 50 seed에서 shortcut이 경로를
늘린 사례 0건, 중앙값 6.82%·평균 7.51%·최대 27.27% 감소. 기존 합성-공간
property test(`test_planning_shortcut.py`)의 "길이 비증가" 결론을 실제
장면에서 재확인하면서 처음으로 실제 감소폭 숫자를 얻었다.

**MP-0017은 예상과 다른, 하지만 정직하게 보고해야 할 결과가 나왔다.**
`research/2026-09/006.md`가 권고한 대응 표본(paired) Wilcoxon signed-rank
검정(scipy 미설치라 정규근사로 직접 구현 — rank 계산에 tie-aware 평균
순위 적용)을 그대로 적용한 결과: **z=-0.174, p≈0.86으로 통계적으로
유의한 차이가 없다.** 22/50 seed에서 RRT*가 더 짧고 21/50에서 더 길어
사실상 동전 던지기 수준이었다 — RRT*는 그러면서도 중앙값 기준
~1200배(15005ms vs 12.5ms) 더 느렸다. 이 장애물 없는 시나리오·15초 예산
조합에서는 RRT-Connect가 명백히 더 나은 선택이라는 뜻이다.

이 결과를 숨기거나 다른 파라미터로 재시도해 "원하는 결론"을 찾으려
하지 않고 그대로 PR 설명·TODO·journal에 적었다 — 왜 이런 결과가 나왔는지
그럴듯한 설명(RRT-Connect의 bidirectional greedy CONNECT가 장애물 없는
공간에서는 이미 거의 직선에 가까운 경로를 한두 번 반복 만에 찾는 반면,
RRT*는 균일 무작위 샘플링 기반이라 같은 시간 안에 그 정도로 직접적인
경로를 못 찾을 수 있다)이 있지만, 검증하지 않은 추측이라 결론에 넣지
않고 "장애물 있는 시나리오나 더 긴 예산에서 재검증이 필요하다"는 후속
과제(MP-0031)로만 남겼다.

## North-star delta

두 TODO 항목이 실측 데이터로 뒷받침됐다 — MP-0007은 확인(shortcut은
안전하고 효과 있음), MP-0017은 "생각보다 애매하다"는 정직한 결론으로
마무리됐다. `docs/guide/motion-planning.md`의 RRT* 섹션이 아직 이 실측
결과를 반영하지 않고 있다는 걸 명시적으로 TODO(MP-0031)에 남겨 다음
cycle이 놓치지 않게 했다.

## Key learnings

- **자율 researcher 루프가 미리 해 둔 조사(대응 표본 Wilcoxon 검정
  선택)를 그대로 적용하니 통계 설계를 처음부터 고민할 필요가 없었다** —
  cron 루프와 대화형 세션이 같은 저장소에서 협업하는 게 실제로 값을
  냈다.
- **"놀라운 결과"가 나왔을 때 파라미터를 바꿔가며 원하는 결론이 나올
  때까지 재실행하고 싶은 유혹이 있지만, 처음 정직하게 설계한 측정
  하나를 있는 그대로 보고하는 게 맞다** — 그럴듯한 사후 설명은 결론이
  아니라 "왜 그런지 모르겠다"는 사실과 함께 후속 조사 항목으로 분리해야
  한다.
- **`format_metric`처럼 이미 exact-match 테스트가 있는 순수 함수를 확장할
  땐, "새 키워드 인자 추가 + 기본값일 때 완전히 동일한 출력" 패턴이
  회귀 위험을 코드 리뷰 없이도 테스트만으로 증명 가능하게 만든다.**

## Recommended next 1–3 priorities

1. PR #13 사람 리뷰/병합.
2. 사용자가 `MP-0031`(RRT* 재검증 여부/문서 반영), `MP-0030`(P7 Tier 2),
   `MP-0021`(hydrax), `MP-0025`(VAMP-MR) 중 우선순위를 정하면 그에 따라
   진행.
3. `MP-0011`/`MP-0012`(IK 시드 다중 재시도) — 다음 자연스러운
   claude-owner 백로그.

## Artifacts

- PR: https://github.com/Geonhee-LEE/aiworker-mj-lab/pull/13
- 실측 데이터: `results/p5-planner-comparison.tsv`(100행)
- 통계 방법론 근거: `research/2026-09/006.md`(researcher cron 작성)
