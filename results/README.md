# results/

브랜치당 TSV 하나, append-only 실험 결과.

## 파일 이름

`results/<phase>-<slug>.tsv` — 브랜치 `planning/<phase>-<slug>`와 1:1 대응한다.

## 스키마

탭 구분, 첫 append 시 헤더 필수:

```
timestamp	commit	metric	status	description
```

- `status` ∈ `keep | discard | crash | in_progress`
- `metric`: 초기에는 `qual:<short>`, 벤치마크가 생긴 뒤에는
  `bench:success=<r>,plan_ms_p50=<t>,path_len=<l>,min_clearance=<c>`

## 규율

**append-only.** 과거 행은 절대 수정하지 않는다. `keep` 행이 나중에 틀린 것으로
밝혀지면 그 행을 참조하는 `discard` 행을 새로 추가한다.

집계: `python3 scripts/aggregate_results.py` → `RESULTS.md` 재생성.
