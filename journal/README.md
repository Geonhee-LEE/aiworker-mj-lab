# journal/

cycle별 전체 보고서를 append-only로 남기는 곳이다. `JOURNAL.md`(저장소 루트)는
이 디렉터리에서 최근 20개를 뽑은 다이제스트이며, REVIEW 단계는 다이제스트의
상위 5개만 읽는다.

## 파일 이름 규칙

`journal/YYYY-MM/DD-HH-<slug>.md` — `YYYY-MM`은 KST 기준 월, `DD-HH`는 일+2자리 시,
`<slug>`는 그 cycle이 작업한 브랜치와 같은 slug다.

## 필수 섹션

```markdown
# <Cycle title — 짧고 구체적으로>

- **Cycle**: 2026-MM-DD HH:MM KST
- **Branch**: `planning/<phase>-<slug>`
- **TODO**: `MP-NNNN` <title>
- **Phase**: P<N>
- **Status**: keep | discard | crash | in_progress

## What I tried
## What worked / what failed
## North-star delta
## Key learnings
## Recommended next 1–3 priorities
## Artifacts
- PR: <url 또는 "pending merge (<branch>)">
- Files touched: <comma list>
- TSV row appended: yes | no
```

## append-only 규율

과거 항목을 절대 수정하지 않는다(오탈자 수정 제외). 다음 cycle이 이를 역사적
신호로 읽으므로, 역사를 고쳐 쓰면 루프가 오염된다. 결론이 틀렸다면 새 항목을
써서 그렇다고 밝힌다. 되돌려 쓰지 않는다.
