# MP-0019: todo_tool.py 단위 테스트(파싱·왕복)

- **Cycle**: 2026-09-07 11:00 KST
- **Branch**: `planning/p0-todo-tool-unit-tests`
- **TODO**: `MP-0019` [infra] `scripts/todo_tool.py` 단위 테스트(파싱·왕복)
- **Phase**: P0
- **Status**: keep

## What I tried

STATE.md "Next claude-actionable" 1순위였던 MP-0018/MP-0019 중 MP-0018
(aggregate_results.py)은 TODO.md `## Done` 섹션에 이미 반영돼 있어 확인만
하고 넘겼고, 아직 미착수였던 **MP-0019**를 선택했다. `scripts/todo_tool.py`
(TODO.md 파싱·렌더링·CLI)에 테스트가 전혀 없었다.

1. `tests/test_todo_tool.py` 신규 — 다른 `scripts/*.py` 관례를 따라 `sys.path`에
   `scripts/`를 넣고 직접 import.
2. 순수 함수 커버: `parse`(섹션별 행 추출, 헤더/구분선/잘못된 ID 무시),
   `render`↔`parse` 왕복(round-trip), `_next_id`, `_escape`/`_unescape`,
   긴 제목 100자 절단.
3. CLI 부작용 커버: `pytest` `monkeypatch`로 `todo_tool.TODO_PATH`를 `tmp_path`의
   격리된 파일로 바꿔치기해 `cmd_add`/`cmd_set`/`cmd_check`/`cmd_next`를 실제
   `TODO.md`를 전혀 건드리지 않고 검증(중복 ID, claude Doing 동시 1건 제한,
   섹션 이동, 존재하지 않는 ID 등).

## What worked / what failed

처음 샘플 데이터에 제목 안 리터럴 `|`를 넣고 "이스케이프된 pipe가 온전히
왕복된다"는 테스트를 썼더니 **8개 테스트가 실패**했다. 원인을 추적하니
`parse()`가 `line.split("|")`로 셀을 나누는데, 이건 `render()`가 붙이는
백슬래시 이스케이프(`\|`)를 전혀 이해하지 못한다 — 셀 개수가 7이 아니라
8이 되어 `len(cells) != len(COLUMNS)` 체크에 걸려 **그 행 전체가 조용히
버려진다**. 즉 제목에 `|`가 들어간 TODO 행은 지금 코드로는 저장 자체가
안 되는 실제 버그다.

이번 TODO는 "테스트 작성"이 스코프라 버그 수정은 하지 않고, 대신 이
동작을 있는 그대로 문서화하는 characterization test
(`test_parse_drops_row_when_title_contains_literal_pipe`)를 추가하고
나머지 테스트는 pipe 없는 제목으로 바꿨다 — 17개 전부 통과.

## North-star delta

직접적인 플래닝 진행은 없지만, TODO 상태 관리 도구(모든 cycle이 매번
의존하는 인프라)에 처음으로 회귀 방지망이 생겼다. 부수적으로 발견한
`|`-제목 버그는 지금까지 아무도 실제 제목에 `|`를 안 써서 드러나지
않았을 뿐 — 후속 TODO로 등록해 재발을 막는다.

## Key learnings

- **파싱 코드에 이스케이프 로직이 있으면 반드시 그 역방향(파서)도 이스케이프를
  실제로 이해하는지 왕복 테스트로 확인해야 한다** — `_escape`/`render`는
  올바르게 백슬래시를 붙이지만, `parse`의 naive `split("|")`는 그걸 몰라서
  깨진다. 이런 비대칭은 유닛 테스트 없이는 코드 리뷰만으로 못 잡기 쉽다.
- **"테스트만 쓰라"는 스코프에서 버그를 발견해도 바로 고치지 말고
  characterization test로 문서화 + 후속 TODO 등록이 맞는 절차** — 한
  cycle/PR에 방향을 하나로 유지하는 소프트 리밋과 일치.
- TODO.md 자체는 feature 브랜치 커밋에 포함하면 안 된다(과거 병합 PR
  전부 확인) — canonical한 상태 변경은 오직 main의 `state_push.sh`를
  거친다. Phase 3에서 `todo_tool.py set --status Doing`으로 생긴 로컬
  TODO.md diff는 커밋 전에 `git checkout -- TODO.md`로 되돌렸다.

## Recommended next 1–3 priorities

1. **신규 TODO**: `todo_tool.py`의 `parse()`가 `_escape`/`_unescape`를
   실제로 이해하도록 이스케이프-aware 셀 분리기로 교체(정규식 기반 split
   또는 `\|`를 임시 플레이스홀더로 치환 후 분리) — 제목에 `|`가 있는
   행이 조용히 사라지는 걸 막는다.
2. PR #13/#14/#15/#16 사람 리뷰/병합 대기(큐 4건, 게이트 임계값 임박).
3. PR #14 병합되면 `MP-0012`(`tests/offline_pose_ik.py` → `planning.goals`
   위임) 착수 가능.

## Artifacts

- PR: https://github.com/Geonhee-LEE/aiworker-mj-lab/pull/16
- Files touched: `tests/test_todo_tool.py`, `results/p0-todo-tool-unit-tests.tsv`
- TSV row appended: yes
