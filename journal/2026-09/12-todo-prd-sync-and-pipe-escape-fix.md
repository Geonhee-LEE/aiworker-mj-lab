# TODO/PRD 실제 병합 상태 동기화 + todo_tool.py 파이프-이스케이프 버그 수정(MP-0034)

- **Cycle**: 2026-09-12
- **Branch**: `docs/prd-roadmap-sync`(PR #17), `planning/p0-todo-tool-pipe-escape-fix`(PR #18)
- **TODO**: MP-0007/MP-0011/MP-0017/MP-0031(Blocked→Done 동기화), MP-0034
- **Phase**: P0/P2-P5/P7 문서, P0 인프라
- **Status**: keep

## What I tried

사용자가 "TODO, PRD md를 보고 다음 작업을 진행해주세요"라고 요청. 직전에
PR #13/#14/#15/#16을 직접 리뷰(diff 전체 읽기 + merge-tree 충돌 검사 +
테스트/ruff 재실행)해 4건 모두 병합 권장 의견을 드렸는데, 사용자가 GitHub
에서 직접 네 건을 병합한 뒤 이 요청을 보낸 것으로 확인됨 — `git fetch`로
`origin/main`을 보니 PR #13~#16이 전부 `MERGED` 상태였다.

1. 로컬 main을 새 병합 커밋들로 rebase하고, 대기 중이던 자율 루프의
   blackboard 변경(MP-0037/0038 등)을 `state_push.sh`로 먼저 정리.
2. `TODO.md`의 MP-0007/0011/0017/0031을 Blocked→Done으로 이동(지금까지
   "PR 리뷰 대기"라고 적혀 있던 제목 문구를 "PR #13/#14 병합 완료(main에
   실제로 있음)"로 갱신) — R-F-008 "양방향 동기화" 원칙 그대로.
3. `docs/prd.md`를 실제 상태와 대조: Phased Roadmap 표가 P2/P3/P4/P5/P7을
   전부 "미착수"/"리뷰 대기"로 몇 주째 방치하고 있었다(실제로는 PR
   #1~#5,#7,#8,#10~#14 전부 병합됨). 로드맵 표, P4 "완벽"의 운영적 정의
   표(가설 임계 vs 실측값), R-F-009, Success Metrics 체크박스 7개를
   실측 근거와 대조해 갱신(PR #17).
4. 갱신 과정에서 발견한 중요한 간극 하나를 숨기지 않고 그대로 노출:
   P2 exit criterion "shortcut 후 경로 길이 30%↓"의 실측값이 6.82%로
   가설 임계에 크게 못 미친다 — 이유(실제 can-sort 장면은 장애물이
   적어 처음부터 덜 꼬인 경로가 나옴)를 적고, 임계 재설정 여부는 사람
   판단으로 남김.
5. `next claude-actionable`으로 이전 STATE.md가 제안했던 MP-0034(todo_tool.py
   parse()가 `_escape()`의 `\|`를 이해 못해 제목에 리터럴 `|`가 있으면
   행이 조용히 버려지는 버그)를 착수. `_CELL_SPLIT_RE = re.compile(r"(?<!\\)\|")`
   로 이스케이프 안 된 파이프만 셀 구분자로 보는 `_split_row()`를 추가해
   `parse()`의 naive `str.split("|")`를 교체. 기존 PR #16의
   characterization test를 회귀 검증으로 전환 + round-trip/`_next_id`
   커버리지 신규 추가(18개 통과), 실제 `TODO.md`로 `check` 재확인.

## What worked / what failed

PR 리뷰→병합 루프가 실제로 한 바퀴 돌았다 — 내가 리뷰 의견을 드리고
사용자가 직접 병합한 흐름이 깨끗하게 맞물렸다(다만 로컬 main이 먼저
push를 시도하다 `fetch first` 거부를 받아 rebase가 필요했다 — 병렬로
일어나는 git 작업에서 흔한 일이지만 매번 `git fetch`로 먼저 원격 상태를
확인하는 습관이 맞다는 걸 재확인).

PRD 싱크 작업에서 "가설 임계 vs 실측"을 나란히 적은 게 가장 가치있는
부분이었다 — 그냥 "✅ 완료"로 덮었으면 30% vs 6.82%라는 실제로 중요한
간극이 묻힐 뻔했다.

todo_tool.py 수정은 깔끔했다: 버그가 정확히 한 곳(`parse()`의 split
로직)에 있었고, 수정·회귀 테스트·실측(check 재실행) 모두 빠르게
끝났다. 이 버그가 이미 한 번 실제 사고(MP-0033 ID 충돌)를 냈다는 걸
커밋 메시지/PR 설명에 명시해 "왜 지금 고치는가"를 분명히 했다.

## North-star delta

문서(PRD)가 실제 저장소 상태를 다시 정확히 반영하게 됐고, TODO.md의
Blocked 큐도 실제로 비었다(4건 Done 이동). `todo_tool.py`의 파싱
버그(이미 한 번 데이터 유실을 일으킨 근본 원인)가 닫혀 앞으로 제목에
`|`가 들어가도 TODO 행이 안전하다.

## Key learnings

- **"PR 큐 포화"로 멈춰 있던 자율 루프가 사람이 한꺼번에 리뷰·병합하면
  즉시 재개 가능해야 한다** — 이번에 로컬 main이 원격보다 뒤처진 상태로
  push를 시도해 거부당했는데, `git fetch` → `git rebase origin/main`
  으로 바로 해결됐다. PR 큐가 오래 쌓였다가 한꺼번에 풀리는 패턴에서는
  항상 먼저 fetch하는 습관이 중요하다.
- **PRD 같은 "가설 임계" 문서는 실측이 나온 시점에 반드시 대조해야
  한다** — 임계를 못 넘겼다는 사실 자체가 숨겨지면 다음 사람이 "P4가
  끝났으니 기준도 충족됐겠지"라고 잘못 넘겨짚을 수 있다. 실측이 임계에
  못 미쳐도 그대로 적고 판단을 사람에게 넘기는 게 맞다.
- **자기참조적 버그(자신을 고치는 TODO의 ID를 잡아먹는 사고)는 고치고
  나면 회귀 테스트로 반드시 봉인해야 한다** — characterization test를
  그냥 삭제하지 않고 "이제는 안 깨진다"는 assertion으로 뒤집어 남겨야
  미래에 같은 회귀가 조용히 재발하지 않는다.

## Recommended next 1–3 priorities

1. PR #17(PRD 동기화)/#18(MP-0034 버그 수정) 사람 리뷰·병합.
2. `MP-0022`(aggregate_results.py Wilson CI) — 의존성 없음, 바로 착수 가능.
3. `MP-0012`(offline_pose_ik.py → planning.goals 위임) — PR #14가 병합돼
   `planning.goals`가 main에 있으니 이제 막힘 없이 착수 가능.

## Artifacts

- PR: https://github.com/Geonhee-LEE/aiworker-mj-lab/pull/17 (PRD 동기화)
- PR: https://github.com/Geonhee-LEE/aiworker-mj-lab/pull/18 (MP-0034)
- 실측: `results/p0-todo-tool-pipe-escape-fix.tsv`
