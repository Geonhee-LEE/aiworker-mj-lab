# TODO 진입점

이 문서는 사람용 안내다. 실제 작업 목록은 저장소 루트의 `TODO.md`에 있다
(문서 사이트가 아니라 저장소를 직접 열어야 보인다).

## 무엇이 진행 중인지 보기

```bash
python3 scripts/todo_tool.py list --status Doing
cat TODO.md   # 사람이 읽기 좋은 표 형태 전체
```

## 새 작업 지시하기

Telegram으로 메시지를 보내면 `telegram_poll.sh`가 `research/inbox.md`에 적재하고,
지시로 보이면 자동으로 TODO를 만든다. 직접 만들려면:

```bash
python3 scripts/todo_tool.py add "제목" --priority P1 --phase P1 --owner user
```

## 작업 완료 알림

executor가 PR을 올리면 Telegram으로 통지가 온다. 머지는 항상 사람이 한다.
`UserTest` 열이 `☑`인 항목은 사람이 직접 시뮬레이션을 돌려 결과를 답장해야 한다
(`ok` / `fail: <한 줄>` / `skip`).

## 어디서 어느 게 권위인가 (canonical authority)

| 정보 | 권위 |
|---|---|
| 작업 상태(Status/Priority/Owner) | `TODO.md` (파일 자체, Notion 없음) |
| 코드 결과 | `main` 브랜치 |
| 시스템 상태(지금 어디, 다음 무엇) | `STATE.md` |
| 북극성·요구사항 | `docs/prd.md` |

## TODO 의 생애주기

```
Backlog → Today → Doing → Done
              ↘ Blocked (사람 액션 대기) ↗
```

- `Backlog`: 아직 우선순위가 배정되지 않음
- `Today`: 이번 cycle에서 뽑힐 수 있는 후보
- `Doing`: 현재 진행 중 (동시에 1건만, `Owner=claude`)
- `Blocked`: 사람의 시뮬레이션 확인·결정이 필요
- `Done`: 완료, 최근 20개만 `TODO.md`에 남고 나머지는 `journal/`에서 추적

## Stuck TODO 처리

`Doing` 상태가 24시간을 넘기면 executor가 스스로 `[stuck]` 태그를 붙이고
다음 후보로 넘어간다. 사람이 원인을 확인하고 `Status`를 되돌리거나 `Blocked`로 옮긴다.
