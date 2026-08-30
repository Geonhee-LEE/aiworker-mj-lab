# 자동화 운영 매뉴얼

## 한 화면 요약

cron이 8개 셸 wrapper를 스케줄대로 돌리고, 각 wrapper는 `claude -p`를 프롬프트
파일과 함께 호출한다. 상태는 저장소 안 마크다운(`TODO.md`, `STATE.md`, `JOURNAL.md`,
`journal/`, `research/`, `results/`)에 쌓이고, 사람과의 소통은 Telegram으로 한다.
Notion 등 외부 서비스는 쓰지 않는다.

## 파일 구성

```
scripts/
├── daily_brief.sh / daily_executor.sh / researcher.sh / daily_wrap.sh
├── curator.sh / weekly_rollup.sh / telegram_poll.sh / urgent_agent.sh
├── telegram_send.sh    공용 발송기
├── telegram_setup.sh   봇 최초 설정
├── state_push.sh       STATE.md 등 화이트리스트 경로만 main에 직접 push
├── todo_tool.py        TODO.md CLI
├── aggregate_results.py  results/*.tsv → RESULTS.md
├── install_cron.sh     crontab 항목 멱등 설치
└── prompts/*.md         실제 로직
```

## 동작 원리

1. cron이 정해진 시각에 wrapper를 실행
2. wrapper는 `flock`으로 중복 실행을 막고, `PATH`를 보정하고, 저장소로 `cd`
3. `claude -p "$(cat prompt.md)"`를 `--permission-mode acceptEdits`와
   `--allowedTools`로 호출
4. 에이전트가 blackboard 파일을 읽고/쓰고, 필요하면 브랜치+PR을 만들고,
   `scripts/telegram_send.sh`로 사람에게 보고
5. wrapper가 stdout/stderr를 로그 파일에 남기고 종료

## Cron 등록

```bash
./scripts/install_cron.sh   # 멱등 — 기존 항목(MPPI 포함)을 지우지 않음
crontab -l | grep aiworker-motion-planning
```

## 🤝 Multi-agent architecture

`docs/agents.md`의 권한 매트릭스가 각 에이전트가 만질 수 있는 범위를 정의한다.
핵심 불변식: **코드는 항상 브랜치+PR, `main` 직접 push는 `scripts/state_push.sh`를
통과한 blackboard 파일만.**

## 외부 의존성

`claude`, `gh`(GitHub PR), `curl`+`jq`(Telegram Bot API), `flock`, `tmux`(긴급 세션).

## Telegram 양방향 흐름

- **발송**: `scripts/telegram_send.sh` — 4096자 자동 분할, `--data-urlencode`로
  이스케이프, `parse_mode` 미사용(평문 고정)
- **수신**: `scripts/telegram_poll.sh` — 30분마다 `getUpdates`를 확인하고 새
  메시지가 있을 때만 `claude -p`를 호출(비용 절감). offset은
  `~/.local/state/aiworker-motion-planning/telegram_last_update_id`에 저장하며
  **rc==0일 때만 전진**한다(멱등성)

## 🚨 Urgent 키워드

`긴급|즉시|urgent|asap|\bnow\b` (대소문자 무시) 매칭 시 `tmux` 분리 세션에서
`urgent_agent.sh`를 즉시 실행한다. 다른 cron 실행과 락을 공유하지 않는다.

## 로그와 상태

| 종류 | 경로 |
|---|---|
| 로그 | `~/.local/share/aiworker-motion-planning/logs/<agent>-YYYY-MM-DD.log` |
| 락 | `~/.local/state/aiworker-motion-planning/<agent>.lock` |
| Telegram offset | `~/.local/state/aiworker-motion-planning/telegram_last_update_id` |
| 비밀값 | `~/.config/aiworker-motion-planning/telegram.env` (chmod 600) |

## 동작 변경·디버깅

- 동작을 바꾸려면 `scripts/prompts/*.md`만 수정한다(wrapper 재배포 불필요)
- 실패 원인은 해당 날짜 로그 파일의 `=== ... start/end ... rc=N ===` 블록을 확인
- `EXECUTOR_SKIP reason=...`이 반복되면 §`docs/prd.md` 위험 표의 "백로그 고갈" 항목 확인
- 수동 1회 실행: `./scripts/daily_executor.sh && tail -60 ~/.local/share/aiworker-motion-planning/logs/executor-$(date +%F).log`

## 봇 최초 설정

1. Telegram에서 BotFather와 대화 → `/newbot` → 토큰 발급
2. 발급된 봇과 대화를 시작(아무 메시지나 전송)
3. `./scripts/telegram_setup.sh` 실행 — `chat_id`를 자동으로 찾아
   `~/.config/aiworker-motion-planning/telegram.env`를 생성

## 의도적 비-기능

- Notion/외부 DB 미사용 — `TODO.md`가 유일한 canonical source
- 자동 머지 없음 — Curator는 rebase·라벨링·정리만, 머지는 항상 사람
- 좁은 pytest 범위 — executor는 `tests/test_planning_*.py`만 돌린다
  (`.venv`에 torch가 없어 `pytest -q` 전체 실행은 IL 테스트 수집에서 깨진다)
