# Skill 매핑

prompt ↔ wrapper ↔ 도구 허용 목록 ↔ 산출물 ↔ 트리거의 대응표다.
karpathy/autoresearch의 `program.md` 개념을 이 저장소 규모에 맞춘 것이다.

| Prompt | Wrapper | 도구 | 산출물 | 트리거 |
|---|---|---|---|---|
| `researcher.md` | `researcher.sh` | Bash Read Edit Write Grep Glob WebSearch WebFetch | `research/feed.md` 추가, TODO ≤2건 | `0 8 * * *` |
| `brief.md` | `daily_brief.sh` | Bash Read | Telegram 브리핑 | `0 9 * * *` |
| `auto_research.md` | `daily_executor.sh` | Bash Read Edit Write Grep Glob | 브랜치+PR, journal, STATE.md, TODO 갱신 | `0 11,21 * * *` |
| `wrap.md` | `daily_wrap.sh` | Bash Read | Telegram 요약 | `30 22 * * *` |
| `curator.md` | `curator.sh` | Bash Read | PR rebase, stale 브랜치 정리 | `0 23 * * 2,4,6` |
| `weekly.md` | `weekly_rollup.sh` | Bash Read | Telegram 주간 요약 | `0 23 * * 0` |
| `telegram_inbox.md` | `telegram_poll.sh` | Bash Read Edit Write | `research/inbox.md`, TODO 생성 | `*/30 * * * *`(새 메시지 있을 때만) |
| `urgent.md` | `urgent_agent.sh` | Bash Read Edit Write Grep Glob | 즉시 처리 결과 | 긴급 키워드 매칭 |

## 공통 계약 (C1..C5) — 모든 cron skill이 지켜야 함

- **C1** 마지막 단계에서 `research/cron_activity.md`에 한 줄 남긴다:
  `- **HH:MM** \`<script>\` · <80자 이내 한국어 한 줄>`
  `<script> ∈ {brief, executor, researcher, wrap, curator, weekly, inbox, urgent}`
- **C2** 락은 `~/.local/state/aiworker-motion-planning/<skill>.lock`,
  로그는 `~/.local/share/aiworker-motion-planning/logs/<skill>-YYYY-MM-DD.log`
- **C3** Telegram 안전 — 비밀 파일 `chmod 600`, skip일 때 알림 발송 금지(수면 방해)
- **C4** 실패 시 `❌ <skill> 실패: <reason>` 발송 + non-zero exit
- **C5** 시간 예산을 프롬프트 안에 명시한다(§ `docs/prd.md` R-NF-002)

## Skill 추가 체크리스트

1. 목적이 기존 skill과 겹치지 않는가
2. 시간 예산이 명확한가 (C5)
3. 종료 시 stdout 센티널을 정의했는가 (`<NAME>_DONE`/`<NAME>_SKIP`)
4. 실패 경로에서 C4를 지키는가
5. 락이 필요한 만큼 오래 도는가 (C2) — 짧은 스크립트는 락 불필요
6. Telegram 발송 여부와 침묵 규칙을 정했는가 (C3)
7. `docs/agents.md` 권한 매트릭스에 반영했는가
8. `crontab -l`에 기존 MPPI/이 프로젝트 스케줄과 겹치지 않는 시각으로 추가했는가

## 외부 의존성

`claude`(`~/.local/bin/claude`), `gh`(Geonhee-LEE 로그인, `repo` scope), `curl`, `jq`,
`flock`, `tmux`. Notion 등 외부 서비스는 사용하지 않는다 — `TODO.md`가 canonical이다.
