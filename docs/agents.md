# 에이전트 명세

모든 에이전트는 **단일 책임 + 단일 prompt 파일 + 단일 cron 슬롯**을 가진다.
셸 스크립트(`scripts/*.sh`)는 wrapper일 뿐이며 실제 로직은 전부 `scripts/prompts/*.md`에
있다 — 동작을 바꾸려면 프롬프트만 수정하면 되고 재빌드가 필요 없다.

```
┌─────────────┐      ┌──────────────┐      ┌───────────────┐
│  cron (8종) │ ───▶ │ scripts/*.sh │ ───▶ │ claude -p ...  │
└─────────────┘      └──────────────┘      └───────┬───────┘
                                                     ▼
                                    ┌────────────────────────────────┐
                                    │ blackboard: TODO.md STATE.md    │
                                    │ JOURNAL.md journal/ research/   │
                                    │ results/*.tsv                   │
                                    └────────────────────────────────┘
```

## 핵심 에이전트 4종

| 에이전트 | 목적 | Cron | Prompt |
|---|---|---|---|
| Researcher | 문헌·기법 조사 → 후속 TODO 제안 | `0 8 * * *` | `scripts/prompts/researcher.md` |
| Executor | TODO 1건 선택 → 구현·테스트·PR | `0 11,21 * * *` | `scripts/prompts/auto_research.md` |
| Curator | PR rebase, stale 브랜치 정리, 라벨링 | `0 23 * * 2,4,6` | `scripts/prompts/curator.md` |
| Brief/Wrap | 아침 브리핑 / 저녁 마무리 요약 | `0 9 * * *` / `30 22 * * *` | `scripts/prompts/brief.md` / `wrap.md` |

## 보조 에이전트 4종

| 에이전트 | 목적 | Cron | Prompt |
|---|---|---|---|
| Weekly | 주간 롤업 | `0 23 * * 0` | `scripts/prompts/weekly.md` |
| Telegram inbox | 수신 메시지 적재·TODO화 | `*/30 * * * *` (새 메시지 있을 때만) | `scripts/prompts/telegram_inbox.md` |
| Urgent | 긴급 키워드 즉시 처리 | 이벤트 기반(tmux) | `scripts/prompts/urgent.md` |
| (없음) mirror | 사용하지 않음 — TODO.md가 이미 canonical이라 미러가 불필요 | — | — |

## 권한 매트릭스

| 에이전트 | Bash | Read | Edit/Write | WebSearch/Fetch | git push | gh pr create |
|---|---|---|---|---|---|---|
| Researcher | ✓ | ✓ | TODO만 | ✓ | ✗ | ✗ |
| Executor | ✓ | ✓ | ✓ | ✗ | `planning/*` 브랜치만 | ✓ |
| Curator | ✓ | ✓ | ✗ | ✗ | rebase만(force-with-lease) | ✗ |
| Brief/Wrap | ✓ | ✓ | STATE 요약만 | ✗ | ✗ | ✗ |
| Telegram inbox | ✓ | ✓ | `research/inbox.md`, TODO | ✗ | ✗ | ✗ |
| Urgent | ✓ | ✓ | ✓ | ✗ | `planning/*` 브랜치만 | ✓ |

모든 에이전트에 공통으로 적용되는 하드 리밋(§`docs/prd.md` R-NF-003):

- `main`에 코드 직접 push 금지 — 코드는 항상 `planning/<phase>-<slug>` 브랜치 + PR
- `crontab`/`systemctl`/`apt`/`pip install` 등 시스템 변경 금지
- 저장소 밖 `rm -rf` 금지, 사용자 dotfile 수정 금지
- 시뮬레이션 실행 2분 초과 금지 → test request로 사람에게 위임
- `src/ffw_sh5_grasp/{kinematics,control,imitation}/**` 수정은 `research/deliberations.md`에
  Q-NNN을 먼저 남겨야 한다. 새 코드는 기본적으로 `src/ffw_sh5_grasp/planning/`에 추가한다

## 새 에이전트 추가 절차

1. `scripts/prompts/<name>.md` 작성 — 목적, 입력, 출력, 종료 신호(stdout 센티널)
2. `scripts/<name>.sh` wrapper 작성 — B4 템플릿 복사, `ALLOWED` 배열만 조정
3. `docs/skills.md`에 매핑 행 추가 (prompt ↔ wrapper ↔ 산출물 ↔ 트리거)
4. 이 표(권한 매트릭스)에 행 추가
5. `crontab -l`에 항목 추가 — 기존 스케줄과 겹치지 않는 시각 선택
6. 손으로 1회 실행 → 로그·Telegram 확인
7. `docs/automation.md`의 "동작 변경·디버깅" 절에 새 로그 경로 추가
