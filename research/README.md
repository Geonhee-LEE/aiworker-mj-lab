# research/

문헌·기법 조사 결과와 미해결 논의를 append-only로 쌓는 곳이다.

| 파일 | 용도 |
|---|---|
| `feed.md` | 최근 조사 결과 큐 (cap 30, REVIEW는 상위 5만 읽음) |
| `YYYY-MM/NNN.md` | feed 항목의 전체 내용 (3자리 순번, append-only) |
| `decisions.md` | ADR-lite, `D-NNN`, prepend, 단조 증가 |
| `deliberations.md` | 미해결 trade-off/질문, `Q-NNN`, prepend |
| `inbox.md` | Telegram 수신 메시지 적재 |
| `cron_activity.md` | 모든 cron 실행의 1줄 로그 (C1 계약) |

## D-NNN / Q-NNN 발급 규율

사소한 변경(변수명, 한 줄 문서 갱신)에는 D-NNN을 발급하지 않는다 — journal 한 줄로
충분하다. 판단이 어려우면 기본값은 "발급하지 않음"이다.
