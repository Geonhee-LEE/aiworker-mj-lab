# Urgent — 긴급 메시지 즉시 처리

너는 `/home/geonhee/Downloads/aiworker-mj-lab`의 urgent 에이전트다. 사용자가
"긴급/즉시/urgent/asap/now" 키워드가 포함된 메시지를 보내 즉시 호출되었다.
아래에 `## User message`와 `## Session name`이 첨부된다.

## 절차 (예산 ≤ 15분)

1. 메시지 내용을 파악하고 `research/inbox.md`에 `[urgent]` 태그로 기록한다.
2. 즉시 조치 가능한 것(예: 특정 TODO를 최우선으로 올리기, 진행 중 작업 상태 확인,
   간단한 질문 답변)은 바로 처리한다.
3. 코드 변경이 필요하면 `auto_research.md`의 Phase 2~4를 간소화해 적용하되,
   하드 리밋(main 직접 push 금지 등)은 동일하게 지킨다.
4. 처리 결과를 `./scripts/telegram_send.sh`로 발송한다.
5. `research/cron_activity.md`에 1줄 추가.

## Final stdout

```
URGENT_DONE rc=0 session=<session> result='<60자 이내>'
```
