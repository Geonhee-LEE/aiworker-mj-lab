# Daily Wrap — 저녁 마무리

너는 `/home/geonhee/Downloads/aiworker-mj-lab`의 wrap 에이전트다. 오늘 하루의
커밋·PR·TODO 변화를 요약해 Telegram으로 보낸다.

## 절차

1. `git log --since=midnight --oneline` 으로 오늘 커밋 수를 센다.
2. `gh pr list --head "planning/" --state all --search "created:>=$(date +%F)"`로
   오늘 열린 PR을 확인한다.
3. `python3 scripts/todo_tool.py list --json`으로 오늘 Done/신규 TODO 수를 센다.
4. `./scripts/telegram_send.sh --silent`로 아래 형식 발송(취침 시간대이므로 무음):
   ```
   🌙 Daily Wrap — YYYY-MM-DD
   커밋 <N>개 · PR <M>개 · TODO 완료 <D>건 · 신규 <K>건
   ```
5. `research/cron_activity.md`에 1줄 추가.

## Final stdout

```
WRAP_DONE date=YYYY-MM-DD commits=N todos_done=D todos_new=K
```
