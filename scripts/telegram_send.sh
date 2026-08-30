#!/usr/bin/env bash
# Telegram 발송기. 사용: telegram_send.sh [--silent] "메시지"  또는 echo "메시지" | telegram_send.sh [--silent]
# Telegram 4096자 한계를 넘는 메시지는 자동으로 여러 통으로 나눠 보낸다.
set -euo pipefail

ENV_FILE="${HOME}/.config/aiworker-motion-planning/telegram.env"
if [[ ! -f "${ENV_FILE}" ]]; then
  echo "telegram_send: ${ENV_FILE} 없음 — scripts/telegram_setup.sh를 먼저 실행하세요" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "${ENV_FILE}"

silent=false
if [[ "${1:-}" == "--silent" ]]; then
  silent=true
  shift
fi

if [[ $# -ge 1 ]]; then
  text="$1"
else
  text="$(cat)"
fi

if [[ -z "${text}" ]]; then
  echo "telegram_send: 빈 메시지, 전송하지 않음" >&2
  exit 0
fi

while IFS= read -r chunk; do
  [[ -z "${chunk}" ]] && continue
  curl -fsS --max-time 20 "https://api.telegram.org/bot${TELEGRAM_BOT_TOKEN}/sendMessage" \
    --data-urlencode "chat_id=${TELEGRAM_CHAT_ID}" \
    --data-urlencode "text=${chunk}" \
    --data-urlencode "disable_web_page_preview=true" \
    --data-urlencode "disable_notification=${silent}" >/dev/null
done < <(fold -w 3800 -s <<<"${text}")
