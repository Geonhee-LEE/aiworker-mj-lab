#!/usr/bin/env bash
# Telegram 봇 최초 설정. BotFather에서 발급한 토큰으로 chat_id를 자동으로 찾는다.
#
# 절차:
#   1. Telegram에서 @BotFather와 대화 → /newbot → 토큰 발급
#   2. 발급된 봇과 대화를 시작 (아무 메시지나 전송)
#   3. 이 스크립트를 TELEGRAM_BOT_TOKEN 인자와 함께 실행
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "사용법: $0 <BOT_TOKEN>" >&2
  exit 1
fi
token="$1"

CONFIG_DIR="${HOME}/.config/aiworker-motion-planning"
ENV_FILE="${CONFIG_DIR}/telegram.env"
mkdir -p "${CONFIG_DIR}"
chmod 700 "${CONFIG_DIR}"

echo "getUpdates 확인 중 — 봇과 먼저 대화를 시작했는지 확인하세요..."
resp="$(curl -fsS "https://api.telegram.org/bot${token}/getUpdates")"
chat_id="$(echo "${resp}" | jq -r '[.result[].message.chat.id] | last // empty')"

if [[ -z "${chat_id}" ]]; then
  echo "chat_id를 찾지 못했습니다. 봇과 대화를 시작한 뒤 다시 실행하세요." >&2
  echo "응답: ${resp}" >&2
  exit 1
fi

cat > "${ENV_FILE}" << ENVEOF
TELEGRAM_BOT_TOKEN=${token}
TELEGRAM_CHAT_ID=${chat_id}
ENVEOF
chmod 600 "${ENV_FILE}"

echo "설정 완료: ${ENV_FILE} (chat_id=${chat_id})"
echo "확인: ./scripts/telegram_send.sh '✅ aiworker-motion-planning 설정 확인'"
