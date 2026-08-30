#!/usr/bin/env bash
# 30분마다 Telegram 새 메시지를 확인한다. 새 메시지가 없으면 claude를 부르지
# 않고(비용 절감) offset만 전진시킨다. 긴급 키워드는 tmux 분리 세션으로 즉시 처리.
set -euo pipefail

REPO=/home/geonhee/Downloads/aiworker-mj-lab
SLUG=aiworker-motion-planning
PROMPT="${REPO}/scripts/prompts/telegram_inbox.md"
STATE_DIR="${HOME}/.local/state/${SLUG}"
LOG_DIR="${HOME}/.local/share/${SLUG}/logs"
LOG="${LOG_DIR}/telegram_poll-$(TZ=Asia/Seoul date +%Y-%m-%d).log"
STATE_FILE="${STATE_DIR}/telegram_last_update_id"
ENV_FILE="${HOME}/.config/${SLUG}/telegram.env"
mkdir -p "${STATE_DIR}" "${LOG_DIR}"

exec 9>"${STATE_DIR}/telegram_poll.lock"
if ! flock -n 9; then
  echo "[$(date -Iseconds)] telegram_poll already running; skipping" >> "${LOG}"
  exit 0
fi

export PATH="${HOME}/.local/bin:/usr/local/bin:/usr/bin:/bin"
cd "${REPO}"

if [[ ! -f "${ENV_FILE}" ]]; then
  echo "[$(date -Iseconds)] ${ENV_FILE} 없음 — telegram_setup.sh 먼저 실행" >> "${LOG}"
  exit 0
fi
# shellcheck disable=SC1090
source "${ENV_FILE}"

{
  echo "=== telegram_poll start $(date -Iseconds) ==="

  last_id=$(cat "${STATE_FILE}" 2>/dev/null || echo 0)
  offset=$((last_id + 1))

  resp=$(curl -fsS --max-time 15 \
    "https://api.telegram.org/bot${TELEGRAM_BOT_TOKEN}/getUpdates?offset=${offset}&allowed_updates=%5B%22message%22%5D&timeout=0")

  if [[ "$(echo "${resp}" | jq -r '.ok')" != "true" ]]; then
    echo "[$(date -Iseconds)] getUpdates failed: ${resp}"
    exit 1
  fi

  new_msgs=$(echo "${resp}" | jq -c --arg cid "${TELEGRAM_CHAT_ID}" '
    [.result[]
      | select(.message != null)
      | select((.message.chat.id|tostring) == $cid)
      | select(.message.text != null)
      | {update_id, ts: (.message.date | strftime("%Y-%m-%dT%H:%M:%S+09:00")), text: .message.text}]
  ')
  count=$(echo "${new_msgs}" | jq 'length')

  if [[ "${count}" -eq 0 ]]; then
    max_seen=$(echo "${resp}" | jq -r '[.result[].update_id] | max // empty')
    if [[ -n "${max_seen}" ]]; then
      echo "${max_seen}" > "${STATE_FILE}"
    fi
    echo "=== telegram_poll end $(date -Iseconds) rc=0 (no new messages) ==="
    exit 0
  fi

  # 긴급 키워드 감지 → tmux 분리 세션으로 즉시 처리.
  URGENT_RE='긴급|즉시|urgent|asap|\bnow\b'
  echo "${new_msgs}" | jq -c '.[]' | while IFS= read -r entry; do
    text=$(echo "${entry}" | jq -r '.text')
    if echo "${text}" | grep -iqE "${URGENT_RE}"; then
      session="amp-urgent-$(date +%Y%m%d-%H%M%S)-$$"
      tmux new-session -d -s "${session}" \
        "${REPO}/scripts/urgent_agent.sh $(printf '%q' "${text}") $(printf '%q' "${session}")" \
        2>>"${LOG}" || echo "[$(date -Iseconds)] tmux spawn failed for ${session}"
      sleep 1
    fi
  done

  payload=$(echo "${new_msgs}" | jq -c '.[] | {ts, text}')
  new_max=$(echo "${new_msgs}" | jq -r '[.[].update_id] | max')

  ALLOWED=( "Bash" "Read" "Edit" "Write" )
  full_prompt="$(cat "${PROMPT}")

## Messages to append (this invocation)

\`\`\`
${payload}
\`\`\`
"
  claude -p "${full_prompt}" \
    --output-format text \
    --permission-mode acceptEdits \
    --allowedTools "${ALLOWED[@]}"
  rc=$?

  if [[ ${rc} -eq 0 ]]; then
    echo "${new_max}" > "${STATE_FILE}"
    "${REPO}/scripts/telegram_send.sh" --silent "📥 inbox에 ${count}건 추가" || true
  fi

  echo "=== telegram_poll end $(date -Iseconds) rc=${rc} ==="
  exit ${rc}
} >> "${LOG}" 2>&1
