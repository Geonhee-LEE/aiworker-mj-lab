#!/usr/bin/env bash
# telegram_poll.sh가 긴급 키워드를 감지하면 tmux 분리 세션에서 이 스크립트를 부른다.
# 사용: urgent_agent.sh "<원본 메시지>" "<세션 이름>"
set -euo pipefail

REPO=/home/geonhee/Downloads/aiworker-mj-lab
SLUG=aiworker-motion-planning
PROMPT="${REPO}/scripts/prompts/urgent.md"
LOG_DIR="${HOME}/.local/share/${SLUG}/logs"
mkdir -p "${LOG_DIR}"

message="${1:?메시지 인자가 필요합니다}"
session="${2:-urgent-$(date +%s)}"
LOG="${LOG_DIR}/urgent-${session}.log"

export PATH="${HOME}/.local/bin:/usr/local/bin:/usr/bin:/bin"
cd "${REPO}"

{
  echo "=== urgent start $(date -Iseconds) session=${session} ==="
  "${REPO}/scripts/telegram_send.sh" "🚨 긴급 작업 시작 [${session}]"

  ALLOWED=( "Bash" "Read" "Edit" "Write" "Grep" "Glob" )
  full_prompt="$(cat "${PROMPT}")

## User message
${message}

## Session name
${session}
"
  set +e
  claude -p "${full_prompt}" \
    --output-format text \
    --permission-mode acceptEdits \
    --allowedTools "${ALLOWED[@]}"
  rc=$?
  set -e

  if [[ ${rc} -ne 0 ]]; then
    "${REPO}/scripts/telegram_send.sh" "❌ 긴급 작업 비정상 종료 [${session}] rc=${rc}"
  fi

  echo "=== urgent end $(date -Iseconds) rc=${rc} ==="
  # 사용자가 tmux attach로 확인할 수 있게 잠시 유지.
  sleep 60
  exit ${rc}
} >> "${LOG}" 2>&1
