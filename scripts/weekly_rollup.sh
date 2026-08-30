#!/usr/bin/env bash
# weekly_rollup — cron wrapper. 실제 로직은 scripts/prompts/weekly.md 에 있다.
set -euo pipefail

REPO=/home/geonhee/Downloads/aiworker-mj-lab
SLUG=aiworker-motion-planning
PROMPT="${REPO}/scripts/prompts/weekly.md"
STATE_DIR="${HOME}/.local/state/${SLUG}"
LOG_DIR="${HOME}/.local/share/${SLUG}/logs"
LOG="${LOG_DIR}/weekly_rollup-$(TZ=Asia/Seoul date +%Y-%m-%d).log"
mkdir -p "${STATE_DIR}" "${LOG_DIR}"

# cron의 PATH는 최소라서 claude/gh/curl이 안 잡힌다.
export PATH="${HOME}/.local/bin:/usr/local/bin:/usr/bin:/bin"
cd "${REPO}"

ALLOWED=( "Bash" "Read" )

{
  echo "=== weekly_rollup start $(date -Iseconds) ==="
  claude -p "$(cat "${PROMPT}")" \
    --output-format text \
    --permission-mode acceptEdits \
    --allowedTools "${ALLOWED[@]}"
  rc=$?
  echo "=== weekly_rollup end $(date -Iseconds) rc=${rc} ==="
  exit ${rc}
} >> "${LOG}" 2>&1
