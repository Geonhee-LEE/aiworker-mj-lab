#!/usr/bin/env bash
# researcher — cron wrapper. 실제 로직은 scripts/prompts/researcher.md 에 있다.
set -euo pipefail

REPO=/home/geonhee/Downloads/aiworker-mj-lab
SLUG=aiworker-motion-planning
PROMPT="${REPO}/scripts/prompts/researcher.md"
STATE_DIR="${HOME}/.local/state/${SLUG}"
LOG_DIR="${HOME}/.local/share/${SLUG}/logs"
LOG="${LOG_DIR}/researcher-$(TZ=Asia/Seoul date +%Y-%m-%d).log"
mkdir -p "${STATE_DIR}" "${LOG_DIR}"

# 단일 인스턴스 보장. 이전 실행이 아직 돌면 조용히 건너뛴다.
exec 9>"${STATE_DIR}/researcher.lock"
if ! flock -n 9; then
  echo "[$(date -Iseconds)] researcher already running; skipping" >> "${LOG}"; exit 0
fi

# cron의 PATH는 최소라서 claude/gh/curl이 안 잡힌다.
export PATH="${HOME}/.local/bin:/usr/local/bin:/usr/bin:/bin"
cd "${REPO}"

ALLOWED=( "Bash" "Read" "Edit" "Write" "Grep" "Glob" "WebSearch" "WebFetch" )

{
  echo "=== researcher start $(date -Iseconds) ==="
  claude -p "$(cat "${PROMPT}")" \
    --output-format text \
    --permission-mode acceptEdits \
    --allowedTools "${ALLOWED[@]}"
  rc=$?
  echo "=== researcher end $(date -Iseconds) rc=${rc} ==="
  exit ${rc}
} >> "${LOG}" 2>&1
