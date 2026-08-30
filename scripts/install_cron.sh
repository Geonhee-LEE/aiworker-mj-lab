#!/usr/bin/env bash
# aiworker-motion-planning cron 항목을 멱등하게 설치한다. 기존 crontab 항목
# (다른 프로젝트의 스케줄 포함)은 절대 건드리지 않는다.
set -euo pipefail

REPO="/home/geonhee/Downloads/aiworker-mj-lab"
MARKER_START="# --- aiworker-motion-planning (system TZ = Asia/Seoul) ---"
MARKER_END="# --- end aiworker-motion-planning ---"

BLOCK=$(cat << CRONEOF
${MARKER_START}
0    8 * * *      ${REPO}/scripts/researcher.sh
0    9 * * *      ${REPO}/scripts/daily_brief.sh
0 11,21 * * *     ${REPO}/scripts/daily_executor.sh
30  22 * * *      ${REPO}/scripts/daily_wrap.sh
0   23 * * 2,4,6  ${REPO}/scripts/curator.sh
0   23 * * 0      ${REPO}/scripts/weekly_rollup.sh
*/30 * * * *      ${REPO}/scripts/telegram_poll.sh
${MARKER_END}
CRONEOF
)

current="$(crontab -l 2>/dev/null || true)"

if echo "${current}" | grep -qF "${MARKER_START}"; then
  # 기존 블록을 새 블록으로 교체 (마커 사이만)
  updated="$(echo "${current}" | awk -v start="${MARKER_START}" -v end="${MARKER_END}" '
    $0 == start { skip = 1 }
    !skip { print }
    $0 == end { skip = 0 }
  ')"
  new_crontab="${updated}
${BLOCK}"
else
  new_crontab="${current}
${BLOCK}"
fi

echo "${new_crontab}" | crontab -
echo "설치 완료. 확인: crontab -l | grep aiworker-motion-planning"
