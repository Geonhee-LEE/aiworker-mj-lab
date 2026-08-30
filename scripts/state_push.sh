#!/usr/bin/env bash
# blackboard 파일(STATE.md, JOURNAL.md, RESULTS.md, TODO.md, journal/, research/,
# results/)만 main에 직접 push하는 유일한 통로다. 코드 경로가 스테이징되어 있으면
# 거부한다 — 코드는 항상 브랜치+PR을 거쳐야 한다(docs/prd.md R-NF-003).
set -euo pipefail

ALLOW='^(TODO\.md|STATE\.md|JOURNAL\.md|RESULTS\.md|journal/|results/|research/)'

staged="$(git diff --cached --name-only)"
if [[ -z "${staged}" ]]; then
  echo "state_push: 스테이징된 변경이 없습니다" >&2
  exit 1
fi

bad="$(echo "${staged}" | grep -vE "${ALLOW}" || true)"
if [[ -n "${bad}" ]]; then
  echo "state_push: 거부 — blackboard 화이트리스트 밖 경로가 스테이징됨:" >&2
  echo "${bad}" >&2
  exit 1
fi

current_branch="$(git rev-parse --abbrev-ref HEAD)"
if [[ "${current_branch}" != "main" ]]; then
  echo "state_push: 거부 — main 브랜치에서만 실행할 수 있습니다 (현재: ${current_branch})" >&2
  exit 1
fi

git commit -m "$(cat <<'MSG'
[state] STATE/TODO/journal/results 자동 갱신

Co-Authored-By: Claude <noreply@anthropic.com>
MSG
)"
git push origin main
