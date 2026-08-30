"""TODO.md를 파싱·수정하는 CLI. 표 정렬·ID 발급·중복 검사를 기계적으로 보장한다.

에이전트가 마크다운 표를 자유 편집하면 정렬이 깨지고 ID가 중복될 수 있다. 이
도구를 거치면 그런 실패가 구조적으로 사라진다.
"""

import argparse
import json
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
TODO_PATH = REPO_ROOT / "TODO.md"

STATUSES = ("Doing", "Today", "Blocked", "Backlog", "Done")
PRIORITIES = ("P0", "P1", "P2", "P3")
PHASES = ("P0", "P1", "P2", "P3", "P4", "P5", "P6")
OWNERS = ("claude", "user")
COLUMNS = ("ID", "Priority", "Phase", "Owner", "Title", "Branch", "UserTest")
ID_RE = re.compile(r"^MP-(\d{4})$")
SECTION_RE = re.compile(r"^## (Doing|Today|Blocked|Backlog|Done)\s*$")


@dataclass
class Row:
    """TODO.md 표 한 행."""

    id: str
    priority: str
    phase: str
    owner: str
    title: str
    branch: str = ""
    usertest: bool = False
    status: str = "Backlog"


def _escape(cell):
    return cell.replace("|", "\\|")


def _unescape(cell):
    return cell.replace("\\|", "|")


def parse(text):
    """TODO.md 본문에서 status → [Row] 매핑을 반환한다."""
    sections = {status: [] for status in STATUSES}
    current = None
    for line in text.splitlines():
        match = SECTION_RE.match(line.strip())
        if match:
            current = match.group(1)
            continue
        if current is None or not line.strip().startswith("|"):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) != len(COLUMNS) or cells[0] in ("ID", "---"):
            continue
        if not ID_RE.match(cells[0]):
            continue
        sections[current].append(
            Row(
                id=cells[0],
                priority=cells[1],
                phase=cells[2],
                owner=cells[3],
                title=_unescape(cells[4]),
                branch=cells[5],
                usertest=cells[6] == "☑",
                status=current,
            )
        )
    return sections


def _next_id(sections):
    max_seen = 0
    for rows in sections.values():
        for row in rows:
            max_seen = max(max_seen, int(ID_RE.match(row.id).group(1)))
    return f"MP-{max_seen + 1:04d}"


def _render_table(rows):
    if not rows:
        return "| ID | Priority | Phase | Owner | Title | Branch | UserTest |\n|---|---|---|---|---|---|---|"
    lines = ["| ID | Priority | Phase | Owner | Title | Branch | UserTest |", "|---|---|---|---|---|---|---|"]
    for row in rows:
        title = _escape(row.title)
        if len(title) > 100:
            title = title[:97] + "..."
        mark = "☑" if row.usertest else "☐"
        lines.append(
            f"| {row.id} | {row.priority} | {row.phase} | {row.owner} "
            f"| {title} | {row.branch} | {mark} |"
        )
    return "\n".join(lines)


def render(sections, *, now):
    open_count = sum(
        len(sections[s]) for s in ("Doing", "Today", "Blocked", "Backlog")
    )
    next_id = _next_id(sections)
    done = sorted(sections["Done"], key=lambda r: r.id, reverse=True)[:20]
    parts = [
        "# TODO — AIWORKER 오른팔 모션 플래닝",
        "",
        "_이 파일이 작업 상태의 유일한 권위다. 사람과 cron 에이전트가 함께 수정한다._",
        "_기계적 수정은 `scripts/todo_tool.py`를 쓴다 (표 정렬·ID 발급·중복 검사 포함)._",
        "",
        f"- Last update: `{now}`",
        f"- Open (Doing + Today + Blocked + Backlog): **{open_count}**",
        f"- Next ID: `{next_id}`",
        "",
    ]
    for status in ("Doing", "Today", "Blocked", "Backlog"):
        parts.append(f"## {status}")
        parts.append(_render_table(sections[status]))
        parts.append("")
    parts.append("## Done")
    parts.append(_render_table(done))
    parts.append("")
    parts.append(
        "> Mirror는 없음 — 이 파일 자체가 canonical이다. "
        "갱신: `python3 scripts/todo_tool.py check`"
    )
    return "\n".join(parts) + "\n"


def _load():
    if not TODO_PATH.exists():
        raise SystemExit(f"todo_tool: {TODO_PATH} 없음")
    return parse(TODO_PATH.read_text(encoding="utf-8"))


def _save(sections):
    import datetime

    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M KST")
    TODO_PATH.write_text(render(sections, now=now), encoding="utf-8")


def cmd_check(_args):
    sections = _load()
    seen_ids = {}
    errors = []
    for status, rows in sections.items():
        for row in rows:
            if row.id in seen_ids:
                errors.append(f"중복 ID: {row.id} ({seen_ids[row.id]}, {status})")
            seen_ids[row.id] = status
            if row.priority not in PRIORITIES:
                errors.append(f"{row.id}: 잘못된 Priority {row.priority!r}")
            if row.phase not in PHASES:
                errors.append(f"{row.id}: 잘못된 Phase {row.phase!r}")
            if row.owner not in OWNERS:
                errors.append(f"{row.id}: 잘못된 Owner {row.owner!r}")
    doing_by_owner = [r for r in sections["Doing"] if r.owner == "claude"]
    if len(doing_by_owner) > 1:
        errors.append(
            f"claude Doing 항목이 {len(doing_by_owner)}개 — 동시 1건만 허용"
        )
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
    print("PASS")


def cmd_list(args):
    sections = _load()
    rows = [row for rows in sections.values() for row in rows]
    if args.status:
        rows = [r for r in rows if r.status == args.status]
    if args.owner:
        rows = [r for r in rows if r.owner == args.owner]
    if args.phase:
        rows = [r for r in rows if r.phase == args.phase]
    if args.json:
        print(json.dumps([asdict(r) for r in rows], ensure_ascii=False, indent=2))
        return
    for row in rows:
        print(f"{row.id}\t{row.status}\t{row.priority}\t{row.phase}\t{row.title}")


def cmd_add(args):
    sections = _load()
    new_id = _next_id(sections)
    row = Row(
        id=new_id,
        priority=args.priority,
        phase=args.phase,
        owner=args.owner,
        title=args.title,
        branch=args.branch or "",
        usertest=args.usertest,
        status=args.status,
    )
    sections[args.status].append(row)
    _save(sections)
    print(new_id)


def cmd_set(args):
    sections = _load()
    target = None
    for rows in sections.values():
        for row in rows:
            if row.id == args.id:
                target = row
                break
    if target is None:
        raise SystemExit(f"todo_tool: {args.id} 없음")
    if args.status:
        sections[target.status].remove(target)
        target.status = args.status
        sections[target.status].append(target)
    if args.branch is not None:
        target.branch = args.branch
    if args.usertest is not None:
        target.usertest = args.usertest
    if args.priority:
        target.priority = args.priority
    _save(sections)
    print(target.id)


def cmd_next(_args):
    sections = _load()
    candidates = [r for r in sections["Doing"] if r.owner == "claude"]
    if candidates:
        print(json.dumps(asdict(candidates[0]), ensure_ascii=False))
        return
    today = sorted(
        (r for r in sections["Today"] if r.owner == "claude"),
        key=lambda r: (PRIORITIES.index(r.priority), PHASES.index(r.phase)),
    )
    if today:
        print(json.dumps(asdict(today[0]), ensure_ascii=False))
        return
    print(json.dumps(None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p_list = sub.add_parser("list")
    p_list.add_argument("--status", choices=STATUSES)
    p_list.add_argument("--owner", choices=OWNERS)
    p_list.add_argument("--phase", choices=PHASES)
    p_list.add_argument("--json", action="store_true")
    p_list.set_defaults(func=cmd_list)

    p_add = sub.add_parser("add")
    p_add.add_argument("title")
    p_add.add_argument("--priority", choices=PRIORITIES, required=True)
    p_add.add_argument("--phase", choices=PHASES, required=True)
    p_add.add_argument("--owner", choices=OWNERS, required=True)
    p_add.add_argument("--status", choices=STATUSES, default="Backlog")
    p_add.add_argument("--branch", default="")
    p_add.add_argument("--usertest", action="store_true")
    p_add.set_defaults(func=cmd_add)

    p_set = sub.add_parser("set")
    p_set.add_argument("id")
    p_set.add_argument("--status", choices=STATUSES)
    p_set.add_argument("--branch")
    p_set.add_argument("--priority", choices=PRIORITIES)
    usertest_group = p_set.add_mutually_exclusive_group()
    usertest_group.add_argument("--usertest", dest="usertest", action="store_true", default=None)
    usertest_group.add_argument("--no-usertest", dest="usertest", action="store_false")
    p_set.set_defaults(func=cmd_set)

    p_next = sub.add_parser("next")
    p_next.set_defaults(func=cmd_next)

    p_check = sub.add_parser("check")
    p_check.set_defaults(func=cmd_check)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
