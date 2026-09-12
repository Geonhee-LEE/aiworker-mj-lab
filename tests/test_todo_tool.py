"""scripts/todo_tool.py의 파싱·렌더링 왕복·CLI 부작용 단위 시험.

MuJoCo가 필요 없어 빠르게 돈다(다른 `scripts/*.py`와 같은 관례).

Headless 단독 실행: ``python3 tests/test_todo_tool.py``
"""

import argparse
import pathlib
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import todo_tool  # noqa: E402

SAMPLE = """# TODO — sample

- Last update: `2026-01-01 00:00 KST`
- Open: 2
- Next ID: `MP-0003`

## Doing
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|
| MP-0002 | P1 | P1 | claude | in progress task | planning/x | ☑ |

## Today
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|

## Blocked
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|

## Backlog
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|
| MP-0001 | P0 | P0 | claude | first task | | ☐ |

## Done
| ID | Priority | Phase | Owner | Title | Branch | UserTest |
|---|---|---|---|---|---|---|
"""


def test_parse_extracts_rows_by_section():
    sections = todo_tool.parse(SAMPLE)
    assert [r.id for r in sections["Doing"]] == ["MP-0002"]
    assert [r.id for r in sections["Backlog"]] == ["MP-0001"]
    assert sections["Today"] == []
    assert sections["Done"] == []


def test_parse_reads_usertest_and_branch():
    sections = todo_tool.parse(SAMPLE)
    row = sections["Doing"][0]
    assert row.title == "in progress task"
    assert row.usertest is True
    assert row.branch == "planning/x"


def test_parse_keeps_row_when_title_contains_escaped_pipe():
    """MP-0034 회귀 시험: 제목에 ``_escape``가 붙인 ``\\|``가 있어도 행이
    버려지지 않고, 파싱된 title에는 이스케이프 없는 리터럴 ``|``가 그대로
    남는다(과거엔 naive ``split("|")``가 셀 개수를 어긋나게 해 행 전체가
    조용히 버려졌다 — MP-0033 ID 충돌 사고의 근본 원인)."""
    text = SAMPLE.replace(
        "| MP-0002 | P1 | P1 | claude | in progress task | planning/x | ☑ |",
        "| MP-0002 | P1 | P1 | claude | in progress \\| with pipe | planning/x | ☑ |",
    )
    sections = todo_tool.parse(text)
    assert [r.id for r in sections["Doing"]] == ["MP-0002"]
    assert sections["Doing"][0].title == "in progress | with pipe"


def test_parse_ignores_header_and_separator_rows():
    sections = todo_tool.parse(SAMPLE)
    all_ids = [r.id for rows in sections.values() for r in rows]
    assert "ID" not in all_ids
    assert "---" not in all_ids


def test_parse_ignores_malformed_id():
    text = SAMPLE.replace("| MP-0001 |", "| NOT-AN-ID |")
    sections = todo_tool.parse(text)
    assert sections["Backlog"] == []


def test_escape_and_unescape_round_trip_pipe():
    original = "a | b | c"
    assert todo_tool._unescape(todo_tool._escape(original)) == original


def test_render_parse_round_trip_preserves_rows():
    sections = todo_tool.parse(SAMPLE)
    rendered = todo_tool.render(sections, now="2026-01-02 00:00 KST")
    reparsed = todo_tool.parse(rendered)
    for status in todo_tool.STATUSES:
        assert [r.id for r in reparsed[status]] == [r.id for r in sections[status]]
    row = reparsed["Doing"][0]
    assert row.title == "in progress task"
    assert row.usertest is True


def test_render_parse_round_trip_preserves_title_with_pipe():
    """렌더→재파싱을 거쳐도 제목 속 리터럴 ``|``가 보존되고, 새 ID 발급
    (``_next_id``)도 이 행을 정상적으로 봐 건너뛰지 않는다(MP-0033 사고
    재발 방지)."""
    sections = todo_tool.parse(SAMPLE)
    sections["Doing"][0].title = "fix a | b bug"
    rendered = todo_tool.render(sections, now="2026-01-02 00:00 KST")
    reparsed = todo_tool.parse(rendered)
    assert reparsed["Doing"][0].title == "fix a | b bug"
    assert todo_tool._next_id(reparsed) == "MP-0003"


def test_render_truncates_long_title():
    sections = {status: [] for status in todo_tool.STATUSES}
    sections["Backlog"] = [
        todo_tool.Row(
            id="MP-0009",
            priority="P0",
            phase="P0",
            owner="claude",
            title="x" * 150,
        )
    ]
    rendered = todo_tool.render(sections, now="2026-01-02 00:00 KST")
    reparsed = todo_tool.parse(rendered)
    assert len(reparsed["Backlog"][0].title) == 100
    assert reparsed["Backlog"][0].title.endswith("...")


def test_next_id_is_max_plus_one_across_sections():
    sections = todo_tool.parse(SAMPLE)
    assert todo_tool._next_id(sections) == "MP-0003"


@pytest.fixture
def isolated_todo(tmp_path, monkeypatch):
    todo_path = tmp_path / "TODO.md"
    todo_path.write_text(SAMPLE, encoding="utf-8")
    monkeypatch.setattr(todo_tool, "TODO_PATH", todo_path)
    return todo_path


def test_cmd_add_then_list_round_trip(isolated_todo, capsys):
    args = argparse.Namespace(
        title="new backlog item",
        priority="P2",
        phase="P3",
        owner="claude",
        status="Backlog",
        branch="",
        usertest=False,
    )
    todo_tool.cmd_add(args)
    new_id = capsys.readouterr().out.strip()
    assert new_id == "MP-0003"

    sections = todo_tool.parse(isolated_todo.read_text(encoding="utf-8"))
    ids = [r.id for r in sections["Backlog"]]
    assert new_id in ids


def test_cmd_set_moves_row_between_sections(isolated_todo):
    args = argparse.Namespace(id="MP-0001", status="Today", branch=None, usertest=None, priority=None)
    todo_tool.cmd_set(args)

    sections = todo_tool.parse(isolated_todo.read_text(encoding="utf-8"))
    assert [r.id for r in sections["Backlog"]] == []
    assert [r.id for r in sections["Today"]] == ["MP-0001"]


def test_cmd_set_unknown_id_raises(isolated_todo):
    args = argparse.Namespace(id="MP-9999", status="Today", branch=None, usertest=None, priority=None)
    with pytest.raises(SystemExit):
        todo_tool.cmd_set(args)


def test_cmd_check_detects_duplicate_id(isolated_todo, capsys):
    text = isolated_todo.read_text(encoding="utf-8")
    text = text.replace(
        "| MP-0001 | P0 | P0 | claude | first task | | ☐ |",
        "| MP-0001 | P0 | P0 | claude | first task | | ☐ |\n"
        "| MP-0001 | P0 | P0 | claude | duplicate | | ☐ |",
    )
    isolated_todo.write_text(text, encoding="utf-8")

    with pytest.raises(SystemExit):
        todo_tool.cmd_check(argparse.Namespace())
    assert "중복 ID" in capsys.readouterr().err


def test_cmd_check_detects_multiple_claude_doing(isolated_todo, capsys):
    text = isolated_todo.read_text(encoding="utf-8")
    text = text.replace(
        "| MP-0002 | P1 | P1 | claude | in progress task | planning/x | ☑ |",
        "| MP-0002 | P1 | P1 | claude | in progress task | planning/x | ☑ |\n"
        "| MP-0003 | P1 | P1 | claude | second doing | planning/y | ☐ |",
    )
    isolated_todo.write_text(text, encoding="utf-8")

    with pytest.raises(SystemExit):
        todo_tool.cmd_check(argparse.Namespace())
    assert "동시 1건만 허용" in capsys.readouterr().err


def test_cmd_check_passes_on_clean_sample(isolated_todo, capsys):
    todo_tool.cmd_check(argparse.Namespace())
    assert capsys.readouterr().out.strip() == "PASS"


def test_cmd_next_returns_doing_before_today(isolated_todo, capsys):
    todo_tool.cmd_next(argparse.Namespace())
    out = capsys.readouterr().out
    assert '"MP-0002"' in out


def test_cmd_next_returns_none_when_no_actionable_row(isolated_todo, capsys):
    args = argparse.Namespace(id="MP-0002", status="Done", branch=None, usertest=None, priority=None)
    todo_tool.cmd_set(args)
    capsys.readouterr()

    todo_tool.cmd_next(argparse.Namespace())
    assert capsys.readouterr().out.strip() == "null"
