import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from aggregate_results import _parse_bench_metric, _success_rate_lines, _wilson_ci


def test_wilson_ci_all_success_is_below_one():
    phat, lo, hi = _wilson_ci(50, 50)
    assert phat == 1.0
    assert 0.0 < lo < 1.0
    assert hi == 1.0


def test_wilson_ci_half_success_is_centered():
    phat, lo, hi = _wilson_ci(25, 50)
    assert phat == 0.5
    assert lo < 0.5 < hi


def test_wilson_ci_narrows_with_more_samples():
    _, lo_small, hi_small = _wilson_ci(9, 10)
    _, lo_large, hi_large = _wilson_ci(90, 100)
    assert (hi_large - lo_large) < (hi_small - lo_small)


def test_parse_bench_metric_extracts_fields():
    fields = _parse_bench_metric("bench:seed=0,planner=rrt_connect,success=1,plan_ms=14.29")
    assert fields == {
        "seed": "0",
        "planner": "rrt_connect",
        "success": "1",
        "plan_ms": "14.29",
    }


def test_parse_bench_metric_ignores_non_bench_rows():
    assert _parse_bench_metric("qual:tests-pass") is None


def test_success_rate_lines_groups_by_planner():
    rows = [
        {"metric": "bench:seed=0,planner=rrt_connect,success=1"},
        {"metric": "bench:seed=1,planner=rrt_connect,success=0"},
        {"metric": "bench:seed=0,planner=rrt_star,success=1"},
    ]
    lines = _success_rate_lines(rows)
    text = "\n".join(lines)
    assert "rrt_connect: 0.500" in text
    assert "rrt_star: 1.000" in text


def test_success_rate_lines_defaults_to_all_group_without_planner():
    rows = [
        {"metric": "bench:seed=0,success=1"},
        {"metric": "bench:seed=1,success=1"},
    ]
    lines = _success_rate_lines(rows)
    assert any(line.startswith("- all: 1.000") for line in lines)


def test_success_rate_lines_empty_when_no_bench_rows():
    rows = [{"metric": "qual:tests-pass"}]
    assert _success_rate_lines(rows) == []
