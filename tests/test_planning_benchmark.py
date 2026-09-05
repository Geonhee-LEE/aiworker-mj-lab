"""benchmark_planning.py의 순수 로직(seed 파싱, TSV metric 인코딩) 단위 시험.

MuJoCo가 필요 없어 빠르게 돈다. 실제 장면 통합은 `benchmark_planning.py`를
headless로 직접 실행해 확인한다(다른 `scripts/*.py`와 같은 관례).

Headless 단독 실행: ``python3 tests/test_planning_benchmark.py``
"""

import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from benchmark_planning import format_metric, parse_seed_spec  # noqa: E402


def test_parse_seed_spec_range():
    assert parse_seed_spec("0-4") == [0, 1, 2, 3, 4]


def test_parse_seed_spec_commas():
    assert parse_seed_spec("0,3,7") == [0, 3, 7]


def test_parse_seed_spec_mixed_and_dedup():
    assert parse_seed_spec("0-2,2,5-6") == [0, 1, 2, 5, 6]


def test_parse_seed_spec_whitespace():
    assert parse_seed_spec(" 0-2, 5 ") == [0, 1, 2, 5]


def test_format_metric_success():
    metric = format_metric(3, True, 12.5, 7, 42, 8.213)
    assert metric == "bench:seed=3,success=1,plan_ms=12.50,iterations=7,checks=42,path_len=8.213"


def test_format_metric_failure_has_na_path_len():
    metric = format_metric(9, False, 5000.0, 4000, 12345, None)
    assert "success=0" in metric
    assert "path_len=NA" in metric


def test_format_metric_omits_planner_and_path_len_after_by_default():
    metric = format_metric(3, True, 12.5, 7, 42, 8.213)
    assert metric == "bench:seed=3,success=1,plan_ms=12.50,iterations=7,checks=42,path_len=8.213"


def test_format_metric_includes_planner_right_after_seed():
    metric = format_metric(3, True, 12.5, 7, 42, 8.213, planner="rrt_star")
    assert metric == (
        "bench:seed=3,planner=rrt_star,success=1,plan_ms=12.50,"
        "iterations=7,checks=42,path_len=8.213"
    )


def test_format_metric_includes_path_len_after_at_the_end():
    metric = format_metric(3, True, 12.5, 7, 42, 8.213, path_len_after=6.5)
    assert metric.endswith(",path_len_after=6.500")


def test_format_metric_path_len_after_none_becomes_na_when_postprocess_enabled():
    # postprocess가 켜졌지만(path_len_after 인자를 넘김) 이 seed는 계획
    # 자체가 실패해 후처리할 경로가 없었던 경우 — 필드 생략이 아니라 NA.
    metric = format_metric(9, False, 5000.0, 4000, 12345, None, path_len_after=None)
    assert metric.endswith(",path_len_after=NA")


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
