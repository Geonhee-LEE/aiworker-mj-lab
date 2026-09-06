"""benchmark_planning.py의 순수 로직(seed 파싱, TSV metric 인코딩) 단위 시험.

MuJoCo가 필요 없어 빠르게 돈다. 실제 장면 통합은 `benchmark_planning.py`를
headless로 직접 실행해 확인한다(다른 `scripts/*.py`와 같은 관례).

Headless 단독 실행: ``python3 tests/test_planning_benchmark.py``
"""

import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from benchmark_planning import (  # noqa: E402
    OBSTACLE_LAYOUTS,
    format_metric,
    parse_seed_spec,
)


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


def test_obstacle_layouts_contains_default_and_is_well_formed():
    # MP-0031: 장애물 배치를 다양화하는 --obstacle-layout이 기대하는 구조 —
    # 각 항목은 (x, y, z, radius) 4-튜플의 시퀀스여야 한다.
    assert "default" in OBSTACLE_LAYOUTS
    for name, spheres in OBSTACLE_LAYOUTS.items():
        assert len(spheres) >= 1, f"{name}: 장애물이 0개면 --with-obstacle의 의미가 없다"
        for sphere in spheres:
            assert len(sphere) == 4, f"{name}: (x, y, z, radius) 4-튜플이어야 함"


def test_obstacle_layouts_have_distinct_sphere_counts_or_positions():
    # 프리셋들이 서로 실제로 다른 배치인지(복붙 실수 방지) 확인.
    layouts = list(OBSTACLE_LAYOUTS.items())
    for i in range(len(layouts)):
        for j in range(i + 1, len(layouts)):
            assert layouts[i][1] != layouts[j][1], (
                f"{layouts[i][0]}와 {layouts[j][0]}가 완전히 같은 배치입니다"
            )


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
