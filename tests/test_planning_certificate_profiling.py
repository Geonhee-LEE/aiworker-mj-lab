"""profile_certificate_caching.py의 순수 로직(seed 파싱, metric 포맷, 손익분기
판정) 단위 시험. MuJoCo가 필요 없어 빠르게 돈다. 실제 장면 통합은
`profile_certificate_caching.py`를 headless로 직접 실행해 확인한다(다른
`scripts/*.py`와 같은 관례).

Headless 단독 실행: ``python3 tests/test_planning_certificate_profiling.py``
"""

import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from profile_certificate_caching import format_metric, parse_seed_spec  # noqa: E402


def test_parse_seed_spec_range():
    assert parse_seed_spec("0-4") == [0, 1, 2, 3, 4]


def test_parse_seed_spec_commas():
    assert parse_seed_spec("0,3,7") == [0, 3, 7]


def _result(cost_ratio, expected_savings):
    return {
        "cost_ratio": cost_ratio,
        "is_valid_ms": 0.15,
        "clearance_ms": 0.15 * cost_ratio,
        "median_clearance_rad": 0.1,
        "p10_clearance_rad": 0.08,
        "resolution_rad": 0.05,
        "expected_savings": expected_savings,
        "n_sampled": 150,
        "worth_it": expected_savings > cost_ratio,
    }


def test_format_metric_not_worth():
    metric = format_metric(_result(cost_ratio=7.6, expected_savings=4.1))
    assert "verdict=not-worth" in metric
    assert "cost_ratio=7.6" in metric


def test_format_metric_worth_poc():
    metric = format_metric(_result(cost_ratio=2.0, expected_savings=10.0))
    assert "verdict=worth-poc" in metric


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
