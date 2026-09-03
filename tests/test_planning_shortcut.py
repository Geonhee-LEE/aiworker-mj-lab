"""shortcut 평활화의 순수 numpy 성질 시험: 길이 비증가, 무충돌 보존, 끝점 보존.

MuJoCo가 필요 없어 빠르게 돈다.

Headless 단독 실행: ``python3 tests/test_planning_shortcut.py``
"""

import pathlib
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.planning.arm_state import RightArmSpace  # noqa: E402
from ffw_sh5_grasp.planning.local_path import EdgeChecker  # noqa: E402
from ffw_sh5_grasp.planning.rrt_connect import plan_rrt_connect  # noqa: E402
from ffw_sh5_grasp.planning.shortcut import path_length_rad, shortcut_path  # noqa: E402


def _box_space():
    lower = np.array([-3.0, -3.0] + [-0.1] * 5)
    upper = np.array([3.0, 3.0] + [0.1] * 5)
    return RightArmSpace.from_limits(lower, upper)


def _slab_predicate(space):
    """joint0 in [1.0, 2.0]인 "벽"이 |joint1| < 1.5 구간만 막는다
    (``test_planning_rrt.py``와 동일한 시나리오)."""

    def is_valid(q):
        if not space.contains(q):
            return False
        in_wall_band = 1.0 <= q[0] <= 2.0
        blocks_here = abs(q[1]) < 1.5
        return not (in_wall_band and blocks_here)

    return is_valid


def _always_valid(space):
    def is_valid(q):
        return space.contains(q)

    return is_valid


def test_path_length_rad_sums_segment_distances():
    space = _box_space()
    path = np.array([[0.0] * 7, [1.0] + [0.0] * 6, [1.0, 1.0] + [0.0] * 5])
    assert path_length_rad(space, path) == 2.0


def test_path_length_rad_handles_degenerate_paths():
    space = _box_space()
    assert path_length_rad(space, np.empty((0, 7))) == 0.0
    assert path_length_rad(space, np.array([[0.0] * 7])) == 0.0


def test_shortcut_reduces_zigzag_to_near_straight_line_in_free_space():
    space = _box_space()
    checker = EdgeChecker(space, _always_valid(space), resolution_rad=0.05)
    start = np.array([0.0] * 7)
    goal = np.array([2.0, 0.0] + [0.0] * 5)
    # 중간에 불필요한 지그재그를 끼워 넣은 경로.
    zigzag = np.array(
        [
            start,
            [0.3, 0.3] + [0.0] * 5,
            [0.6, -0.3] + [0.0] * 5,
            [1.0, 0.3] + [0.0] * 5,
            [1.5, -0.3] + [0.0] * 5,
            goal,
        ]
    )
    original_length = path_length_rad(space, zigzag)

    result = shortcut_path(space, checker, zigzag, rng=np.random.default_rng(0), iterations=200)

    assert np.array_equal(result[0], start)
    assert np.array_equal(result[-1], goal)
    straight_line = space.distance(start, goal)
    shortened_length = path_length_rad(space, result)
    assert shortened_length <= original_length
    # 자유 공간이라 전부 잘려나가 사실상 직선이 되어야 한다.
    assert abs(shortened_length - straight_line) < 1e-9


def test_shortcut_never_increases_length_and_stays_collision_free():
    space = _box_space()
    is_valid = _slab_predicate(space)
    start = np.array([0.0] * 7)
    goal = np.array([3.0] + [0.0] * 6)
    for seed in range(10):
        checker = EdgeChecker(space, is_valid, resolution_rad=0.05)
        result = plan_rrt_connect(
            space, checker, start, goal,
            rng=np.random.default_rng(seed), step_size_rad=0.4, goal_bias=0.1,
            max_iterations=3000, time_budget_s=5.0,
        )
        if not result.success:
            continue
        original_length = path_length_rad(space, result.path)

        shortcut_checker = EdgeChecker(space, is_valid, resolution_rad=0.05)
        shortened = shortcut_path(
            space, shortcut_checker, result.path,
            rng=np.random.default_rng(seed), iterations=200,
        )

        assert np.array_equal(shortened[0], start), f"seed={seed}: start changed"
        assert np.array_equal(shortened[-1], goal), f"seed={seed}: goal changed"
        assert path_length_rad(space, shortened) <= original_length + 1e-9, (
            f"seed={seed}: shortcut lengthened the path"
        )
        for point in shortened:
            assert is_valid(point), f"seed={seed}: invalid waypoint {point}"
        for a, b in zip(shortened[:-1], shortened[1:]):
            assert checker.is_valid_edge(a, b), f"seed={seed}: invalid edge {a}->{b}"


def test_shortcut_is_deterministic_for_a_seed():
    space = _box_space()
    checker = EdgeChecker(space, _always_valid(space), resolution_rad=0.05)
    path = np.array(
        [[0.0] * 7, [0.5, 0.5] + [0.0] * 5, [1.0, -0.5] + [0.0] * 5, [2.0, 0.0] + [0.0] * 5]
    )
    first = shortcut_path(space, checker, path, rng=np.random.default_rng(3), iterations=50)
    second = shortcut_path(space, checker, path, rng=np.random.default_rng(3), iterations=50)
    assert np.array_equal(first, second)


def test_shortcut_leaves_short_paths_untouched():
    space = _box_space()
    checker = EdgeChecker(space, _always_valid(space), resolution_rad=0.05)
    two_point = np.array([[0.0] * 7, [1.0] + [0.0] * 6])
    result = shortcut_path(space, checker, two_point, rng=np.random.default_rng(0), iterations=50)
    assert np.array_equal(result, two_point)


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
