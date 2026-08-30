"""RRT-Connect의 순수 numpy 성질 시험: 결정론, 무충돌 경로 보장.

MuJoCo가 필요 없어 빠르게 돈다. 실제 로봇 장면에서의 질의는
``tests/test_planning_rrt_scene.py``에 있다.

Headless 단독 실행: ``python3 tests/test_planning_rrt.py``
"""

import pathlib
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.planning.arm_state import RightArmSpace  # noqa: E402
from ffw_sh5_grasp.planning.local_path import EdgeChecker  # noqa: E402
from ffw_sh5_grasp.planning.rrt_connect import plan_rrt_connect  # noqa: E402


def _box_space():
    # joint0·joint1은 넓게 열어 "벽에 뚫린 틈"을 실제로 우회할 공간을 주고,
    # 나머지 5개는 좁혀서 무관한 표본 낭비 없이 우회 로직만 빠르게 검증한다.
    lower = np.array([-3.0, -3.0] + [-0.1] * 5)
    upper = np.array([3.0, 3.0] + [0.1] * 5)
    return RightArmSpace.from_limits(lower, upper)


def _slab_predicate(space):
    """joint0 in [1.0, 2.0]인 "벽"이 |joint1| < 1.5 구간만 막는다 — start와
    goal을 잇는 직선(joint1=0)은 막히지만, joint1으로 틈을 돌아가면 통과할 수
    있다. 완전히 막힌 장애물이 아니라 실제로 풀리는 우회 경로 시험이다."""

    def is_valid(q):
        if not space.contains(q):
            return False
        in_wall_band = 1.0 <= q[0] <= 2.0
        blocks_here = abs(q[1]) < 1.5
        return not (in_wall_band and blocks_here)

    return is_valid


def _plan(seed, **overrides):
    space = _box_space()
    is_valid = _slab_predicate(space)
    checker = EdgeChecker(space, is_valid, resolution_rad=0.1)
    start = np.array([0.0] * 7)
    goal = np.array([3.0] + [0.0] * 6)
    kwargs = dict(
        rng=np.random.default_rng(seed),
        step_size_rad=0.4,
        goal_bias=0.1,
        max_iterations=3000,
        time_budget_s=5.0,
    )
    kwargs.update(overrides)
    return plan_rrt_connect(space, checker, start, goal, **kwargs)


def test_straight_line_blocked_but_rrt_connect_succeeds():
    result = _plan(seed=0)
    assert result.success
    assert result.path is not None
    assert result.path[0].tolist() == [0.0] * 7
    assert result.path[-1].tolist() == [3.0] + [0.0] * 6


def test_planner_is_deterministic_for_a_seed():
    first = _plan(seed=7)
    second = _plan(seed=7)
    assert first.success and second.success
    assert np.array_equal(first.path, second.path)
    assert first.iterations == second.iterations


def test_planner_never_returns_a_colliding_path():
    space = _box_space()
    is_valid = _slab_predicate(space)
    for seed in range(20):
        checker = EdgeChecker(space, is_valid, resolution_rad=0.05)
        result = _plan(seed=seed, max_iterations=2000)
        if not result.success:
            continue
        for point in result.path:
            assert is_valid(point), f"seed={seed}: invalid waypoint {point}"
        for a, b in zip(result.path[:-1], result.path[1:]):
            assert checker.is_valid_edge(a, b), f"seed={seed}: invalid edge {a}->{b}"


def test_invalid_start_or_goal_reported():
    space = _box_space()
    is_valid = _slab_predicate(space)
    checker = EdgeChecker(space, is_valid, resolution_rad=0.1)
    bad_start = np.array([1.5] + [0.0] * 6)  # slab 안, 항상 무효
    good_goal = np.array([3.0] + [0.0] * 6)
    result = plan_rrt_connect(
        space, checker, bad_start, good_goal,
        rng=np.random.default_rng(0), step_size_rad=0.4, goal_bias=0.1,
        max_iterations=100, time_budget_s=5.0,
    )
    assert not result.success
    assert result.reason == "invalid_start"


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
