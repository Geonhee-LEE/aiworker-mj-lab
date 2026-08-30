"""오른팔 관절공간 추상화(``RightArmSpace``)와 선분 검사(``EdgeChecker``)의 순수
numpy 단위 시험. MuJoCo 모델이 필요 없어 빠르게 돈다.

Headless 단독 실행: ``python3 tests/test_planning_core.py``
"""

import pathlib
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.planning.arm_state import (  # noqa: E402
    RIGHT_ARM_JOINTS,
    RightArmSpace,
)
from ffw_sh5_grasp.planning.local_path import EdgeChecker  # noqa: E402


def _box_space(n=7, lo=-3.0, hi=3.0):
    return RightArmSpace.from_limits(np.full(n, lo), np.full(n, hi))


def test_from_limits_shapes_and_names():
    space = _box_space()
    assert space.n == 7
    assert space.joint_names == tuple(f"joint{i}" for i in range(7))
    assert space.lower.shape == (7,)
    assert space.contains(np.zeros(7))
    assert not space.contains(np.full(7, 10.0))


def test_clip_and_write():
    space = _box_space()
    clipped = space.clip(np.full(7, 100.0))
    assert np.allclose(clipped, space.upper)
    qpos = np.zeros(10)
    qpos[space.qpos_adrs] = 0
    written = space.write(qpos.copy(), np.arange(7, dtype=float))
    assert np.allclose(written[space.qpos_adrs], np.arange(7, dtype=float))


def test_sample_always_in_range():
    space = _box_space()
    rng = np.random.default_rng(0)
    for _ in range(200):
        q = space.sample(rng)
        assert space.contains(q)


def test_steer_stops_at_max_step():
    space = _box_space()
    source = np.zeros(7)
    target = np.ones(7) * 10.0
    stepped = space.steer(source, target, max_step_rad=1.0)
    assert np.isclose(space.distance(source, stepped), 1.0, atol=1e-9)
    # 목표가 이미 step 이내면 목표 자체를 반환한다.
    close_target = np.ones(7) * 0.1
    stepped2 = space.steer(source, close_target, max_step_rad=1.0)
    assert np.allclose(stepped2, close_target)


def test_interpolate_endpoints():
    space = _box_space()
    a, b = np.zeros(7), np.ones(7)
    assert np.allclose(space.interpolate(a, b, 0.0), a)
    assert np.allclose(space.interpolate(a, b, 1.0), b)
    assert np.allclose(space.interpolate(a, b, 0.5), 0.5 * np.ones(7))


def test_right_arm_joint_names_are_stable():
    assert RIGHT_ARM_JOINTS == tuple(f"arm_r_joint{i}" for i in range(1, 8))


def _slab_predicate(space):
    """joint0이 [1.0, 2.0] 구간에 있으면 무효로 판정하는 인위적 장애물."""

    def is_valid(q):
        if not space.contains(q):
            return False
        return not (1.0 <= q[0] <= 2.0)

    return is_valid


def test_edge_checker_rejects_blocked_straight_line():
    space = _box_space()
    is_valid = _slab_predicate(space)
    checker = EdgeChecker(space, is_valid, resolution_rad=0.05)
    start = np.array([0.0] * 7)
    goal = np.array([3.0] + [0.0] * 6)
    assert is_valid(start) and is_valid(goal)
    assert not checker.is_valid_edge(start, goal)


def test_edge_checker_accepts_clear_straight_line():
    space = _box_space()
    is_valid = _slab_predicate(space)
    checker = EdgeChecker(space, is_valid, resolution_rad=0.05)
    start = np.array([-1.0] * 7)
    goal = np.array([0.5] * 7)
    assert checker.is_valid_edge(start, goal)


def test_edge_checker_last_valid_stops_before_slab():
    space = _box_space()
    is_valid = _slab_predicate(space)
    checker = EdgeChecker(space, is_valid, resolution_rad=0.01)
    start = np.array([0.0] * 7)
    goal = np.array([3.0] + [0.0] * 6)
    point, fraction = checker.last_valid(start, goal)
    assert 0.0 < fraction < 1.0
    assert point[0] < 1.0 + 1e-6


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
