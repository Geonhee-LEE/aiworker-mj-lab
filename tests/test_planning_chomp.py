"""CHOMP류 궤적 평활화의 순수 numpy 성질 시험: 끝점 보존, 비용 개선, 무충돌 보장.

``tests/test_planning_rrt.py``와 같은 벽-틈(slab) 시나리오 fixture를 재사용한다.

Headless 단독 실행: ``python3 tests/test_planning_chomp.py``
"""

import pathlib
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.planning.arm_state import RightArmSpace  # noqa: E402
from ffw_sh5_grasp.planning.chomp import (  # noqa: E402
    path_smoothness_cost,
    smooth_posture,
)
from ffw_sh5_grasp.planning.local_path import EdgeChecker  # noqa: E402
from ffw_sh5_grasp.planning.rrt_connect import plan_rrt_connect  # noqa: E402


def _box_space():
    lower = np.array([-3.0, -3.0] + [-0.1] * 5)
    upper = np.array([3.0, 3.0] + [0.1] * 5)
    return RightArmSpace.from_limits(lower, upper)


def _slab_predicate(space):
    """joint0 in [1.0, 2.0]인 "벽"이 |joint1| < 1.5 구간만 막는다."""

    def is_valid(q):
        if not space.contains(q):
            return False
        in_wall_band = 1.0 <= q[0] <= 2.0
        blocks_here = abs(q[1]) < 1.5
        return not (in_wall_band and blocks_here)

    return is_valid


def _zigzag_path():
    """일부러 지그재그로 만든 무충돌 경로(joint0만 지그재그, 나머지는 0 근처)."""
    joint0 = np.array([0.0, 0.3, -0.2, 0.5, 0.1, 0.6, 0.4, 0.8])
    path = np.zeros((len(joint0), 7))
    path[:, 0] = joint0
    return path


def test_endpoints_are_preserved():
    space = _box_space()
    is_valid = _slab_predicate(space)
    checker = EdgeChecker(space, is_valid, resolution_rad=0.05)
    path = _zigzag_path()
    optimized = smooth_posture(space, checker, path, trust_region_rad=0.3)
    assert np.array_equal(optimized[0], path[0])
    assert np.array_equal(optimized[-1], path[-1])


def test_smoothness_cost_does_not_increase():
    space = _box_space()
    is_valid = _slab_predicate(space)
    checker = EdgeChecker(space, is_valid, resolution_rad=0.05)
    path = _zigzag_path()
    optimized = smooth_posture(space, checker, path, trust_region_rad=0.3)
    assert path_smoothness_cost(optimized) <= path_smoothness_cost(path) + 1e-9
    # 실제로 개선돼야 한다 — 항상 원본을 그대로 반환해도 통과하는 시험이 되지 않도록.
    assert path_smoothness_cost(optimized) < path_smoothness_cost(path) - 1e-6


def test_trust_region_is_respected():
    space = _box_space()
    is_valid = _slab_predicate(space)
    checker = EdgeChecker(space, is_valid, resolution_rad=0.05)
    path = _zigzag_path()
    trust_region_rad = 0.15
    optimized = smooth_posture(space, checker, path, trust_region_rad=trust_region_rad)
    deviation = np.max(np.abs(optimized - path))
    assert deviation <= trust_region_rad + 1e-9


def test_never_returns_invalid_path():
    space = _box_space()
    is_valid = _slab_predicate(space)
    for seed in range(20):
        plan_checker = EdgeChecker(space, is_valid, resolution_rad=0.1)
        start = np.array([0.0] * 7)
        goal = np.array([3.0] + [0.0] * 6)
        result = plan_rrt_connect(
            space, plan_checker, start, goal,
            rng=np.random.default_rng(seed), step_size_rad=0.4, goal_bias=0.1,
            max_iterations=2000, time_budget_s=5.0,
        )
        if not result.success:
            continue
        verify_checker = EdgeChecker(space, is_valid, resolution_rad=0.1)
        optimized = smooth_posture(space, verify_checker, result.path, trust_region_rad=0.4)
        for point in optimized:
            assert is_valid(point), f"seed={seed}: invalid waypoint {point}"
        for a, b in zip(optimized[:-1], optimized[1:]):
            assert verify_checker.is_valid_edge(a, b), f"seed={seed}: invalid edge {a}->{b}"


def test_falls_back_to_original_when_optimization_would_collide():
    """직선 당김이 벽에 닿도록 구성한 입력 경로 — 결과가 원본과 정확히 같아야 한다."""
    space = _box_space()
    is_valid = _slab_predicate(space)
    checker = EdgeChecker(space, is_valid, resolution_rad=0.05)
    # 벽(1.0<=joint0<=2.0)을 |joint1|>=1.5인 좁은 틈으로 우회하는, 실제로 전부
    # 유효한 4-waypoint 경로다: joint1이 1.6으로 먼저 올라간 뒤(0->1의 구간은
    # joint0<1.0이라 벽 밖) joint0이 틈을 지나가고(1->2), 그 다음에야 joint1이
    # 다시 0으로 내려온다(2->3의 구간은 joint0>2.0이라 벽 밖). 2차 차분
    # 최소화(=국소 직선화)는 중간 두 점을 시작·끝을 잇는 직선(틈 밖, joint1=0
    # 근처) 쪽으로 당기려 하므로, 벽 안(joint1<1.5)으로 끌려 들어가 무효가
    # 된다 — 폴백이 반드시 발동해야 하는 시나리오.
    path = np.zeros((4, 7))
    path[0] = [0.0, 0.0, 0, 0, 0, 0, 0]
    path[1] = [0.9, 1.6, 0, 0, 0, 0, 0]
    path[2] = [2.1, 1.6, 0, 0, 0, 0, 0]
    path[3] = [3.0, 0.0, 0, 0, 0, 0, 0]
    assert all(is_valid(q) for q in path)
    for a, b in zip(path[:-1], path[1:]):
        assert checker.is_valid_edge(a, b)

    optimized = smooth_posture(space, checker, path, trust_region_rad=1.0, max_retries=3)
    assert np.array_equal(optimized, path), "폴백이 발동하지 않고 무효한 최적화 결과를 반환했다"


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
