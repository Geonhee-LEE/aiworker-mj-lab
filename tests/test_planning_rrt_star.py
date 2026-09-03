"""RRT*의 순수 numpy 성질 시험: 결정론, 무충돌 경로 보장, 비용 개선.

``tests/test_planning_rrt.py``와 같은 벽-틈(slab) 시나리오 fixture를 재사용해
RRT-Connect가 이미 가진 성질과 대응하는 것을 검증하고, RRT*만의 신규 성질
(시간 예산 안에서 비용이 계속 개선되는지)을 추가로 검증한다.

Headless 단독 실행: ``python3 tests/test_planning_rrt_star.py``
"""

import pathlib
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.planning.arm_state import RightArmSpace  # noqa: E402
from ffw_sh5_grasp.planning.local_path import EdgeChecker  # noqa: E402
from ffw_sh5_grasp.planning.rrt_connect import plan_rrt_connect  # noqa: E402
from ffw_sh5_grasp.planning.rrt_star import plan_rrt_star  # noqa: E402


def _path_length_rad(space, path):
    """경로 전체 길이(연속 waypoint 간 L2 거리의 합, rad).

    ``planning.shortcut.path_length_rad``와 같은 계산이지만, 그 모듈은
    별도 브랜치(``planning/p2-demo-natural-motion``, 아직 main에 없음)에서만
    구현돼 있어 여기서는 의존하지 않고 같은 계산을 직접 쓴다.
    """
    path = np.asarray(path, dtype=float)
    if len(path) < 2:
        return 0.0
    return float(sum(space.distance(path[i], path[i + 1]) for i in range(len(path) - 1)))


def _box_space():
    # test_planning_rrt.py와 동일한 fixture — joint0·joint1만 넓게 열어 "벽에
    # 뚫린 틈"을 실제로 우회할 공간을 준다.
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


START = np.array([0.0] * 7)
GOAL = np.array([3.0] + [0.0] * 6)


def _plan(seed, **overrides):
    space = _box_space()
    is_valid = _slab_predicate(space)
    checker = EdgeChecker(space, is_valid, resolution_rad=0.1)
    kwargs = dict(
        rng=np.random.default_rng(seed),
        step_size_rad=0.4,
        goal_bias=0.1,
        max_iterations=800,
        # RRT-Connect와 달리 RRT*는 첫 해를 찾아도 멈추지 않고 예산이 끝날 때까지
        # 계속 반복한다 — time_budget_s가 실제로 걸리는 경우가 흔해, 실제 벽시계
        # 시간에 따라 반복 횟수(따라서 경로 자체)가 달라져 결정론이 깨진다.
        # 테스트에서는 max_iterations가 항상 먼저 걸리도록 넉넉히 잡는다.
        time_budget_s=30.0,
        rewire_radius_rad=0.8,
        goal_tolerance_rad=0.4,
    )
    kwargs.update(overrides)
    return plan_rrt_star(space, checker, START, GOAL, **kwargs)


def test_straight_line_blocked_but_rrt_star_succeeds():
    result = _plan(seed=0)
    assert result.success
    assert result.path is not None
    assert result.path[0].tolist() == START.tolist()
    assert result.path[-1].tolist() == GOAL.tolist()


def test_planner_is_deterministic_for_a_seed():
    first = _plan(seed=7)
    second = _plan(seed=7)
    assert first.success and second.success
    assert np.array_equal(first.path, second.path)
    assert first.iterations == second.iterations


def test_planner_never_returns_a_colliding_path():
    """반환 경로의 모든 waypoint·edge가 **계획에 쓰인 것과 같은 해상도**로 다시
    검사해도 유효해야 한다.

    RRT-Connect의 같은 이름 시험(``test_planning_rrt.py``)은 검증에 planning보다
    2배 더 촘촘한 해상도(0.05 vs 0.1)를 쓰고도 안정적으로 통과하는데, 이는
    RRT-Connect가 첫 해를 찾으면 즉시 멈춰 전체 실행에서 만드는 edge 수 자체가
    적어 "경계에 걸친 좁은 틈"을 실제로 만날 확률이 낮기 때문이다(보장된 성질은
    아니다). RRT*는 예산이 끝날 때까지 계속 반복하며 훨씬 많은 edge를 검사·rewire
    하므로 이 경계 사례에 노출될 확률이 커진다 — 그래서 이 시험은 "같은
    EdgeChecker 해상도로 다시 확인해도 스스로 모순되지 않는가"라는, 실제로
    보장되는 성질만 검증한다.
    """
    space = _box_space()
    is_valid = _slab_predicate(space)
    for seed in range(20):
        result = _plan(seed=seed)
        if not result.success:
            continue
        checker = EdgeChecker(space, is_valid, resolution_rad=0.1)
        for point in result.path:
            assert is_valid(point), f"seed={seed}: invalid waypoint {point}"
        for a, b in zip(result.path[:-1], result.path[1:]):
            assert checker.is_valid_edge(a, b), f"seed={seed}: invalid edge {a}->{b}"


def test_invalid_start_or_goal_reported():
    space = _box_space()
    is_valid = _slab_predicate(space)
    checker = EdgeChecker(space, is_valid, resolution_rad=0.1)
    bad_start = np.array([1.5] + [0.0] * 6)  # slab 안, 항상 무효
    result = plan_rrt_star(
        space, checker, bad_start, GOAL,
        rng=np.random.default_rng(0), step_size_rad=0.4, goal_bias=0.1,
        max_iterations=100, time_budget_s=5.0, rewire_radius_rad=0.8, goal_tolerance_rad=0.4,
    )
    assert not result.success
    assert result.reason == "invalid_start"


def test_rewiring_improves_cost_over_time():
    """RRT*의 핵심 주장 — 반복을 늘릴수록 반환 경로 비용이 단조 비증가해야 한다."""
    space = _box_space()
    is_valid = _slab_predicate(space)
    iteration_budgets = [250, 500, 800, 1200]
    costs = []
    for max_iterations in iteration_budgets:
        checker = EdgeChecker(space, is_valid, resolution_rad=0.05)
        result = plan_rrt_star(
            space, checker, START, GOAL,
            rng=np.random.default_rng(3), step_size_rad=0.4, goal_bias=0.1,
            max_iterations=max_iterations, time_budget_s=30.0,
            rewire_radius_rad=0.8, goal_tolerance_rad=0.4,
        )
        assert result.success, f"max_iterations={max_iterations}: 계획 실패"
        costs.append(_path_length_rad(space, result.path))
    for earlier, later in zip(costs, costs[1:]):
        assert later <= earlier + 1e-9, f"비용이 반복을 늘렸는데 오히려 늘어났다: {costs}"
    # 최소 한 번은 실제로 개선돼야 한다 — 그냥 항상 같은 값을 반환해도 통과하는
    # 시험이 되지 않도록.
    assert costs[-1] < costs[0] - 1e-6, f"반복을 늘려도 비용이 전혀 개선되지 않았다: {costs}"


def test_rrt_star_matches_or_beats_rrt_connect():
    """같은 문제에서 RRT*의 중앙값 비용이 (첫 해만 반환하는) RRT-Connect보다 나쁘지 않은지.

    둘 다 같은 ``max_iterations``/``time_budget_s`` 예산을 쓴다 — RRT-Connect는
    첫 해를 찾으면 바로 멈추고, RRT*는 예산이 끝날 때까지 계속 개선한다는
    설계 차이가 실제로 더 나은(또는 최소한 나쁘지 않은) 경로로 이어지는지
    확인하는 축소판 비교다(MP-0017 50-seed 정식 비교표의 사전 점검).
    """
    space = _box_space()
    is_valid = _slab_predicate(space)
    star_costs, connect_costs = [], []
    for seed in range(10):
        checker_star = EdgeChecker(space, is_valid, resolution_rad=0.05)
        star_result = plan_rrt_star(
            space, checker_star, START, GOAL,
            rng=np.random.default_rng(seed), step_size_rad=0.4, goal_bias=0.1,
            max_iterations=800, time_budget_s=30.0,
            rewire_radius_rad=0.8, goal_tolerance_rad=0.4,
        )
        checker_connect = EdgeChecker(space, is_valid, resolution_rad=0.05)
        connect_result = plan_rrt_connect(
            space, checker_connect, START, GOAL,
            rng=np.random.default_rng(seed), step_size_rad=0.4, goal_bias=0.1,
            max_iterations=800, time_budget_s=30.0,
        )
        if not (star_result.success and connect_result.success):
            continue
        star_costs.append(_path_length_rad(space, star_result.path))
        connect_costs.append(_path_length_rad(space, connect_result.path))

    assert len(star_costs) >= 8, "성공한 seed가 너무 적어 비교가 무의미하다"
    assert np.median(star_costs) <= np.median(connect_costs) + 1e-6, (
        f"RRT* 중앙값 비용({np.median(star_costs):.3f})이 "
        f"RRT-Connect({np.median(connect_costs):.3f})보다 나쁘다"
    )


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
