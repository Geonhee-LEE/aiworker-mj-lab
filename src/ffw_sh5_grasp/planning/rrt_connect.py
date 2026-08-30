"""RRT-Connect: 두 트리를 오른팔 관절 공간에서 확장·연결하는 샘플링 플래너.

``ArmCollisionChecker.is_valid``와 ``EdgeChecker``를 그대로 주입받아 쓴다 —
충돌 판정 로직은 이 모듈에 없다. 결정론을 위해 난수는 항상 호출자가 넘긴
``np.random.Generator``만 쓴다.
"""

import time
from dataclasses import dataclass

import numpy as np

_REACHED = "reached"
_ADVANCED = "advanced"
_TRAPPED = "trapped"


@dataclass(frozen=True)
class PlannerResult:
    """RRT-Connect 한 번의 질의 결과."""

    success: bool
    path: object  # (K, 7) np.ndarray 또는 None
    reason: str
    iterations: int
    node_counts: tuple
    state_checks: int
    elapsed_s: float


class _Tree:
    """샘플링 트리: 노드 목록과 부모 인덱스만 보관하는 최소 자료구조."""

    def __init__(self, root):
        self.nodes = [np.asarray(root, dtype=float)]
        self.parents = [-1]

    def __len__(self):
        return len(self.nodes)

    def nearest(self, space, point):
        """선형 탐색 최근접 노드 인덱스. 7-D·수천 노드 규모에선 KD-tree보다
        구현이 단순하고 의존성이 없으면서도 충분히 빠르다."""
        best_index, best_distance = 0, float("inf")
        for index, node in enumerate(self.nodes):
            distance = space.distance(node, point)
            if distance < best_distance:
                best_index, best_distance = index, distance
        return best_index

    def add(self, point, parent_index):
        self.nodes.append(np.asarray(point, dtype=float))
        self.parents.append(parent_index)
        return len(self.nodes) - 1

    def path_to_root(self, index):
        path = []
        while index != -1:
            path.append(self.nodes[index])
            index = self.parents[index]
        path.reverse()
        return path


def straight_line_path(space, start, goal):
    """단순 두 점 경로(성공 시 다른 후처리 단계의 기준선으로 쓴다)."""
    return np.stack([np.asarray(start, dtype=float), np.asarray(goal, dtype=float)])


def _extend(tree, space, edge_checker, target, step_size_rad):
    """트리를 ``target`` 방향으로 최대 ``step_size_rad``만큼 한 번 확장한다."""
    nearest_index = tree.nearest(space, target)
    nearest = tree.nodes[nearest_index]
    new_point = space.steer(nearest, target, step_size_rad)
    if not edge_checker.is_valid_edge(nearest, new_point, check_endpoints=False):
        return _TRAPPED, None
    if not edge_checker.is_valid(new_point):
        return _TRAPPED, None
    new_index = tree.add(new_point, nearest_index)
    status = _REACHED if np.array_equal(new_point, np.asarray(target, dtype=float)) else _ADVANCED
    return status, new_index


def _connect(tree, space, edge_checker, target, step_size_rad):
    """target에 닿거나 막힐 때까지 같은 target으로 반복 확장한다
    (RRT-Connect의 CONNECT: ``repeat EXTEND(tree, target) until status != ADVANCED``)."""
    while True:
        status, index = _extend(tree, space, edge_checker, target, step_size_rad)
        if status != _ADVANCED:
            return status, index


def plan_rrt_connect(
    space,
    edge_checker,
    start,
    goal,
    *,
    rng,
    step_size_rad,
    goal_bias,
    max_iterations,
    time_budget_s,
):
    """오른팔 관절 공간에서 ``start``-``goal``을 잇는 충돌 없는 경로를 찾는다.

    두 트리(시작·목표)를 번갈아 확장하고, 한쪽이 다른 쪽 최신 노드에 닿으면
    (CONNECT) 성공이다. ``time_budget_s``가 실제 종료 조건이고
    ``max_iterations``는 폭주 방지용 상한이다.
    """
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    start_time = time.perf_counter()

    if not edge_checker.is_valid(start):
        return PlannerResult(False, None, "invalid_start", 0, (1, 1), edge_checker.space.n, 0.0)
    if not edge_checker.is_valid(goal):
        return PlannerResult(False, None, "no_valid_goal", 0, (1, 1), edge_checker.space.n, 0.0)

    tree_a = _Tree(start)
    tree_b = _Tree(goal)
    swapped = False

    for iteration in range(1, max_iterations + 1):
        if time.perf_counter() - start_time > time_budget_s:
            return PlannerResult(
                False, None, "time_budget", iteration, (len(tree_a), len(tree_b)),
                _checks(edge_checker), time.perf_counter() - start_time,
            )

        sample = goal if rng.random() < goal_bias else space.sample(rng)
        status_a, index_a = _extend(tree_a, space, edge_checker, sample, step_size_rad)
        if status_a != _TRAPPED:
            status_b, _ = _connect(tree_b, space, edge_checker, tree_a.nodes[index_a], step_size_rad)
            if status_b == _REACHED:
                path_a = tree_a.path_to_root(index_a)
                path_b = tree_b.path_to_root(len(tree_b) - 1)
                path = path_a + path_b[::-1]
                if swapped:
                    path = path[::-1]
                counts = (len(tree_a), len(tree_b)) if not swapped else (len(tree_b), len(tree_a))
                return PlannerResult(
                    True, np.stack(path), "goal_reached", iteration, counts,
                    _checks(edge_checker), time.perf_counter() - start_time,
                )
        tree_a, tree_b = tree_b, tree_a
        swapped = not swapped

    counts = (len(tree_a), len(tree_b)) if not swapped else (len(tree_b), len(tree_a))
    return PlannerResult(
        False, None, "iteration_limit", max_iterations, counts,
        _checks(edge_checker), time.perf_counter() - start_time,
    )


def _checks(edge_checker):
    is_valid = getattr(edge_checker, "is_valid", None)
    checker = getattr(is_valid, "__self__", None)
    return getattr(checker, "state_checks", 0)


__all__ = ["PlannerResult", "plan_rrt_connect", "straight_line_path"]
