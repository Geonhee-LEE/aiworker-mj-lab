"""RRT*: 단일 트리를 확장하며 비용(경로 길이)이 낮은 해로 계속 갱신하는 플래너.

``rrt_connect.plan_rrt_connect``는 첫 해를 찾으면 즉시 반환하므로 경로 품질이
운에 좌우된다. RRT*는 시간 예산이 끝날 때까지 계속 탐색하며, 새 노드를 삽입할
때마다 (1) 반경 안 근접 노드 중 비용이 가장 낮은 쪽을 부모로 선택하고
(2) 반경 안 다른 노드들도 새 노드를 거치는 편이 더 싸면 부모를 새 노드로
바꾸는(rewire) 방식으로 점진적으로 개선한다.

두 트리 bidirectional connect(``rrt_connect``)와 rewiring을 결합하면 복잡도가
크게 늘어나므로, 이 모듈은 의도적으로 **단일 트리**로 설계했다. 점근적
최적성을 위한 이론적 shrinking radius(``gamma * (log n / n)^(1/d)``) 대신
**고정 반경**(``rewire_radius_rad``)을 쓴다 — 7-DOF 관절 공간에서 이론식은
반경이 매우 빠르게 줄어들어 rewiring이 사실상 거의 일어나지 않는다. 이는
``trajectory.py``가 코너 blending을 범위 밖으로 명시 보류한 것과 같은 종류의
실용적 단순화다 — 점근적 최적성을 엄밀히 보장하지는 않지만, 시간 예산 안에서
비용을 계속 개선한다는 RRT*의 핵심 이득은 그대로 유지한다.

충돌 판정 로직은 이 모듈에 없다 — ``ArmCollisionChecker``/``EdgeChecker``를
그대로 주입받아 쓴다. 결정론을 위해 난수는 항상 호출자가 넘긴
``np.random.Generator``만 쓴다.
"""

import time

import numpy as np

from .rrt_connect import PlannerResult, TreeSnapshot


class _StarTree:
    """비용(root부터 누적 거리)과 부모/자식 관계를 함께 보관하는 트리.

    ``rrt_connect._Tree``와 달리 rewiring을 지원해야 해서 자식 목록
    (``children``)도 유지한다 — 한 노드의 부모가 바뀌면 그 아래 서브트리
    전체의 비용을 다시 계산해야 하는데, 자식 목록이 있어야 그 서브트리를
    빠르게 순회할 수 있다.
    """

    def __init__(self, root):
        self.nodes = [np.asarray(root, dtype=float)]
        self.parents = [-1]
        self.costs = [0.0]
        self.children = [[]]

    def __len__(self):
        return len(self.nodes)

    def nearest(self, space, point):
        """선형 탐색 최근접 노드 인덱스(``rrt_connect._Tree.nearest``와 동일한 이유로 충분히 빠르다)."""
        best_index, best_distance = 0, float("inf")
        for index, node in enumerate(self.nodes):
            distance = space.distance(node, point)
            if distance < best_distance:
                best_index, best_distance = index, distance
        return best_index

    def near(self, space, point, radius):
        """``radius`` 안의 모든 노드 인덱스(선형 탐색)."""
        return [
            index for index, node in enumerate(self.nodes)
            if space.distance(node, point) <= radius
        ]

    def add(self, point, parent_index, cost):
        index = len(self.nodes)
        self.nodes.append(np.asarray(point, dtype=float))
        self.parents.append(parent_index)
        self.costs.append(cost)
        self.children.append([])
        self.children[parent_index].append(index)
        return index

    def rewire_parent(self, index, new_parent_index, new_cost):
        """``index`` 노드의 부모를 바꾸고, 비용 변화량을 서브트리 전체에 전파한다."""
        delta = new_cost - self.costs[index]
        old_parent = self.parents[index]
        self.children[old_parent].remove(index)
        self.parents[index] = new_parent_index
        self.costs[index] = new_cost
        self.children[new_parent_index].append(index)
        if delta != 0.0:
            stack = list(self.children[index])
            while stack:
                i = stack.pop()
                self.costs[i] += delta
                stack.extend(self.children[i])

    def path_to_root(self, index):
        path = []
        while index != -1:
            path.append(self.nodes[index])
            index = self.parents[index]
        path.reverse()
        return path

    def snapshot(self):
        return TreeSnapshot(np.stack(self.nodes), np.asarray(self.parents, dtype=int))


def _sample(space, rng, goal, goal_bias, start, best_cost, use_informed, max_rejections=20):
    """``goal_bias``로 목표를, 아니면 균등 표본을 뽑는다.

    ``use_informed``면(첫 해를 찾은 뒤) ``dist(start, x) + dist(x, goal) <
    best_cost``인 표본만 받아들이는 거부 표집을 쓴다 — 이미 찾은 해보다 더
    나은 해가 지날 수 있는 영역으로만 탐색을 좁힌다. 7-D 타원체의 회전행렬을
    직접 구성하는 정식 Informed RRT* 샘플러보다 훨씬 단순하면서 같은 효과를
    낸다. ``max_rejections``번 안에 못 뽑으면 그냥 균등 표본으로 물러난다.
    """
    if rng.random() < goal_bias:
        return goal
    if not use_informed:
        return space.sample(rng)
    for _ in range(max_rejections):
        candidate = space.sample(rng)
        if space.distance(start, candidate) + space.distance(candidate, goal) < best_cost:
            return candidate
    return space.sample(rng)


def _checks(edge_checker):
    is_valid = getattr(edge_checker, "is_valid", None)
    checker = getattr(is_valid, "__self__", None)
    return getattr(checker, "state_checks", 0)


def _result(success, path, reason, iterations, tree, goal, checks, elapsed_s):
    goal_tree = TreeSnapshot(np.asarray([goal], dtype=float), np.asarray([-1], dtype=int))
    return PlannerResult(
        success, path, reason, iterations, (len(tree), 1), checks, elapsed_s,
        tree.snapshot(), goal_tree,
    )


def plan_rrt_star(
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
    rewire_radius_rad,
    goal_tolerance_rad,
    informed_sampling=True,
):
    """단일 트리 RRT*로 ``start``-``goal``을 잇는 경로를 찾고 시간 예산 안에서 비용을 계속 개선한다.

    ``plan_rrt_connect``와 달리 첫 해를 찾아도 멈추지 않고 ``time_budget_s``
    (또는 ``max_iterations``)가 끝날 때까지 계속 탐색하며, 그 시점까지 찾은
    최선의(비용이 가장 낮은) 해를 반환한다. 반환값은 ``rrt_connect.PlannerResult``를
    그대로 재사용한다 — 단일 트리이므로 ``goal_tree``는 노드 1개(``goal`` 자체)
    짜리 스냅샷으로 채워, 기존 트리 시각화 코드가 그대로 동작하게 한다.
    """
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    start_time = time.perf_counter()

    if not edge_checker.is_valid(start):
        return _result(False, None, "invalid_start", 0, _StarTree(start), goal,
                        edge_checker.space.n, 0.0)
    if not edge_checker.is_valid(goal):
        return _result(False, None, "no_valid_goal", 0, _StarTree(start), goal,
                        edge_checker.space.n, 0.0)

    tree = _StarTree(start)
    best_goal_parent = None
    best_cost = float("inf")
    iteration = 0
    reason = "iteration_limit"

    for iteration in range(1, max_iterations + 1):
        if time.perf_counter() - start_time > time_budget_s:
            reason = "time_budget"
            break

        use_informed = informed_sampling and best_goal_parent is not None
        sample = _sample(space, rng, goal, goal_bias, start, best_cost, use_informed)

        nearest_index = tree.nearest(space, sample)
        nearest = tree.nodes[nearest_index]
        new_point = space.steer(nearest, sample, step_size_rad)
        if not edge_checker.is_valid_edge(nearest, new_point, check_endpoints=False):
            continue
        if not edge_checker.is_valid(new_point):
            continue

        near_indices = tree.near(space, new_point, rewire_radius_rad)

        best_parent = nearest_index
        best_parent_cost = tree.costs[nearest_index] + space.distance(nearest, new_point)
        for index in near_indices:
            if index == nearest_index:
                continue
            candidate_cost = tree.costs[index] + space.distance(tree.nodes[index], new_point)
            if candidate_cost < best_parent_cost and edge_checker.is_valid_edge(
                tree.nodes[index], new_point, check_endpoints=False
            ):
                best_parent, best_parent_cost = index, candidate_cost

        new_index = tree.add(new_point, best_parent, best_parent_cost)

        for index in near_indices:
            if index in (best_parent, new_index):
                continue
            candidate_cost = tree.costs[new_index] + space.distance(new_point, tree.nodes[index])
            if candidate_cost < tree.costs[index] and edge_checker.is_valid_edge(
                new_point, tree.nodes[index], check_endpoints=False
            ):
                tree.rewire_parent(index, new_index, candidate_cost)

        distance_to_goal = space.distance(new_point, goal)
        if distance_to_goal <= goal_tolerance_rad and edge_checker.is_valid_edge(
            new_point, goal, check_endpoints=False
        ):
            candidate_goal_cost = tree.costs[new_index] + distance_to_goal
            if candidate_goal_cost < best_cost:
                best_cost = candidate_goal_cost
                best_goal_parent = new_index

    elapsed = time.perf_counter() - start_time
    if best_goal_parent is None:
        return _result(False, None, reason, iteration, tree, goal, _checks(edge_checker), elapsed)

    path = tree.path_to_root(best_goal_parent) + [goal]
    return _result(True, np.stack(path), "goal_reached", iteration, tree, goal,
                    _checks(edge_checker), elapsed)


__all__ = ["plan_rrt_star"]
