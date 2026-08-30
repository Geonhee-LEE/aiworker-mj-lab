"""RRT-Connect 경로 후처리: 무작위 shortcut 평활화.

RRT-Connect의 경로는 트리 확장 과정에서 생긴 지그재그 waypoint를 그대로
담고 있어, 두 점을 직접 이어도 무충돌인 구간이 많이 남는다. 무작위로 두
waypoint를 골라 그 사이를 직선으로 잘라낼 수 있으면 잘라내는 과정을
반복해 경로 길이를 줄인다. 결정론을 위해 난수는 항상 호출자가 넘긴
``np.random.Generator``만 쓴다.
"""

import numpy as np


def path_length_rad(space, path):
    """경로 전체 길이(연속 waypoint 간 L2 거리의 합, rad)."""
    path = np.asarray(path, dtype=float)
    if len(path) < 2:
        return 0.0
    return float(
        sum(space.distance(path[i], path[i + 1]) for i in range(len(path) - 1))
    )


def shortcut_path(space, edge_checker, path, *, rng, iterations):
    """``path``에서 무작위 두 waypoint 사이를 직선으로 잘라낼 수 있으면 잘라낸다.

    ``iterations``번 시도한다. 매 시도마다 현재 경로에서 서로 다른 두
    인덱스를 뽑아, 그 사이 직선 구간이 전부 유효하면(``edge_checker``) 그
    사이 waypoint를 모두 버린다. 원래 waypoint는 planner가 이미 유효성을
    검증했으므로 끝점 재검사는 생략한다(``check_endpoints=False``).
    시작·목표점은 항상 보존되고, 결과 길이는 원래 길이 이하다.
    """
    waypoints = [np.asarray(point, dtype=float) for point in path]
    if len(waypoints) < 3:
        return np.stack(waypoints) if waypoints else np.empty((0, space.n))

    for _ in range(iterations):
        if len(waypoints) < 3:
            break
        low, high = sorted(int(index) for index in rng.integers(0, len(waypoints), size=2))
        if high - low < 2:
            continue
        if edge_checker.is_valid_edge(waypoints[low], waypoints[high], check_endpoints=False):
            waypoints = waypoints[: low + 1] + waypoints[high:]

    return np.stack(waypoints)


__all__ = ["path_length_rad", "shortcut_path"]
