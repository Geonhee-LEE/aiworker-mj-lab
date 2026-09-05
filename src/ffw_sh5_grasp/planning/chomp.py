"""CHOMP류 궤적 후처리: 가속도(2차 차분) 최소화로 경로를 매끄럽게 한다.

RRT-Connect/RRT*가 만든 경로는 redundant DOF를 무작위로 표본화하므로, 이웃
waypoint와 비교해 관절값이 툭 튀는("기괴한") 지점이 남을 수 있다. shortcut
평활화는 경로를 *짧게* 만들 뿐 남은 waypoint 자체를 *매끄럽게* 만들지는
않는다. 이 모듈은 각 내부 waypoint를 이웃과의 2차 차분(가속도류) 비용이
줄어드는 방향으로 당긴다 — CHOMP의 핵심 아이디어를 가장 단순한 형태로
구현한 것이다. 시작·끝 waypoint는 절대 바뀌지 않는다.

관절 간 결합이 전혀 없어(비용도, box 제약도 관절별 독립) 7-DOF를 하나의 큰
QP로 풀지 않고 관절마다 작은 1-D QP를 독립적으로 푼다.
``kinematics.optimization``의 범용 convex QP 유틸리티를 그대로 재사용한다
(로봇 모델을 모르는 순수 수치 구현이라 이 모듈에서 수정할 필요가 없다).

QP 자체는 충돌을 모른다 — 안전장치는 trust region(원래 waypoint 근처로만
움직임 허용) + 최적화 후 ``EdgeChecker`` 재검증 + 실패 시 원본 그대로
반환이다. 난수를 쓰지 않아 결정론적이다(``rng`` 인자 없음).
"""

import numpy as np

from ..kinematics.optimization import bounded_quadratic_program, least_squares_to_qp


def path_smoothness_cost(path):
    """경로의 가속도류 비용 ``Σ‖q_{i-1}-2q_i+q_{i+1}‖²``. 최적화 전/후 비교용."""
    path = np.asarray(path, dtype=float)
    if len(path) < 3:
        return 0.0
    accel = path[:-2] - 2.0 * path[1:-1] + path[2:]
    return float(np.sum(accel**2))


def _solve_joint(values, lower, upper):
    """관절 하나의 내부 waypoint 값을 2차 차분 최소화로 푼다.

    ``values``는 시작·끝을 포함한 전체 waypoint의 그 관절 값(길이 K).
    반환값은 내부 waypoint(``values[1:-1]``)에 대한 새 값이다.
    """
    count = len(values) - 2  # 내부 waypoint 개수
    matrix = np.zeros((count, count))
    vector = np.zeros(count)
    for row in range(count):
        matrix[row, row] = -2.0
        if row > 0:
            matrix[row, row - 1] = 1.0
        else:
            vector[row] -= values[0]  # 시작 waypoint(고정)를 우변으로 이동
        if row < count - 1:
            matrix[row, row + 1] = 1.0
        else:
            vector[row] -= values[-1]  # 끝 waypoint(고정)를 우변으로 이동
    hessian, linear = least_squares_to_qp(matrix, vector)
    return bounded_quadratic_program(hessian, linear, lower, upper)


def smooth_posture(space, edge_checker, path, *, trust_region_rad, max_retries=3):
    """CHOMP류 가속도 최소화로 경로 내부 waypoint를 매끄럽게 한다.

    ``trust_region_rad``(및 관절 한계) 안에서만 움직이도록 제한한 뒤,
    결과 전체를 ``edge_checker``로 재검증한다. 무효하면 ``trust_region_rad``를
    절반으로 줄여 ``max_retries``번까지 재시도하고, 그래도 무효하면 원본
    ``path``를 그대로 반환한다 — 다른 플래너들과 같은 "절대 충돌 경로를
    반환하지 않는다" 계약을 유지한다.
    """
    path = np.asarray(path, dtype=float)
    if len(path) < 3:
        return path.copy()

    radius = float(trust_region_rad)
    for _ in range(max_retries + 1):
        optimized = path.copy()
        for joint in range(space.n):
            values = path[:, joint]
            joint_lower = space.lower[joint] if space.limited[joint] else -np.inf
            joint_upper = space.upper[joint] if space.limited[joint] else np.inf
            lower = np.maximum(values[1:-1] - radius, joint_lower)
            upper = np.minimum(values[1:-1] + radius, joint_upper)
            optimized[1:-1, joint] = _solve_joint(values, lower, upper)

        if all(edge_checker.is_valid(q) for q in optimized) and all(
            edge_checker.is_valid_edge(a, b, check_endpoints=False)
            for a, b in zip(optimized[:-1], optimized[1:])
        ):
            return optimized
        radius *= 0.5

    return path.copy()


__all__ = ["path_smoothness_cost", "smooth_posture"]
