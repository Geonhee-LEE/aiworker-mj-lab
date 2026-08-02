"""Whole-body IK가 사용하는 NumPy 기반 bounded least-squares solver.

18개 제어 변수의 box constraint와 collision CBF soft constraint를 외부 QP 패키지
없이 푼다. 로봇 모델이나 MuJoCo 상태를 알지 않는 순수 수치 계산 모듈이다.
"""

import math

import numpy as np

from ..config import SETTINGS


TOLERANCE = SETTINGS.number("optimization.tolerance", positive=True)
ITERATION_MULTIPLIER = SETTINGS.integer(
    "optimization.iteration_multiplier", minimum=1)
EXTRA_ITERATIONS = SETTINGS.integer("optimization.extra_iterations", minimum=0)
BARRIER_ITERATION_MULTIPLIER = SETTINGS.integer(
    "optimization.barrier_iteration_multiplier", minimum=1)
BARRIER_EXTRA_ITERATIONS = SETTINGS.integer(
    "optimization.barrier_extra_iterations", minimum=0)


def bounded_least_squares(matrix, vector, lower, upper):
    """BVLS active set으로 box-constrained least-squares를 푼다.

    경계에 고정된 변수가 KKT 조건을 위반하면 다시 free set으로 돌려보낸다. 따라서
    한 번 경계에 닿은 변수를 영구 고정하던 단방향 active set과 달리 box-QP 최적해에
    도달할 수 있다.
    """
    matrix = np.asarray(matrix, dtype=float)
    vector = np.asarray(vector, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    if matrix.ndim != 2 or vector.shape != (matrix.shape[0],):
        raise ValueError("incompatible least-squares matrix/vector shapes")
    if lower.shape != (matrix.shape[1],) or upper.shape != lower.shape:
        raise ValueError("incompatible least-squares bound shapes")
    if np.any(lower > upper):
        raise ValueError("lower bound exceeds upper bound")

    tolerance = TOLERANCE
    fixed = upper - lower <= tolerance
    movable = ~fixed
    x = np.zeros(matrix.shape[1], dtype=float)
    x[fixed] = 0.5 * (lower[fixed] + upper[fixed])
    if np.any(movable):
        reduced_rhs = vector - matrix[:, fixed] @ x[fixed]
        solution, *_ = np.linalg.lstsq(
            matrix[:, movable], reduced_rhs, rcond=None)
        x[movable] = np.clip(solution, lower[movable], upper[movable])
    active_lower = movable & (x <= lower + tolerance)
    active_upper = movable & ~active_lower & (x >= upper - tolerance)

    for _ in range(ITERATION_MULTIPLIER * matrix.shape[1] + EXTRA_ITERATIONS):
        active = fixed | active_lower | active_upper
        free = ~active
        candidate = x.copy()
        if np.any(free):
            reduced_rhs = vector - matrix[:, active] @ x[active]
            candidate[free], *_ = np.linalg.lstsq(
                matrix[:, free], reduced_rhs, rcond=None)

        # 후보가 box 밖이면 가장 먼저 닿는 경계까지만 이동한다.
        direction = candidate - x
        step = 1.0
        for index in np.flatnonzero(free):
            if candidate[index] < lower[index] - tolerance:
                step = min(
                    step, (lower[index] - x[index]) / direction[index])
            elif candidate[index] > upper[index] + tolerance:
                step = min(
                    step, (upper[index] - x[index]) / direction[index])
        if step < 1.0 - tolerance:
            x += max(0.0, step) * direction
            x = np.clip(x, lower, upper)
            active_lower |= free & (direction < 0.0) & (x <= lower + tolerance)
            active_upper |= free & (direction > 0.0) & (x >= upper - tolerance)
            continue

        # 실행 가능한 후보에서는 gradient 부호를 보고 잘못 고정된 경계를 하나 해제한다.
        x = np.clip(candidate, lower, upper)
        residual = matrix @ x - vector
        gradient = matrix.T @ residual
        lower_violation = np.where(active_lower, -gradient, -np.inf)
        upper_violation = np.where(active_upper, gradient, -np.inf)
        lower_index = int(np.argmax(lower_violation))
        upper_index = int(np.argmax(upper_violation))
        worst_lower = lower_violation[lower_index]
        worst_upper = upper_violation[upper_index]
        if max(worst_lower, worst_upper) <= tolerance:
            break
        if worst_lower >= worst_upper:
            active_lower[lower_index] = False
        else:
            active_upper[upper_index] = False
    return np.clip(x, lower, upper)


def bounded_least_squares_with_barriers(
        matrix, vector, lower, upper,
        barrier_matrix, barrier_lower, slack_weight):
    """box least-squares에 한쪽 방향 soft barrier를 추가해 푼다.

    ``Gx + s >= h``의 non-negative slack을 제거하면
    ``weight * max(0, h-Gx)^2``가 된다. 위반 중인 행만 least-squares에 추가하고
    active set이 안정될 때까지 반복한다.
    """
    barrier_matrix = np.asarray(barrier_matrix, dtype=float)
    barrier_lower = np.asarray(barrier_lower, dtype=float)
    if barrier_matrix.ndim != 2 or barrier_matrix.shape[1] != np.shape(matrix)[1]:
        raise ValueError("incompatible collision barrier matrix shape")
    if barrier_lower.shape != (barrier_matrix.shape[0],):
        raise ValueError("incompatible collision barrier lower-bound shape")
    if barrier_matrix.shape[0] == 0:
        return bounded_least_squares(matrix, vector, lower, upper)

    root_weight = math.sqrt(float(slack_weight))
    solution = bounded_least_squares(matrix, vector, lower, upper)
    active = barrier_matrix @ solution < barrier_lower
    for _ in range(
            BARRIER_ITERATION_MULTIPLIER * barrier_matrix.shape[0]
            + BARRIER_EXTRA_ITERATIONS):
        if not np.any(active):
            return solution
        augmented_matrix = np.vstack(
            (matrix, root_weight * barrier_matrix[active]))
        augmented_vector = np.concatenate(
            (vector, root_weight * barrier_lower[active]))
        candidate = bounded_least_squares(
            augmented_matrix, augmented_vector, lower, upper)
        next_active = barrier_matrix @ candidate < barrier_lower - TOLERANCE
        if np.array_equal(next_active, active):
            return candidate
        solution, active = candidate, next_active
    return solution
