"""Differential IK가 사용하는 NumPy 기반 convex QP 수치 구현.

로봇 모델이나 제어 상태를 알지 않으며, least-squares 변환과 box/soft-barrier
active-set 계산만 담당한다.
"""

import numpy as np

# 반복 상한과 허용 오차는 사용자 제어 이득이 아니라 수치 구현의 안전장치다.
TOLERANCE = 1e-10
ITERATION_MULTIPLIER = 4
EXTRA_ITERATIONS = 4
BARRIER_ITERATION_MULTIPLIER = 2
BARRIER_EXTRA_ITERATIONS = 4


def least_squares_to_qp(matrix, vector):
    """``||Ax-b||²``를 표준 QP의 Hessian과 선형항으로 변환한다."""
    matrix = np.asarray(matrix, dtype=float)
    vector = np.asarray(vector, dtype=float)
    if matrix.ndim != 2 or vector.shape != (matrix.shape[0],):
        raise ValueError("incompatible least-squares matrix/vector shapes")
    return 2.0 * (matrix.T @ matrix), -2.0 * (matrix.T @ vector)


def bounded_quadratic_program(hessian, linear, lower, upper):
    """Active-set으로 ``lower <= x <= upper``인 convex QP를 푼다."""
    hessian = np.asarray(hessian, dtype=float)
    linear = np.asarray(linear, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    if hessian.ndim != 2 or hessian.shape[0] != hessian.shape[1]:
        raise ValueError("QP Hessian must be square")
    variable_count = hessian.shape[0]
    if linear.shape != (variable_count,):
        raise ValueError("incompatible QP Hessian/linear shapes")
    if lower.shape != (variable_count,) or upper.shape != lower.shape:
        raise ValueError("incompatible QP bound shapes")
    if np.any(lower > upper):
        raise ValueError("lower bound exceeds upper bound")
    if not np.allclose(hessian, hessian.T, atol=TOLERANCE, rtol=TOLERANCE):
        raise ValueError("QP Hessian must be symmetric")
    hessian = 0.5 * (hessian + hessian.T)

    fixed = upper - lower <= TOLERANCE
    movable = ~fixed
    x = np.zeros(variable_count, dtype=float)
    x[fixed] = 0.5 * (lower[fixed] + upper[fixed])
    if np.any(movable):
        reduced_hessian = hessian[np.ix_(movable, movable)]
        reduced_linear = (
            linear[movable] + hessian[np.ix_(movable, fixed)] @ x[fixed])
        solution, *_ = np.linalg.lstsq(
            reduced_hessian, -reduced_linear, rcond=None)
        x[movable] = np.clip(solution, lower[movable], upper[movable])
    active_lower = movable & (x <= lower + TOLERANCE)
    active_upper = movable & ~active_lower & (x >= upper - TOLERANCE)

    iteration_count = ITERATION_MULTIPLIER * variable_count + EXTRA_ITERATIONS
    for _ in range(iteration_count):
        active = fixed | active_lower | active_upper
        free = ~active
        candidate = x.copy()
        if np.any(free):
            reduced_hessian = hessian[np.ix_(free, free)]
            reduced_linear = (
                linear[free] + hessian[np.ix_(free, active)] @ x[active])
            candidate[free], *_ = np.linalg.lstsq(
                reduced_hessian, -reduced_linear, rcond=None)

        direction = candidate - x
        step = 1.0
        for index in np.flatnonzero(free):
            if candidate[index] < lower[index] - TOLERANCE:
                step = min(step, (lower[index] - x[index]) / direction[index])
            elif candidate[index] > upper[index] + TOLERANCE:
                step = min(step, (upper[index] - x[index]) / direction[index])
        if step < 1.0 - TOLERANCE:
            x += max(0.0, step) * direction
            x = np.clip(x, lower, upper)
            active_lower |= free & (direction < 0.0) & (x <= lower + TOLERANCE)
            active_upper |= free & (direction > 0.0) & (x >= upper - TOLERANCE)
            continue

        x = np.clip(candidate, lower, upper)
        gradient = hessian @ x + linear
        lower_violation = np.where(active_lower, -gradient, -np.inf)
        upper_violation = np.where(active_upper, gradient, -np.inf)
        lower_index = int(np.argmax(lower_violation))
        upper_index = int(np.argmax(upper_violation))
        worst_lower = lower_violation[lower_index]
        worst_upper = upper_violation[upper_index]
        if max(worst_lower, worst_upper) <= TOLERANCE:
            break
        if worst_lower >= worst_upper:
            active_lower[lower_index] = False
        else:
            active_upper[upper_index] = False
    return np.clip(x, lower, upper)


def bounded_quadratic_program_with_barriers(
        hessian, linear, lower, upper,
        barrier_matrix, barrier_lower, slack_weight):
    """Box-QP에 ``Gx >= h`` quadratic soft barrier를 추가해 푼다."""
    hessian = np.asarray(hessian, dtype=float)
    linear = np.asarray(linear, dtype=float)
    barrier_matrix = np.asarray(barrier_matrix, dtype=float)
    barrier_lower = np.asarray(barrier_lower, dtype=float)
    if barrier_matrix.ndim != 2 or barrier_matrix.shape[1] != hessian.shape[1]:
        raise ValueError("incompatible collision barrier matrix shape")
    if barrier_lower.shape != (barrier_matrix.shape[0],):
        raise ValueError("incompatible collision barrier lower-bound shape")
    slack_weight = float(slack_weight)
    if slack_weight <= 0.0:
        raise ValueError("barrier slack weight must be positive")
    if barrier_matrix.shape[0] == 0:
        return bounded_quadratic_program(hessian, linear, lower, upper)

    solution = bounded_quadratic_program(hessian, linear, lower, upper)
    active = barrier_matrix @ solution < barrier_lower
    iteration_count = (
        BARRIER_ITERATION_MULTIPLIER * barrier_matrix.shape[0]
        + BARRIER_EXTRA_ITERATIONS)
    for _ in range(iteration_count):
        if not np.any(active):
            return solution
        active_matrix = barrier_matrix[active]
        active_lower = barrier_lower[active]
        augmented_hessian = (
            hessian + 2.0 * slack_weight * active_matrix.T @ active_matrix)
        augmented_linear = (
            linear - 2.0 * slack_weight * active_matrix.T @ active_lower)
        candidate = bounded_quadratic_program(
            augmented_hessian, augmented_linear, lower, upper)
        next_active = barrier_matrix @ candidate < barrier_lower - TOLERANCE
        if np.array_equal(next_active, active):
            return candidate
        solution, active = candidate, next_active
    return solution


__all__ = [
    "bounded_quadratic_program",
    "bounded_quadratic_program_with_barriers",
    "least_squares_to_qp",
]
