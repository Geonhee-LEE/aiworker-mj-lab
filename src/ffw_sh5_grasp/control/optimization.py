"""Whole-body IK가 사용하는 NumPy 기반 convex QP solver.

Weighted DLS의 ``A, b``를 표준 QP의 Hessian ``H``와 선형항 ``g``로 변환하고,
18개 제어 변수의 box constraint와 collision CBF soft constraint를 외부 QP 패키지
없이 active-set으로 푼다. 로봇 모델이나 MuJoCo 상태를 알지 않는 순수 수치 계산
모듈이며, 이전 bounded least-squares API는 호환 wrapper로 유지한다.
"""

import numpy as np


# 다음 값은 사용자가 튜닝할 제어 파라미터가 아니라 18변수 active-set 구현의 수치
# 안전장치다. YAML 표면적을 줄이고 검증된 해법과 설정을 함께 바꾸는 일을 막는다.
TOLERANCE = 1e-10
ITERATION_MULTIPLIER = 4
EXTRA_ITERATIONS = 4
BARRIER_ITERATION_MULTIPLIER = 2
BARRIER_EXTRA_ITERATIONS = 4


def least_squares_to_qp(matrix, vector):
    """``||Ax-b||²``를 표준 QP의 ``(H, g)``로 변환한다.

    상수 ``b.T @ b``를 제외하면 ``0.5*x.T@H@x + g.T@x``와 같도록
    ``H=2*A.T@A``, ``g=-2*A.T@b``를 반환한다. Weighted task와 DLS 정규화 행을
    ``A``에 함께 쌓으면 동일한 변환으로 전신 QP 목적함수를 만들 수 있다.
    """
    matrix = np.asarray(matrix, dtype=float)
    vector = np.asarray(vector, dtype=float)
    if matrix.ndim != 2 or vector.shape != (matrix.shape[0],):
        raise ValueError("incompatible least-squares matrix/vector shapes")
    return 2.0 * (matrix.T @ matrix), -2.0 * (matrix.T @ vector)


def bounded_quadratic_program(hessian, linear, lower, upper):
    """Active-set으로 box-constrained convex QP를 푼다.

    목적함수는 ``0.5*x.T@H@x + g.T@x``이고 제약은 ``lower <= x <= upper``다.
    경계에 고정된 변수가 KKT 조건을 위반하면 다시 free set으로 돌려보내므로 한 번
    경계에 닿은 변수를 영구 고정하지 않는다. ``H``가 singular한 convex 문제도
    최소제곱 선형계로 유한한 최소노름 후보를 계산한다.
    """
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
    if not np.allclose(
            hessian, hessian.T, atol=TOLERANCE, rtol=TOLERANCE):
        raise ValueError("QP Hessian must be symmetric")
    hessian = 0.5 * (hessian + hessian.T)

    tolerance = TOLERANCE
    fixed = upper - lower <= tolerance
    movable = ~fixed
    x = np.zeros(variable_count, dtype=float)
    x[fixed] = 0.5 * (lower[fixed] + upper[fixed])
    if np.any(movable):
        reduced_hessian = hessian[np.ix_(movable, movable)]
        reduced_linear = (
            linear[movable]
            + hessian[np.ix_(movable, fixed)] @ x[fixed]
        )
        solution, *_ = np.linalg.lstsq(
            reduced_hessian, -reduced_linear, rcond=None)
        x[movable] = np.clip(solution, lower[movable], upper[movable])
    active_lower = movable & (x <= lower + tolerance)
    active_upper = movable & ~active_lower & (x >= upper - tolerance)

    for _ in range(ITERATION_MULTIPLIER * variable_count + EXTRA_ITERATIONS):
        active = fixed | active_lower | active_upper
        free = ~active
        candidate = x.copy()
        if np.any(free):
            reduced_hessian = hessian[np.ix_(free, free)]
            reduced_linear = (
                linear[free]
                + hessian[np.ix_(free, active)] @ x[active]
            )
            candidate[free], *_ = np.linalg.lstsq(
                reduced_hessian, -reduced_linear, rcond=None)

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
        gradient = hessian @ x + linear
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


def bounded_quadratic_program_with_barriers(
        hessian, linear, lower, upper,
        barrier_matrix, barrier_lower, slack_weight):
    """Box-QP에 한쪽 방향의 quadratic soft barrier를 추가해 푼다.

    ``Gx + s >= h``의 non-negative slack을 제거한
    ``weight * max(0, h-Gx)^2``를 목적함수에 더한다. 위반 중인 행의 Hessian과
    선형항만 추가하고 barrier active set이 안정될 때까지 반복한다.
    """
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
        return bounded_quadratic_program(
            hessian, linear, lower, upper)

    solution = bounded_quadratic_program(
        hessian, linear, lower, upper)
    active = barrier_matrix @ solution < barrier_lower
    for _ in range(
            BARRIER_ITERATION_MULTIPLIER * barrier_matrix.shape[0]
            + BARRIER_EXTRA_ITERATIONS):
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


def bounded_least_squares(matrix, vector, lower, upper):
    """기존 ``Ax≈b`` API를 명시적 box-QP solver에 연결하는 호환 wrapper다."""
    hessian, linear = least_squares_to_qp(matrix, vector)
    return bounded_quadratic_program(
        hessian, linear, lower, upper)


def bounded_least_squares_with_barriers(
        matrix, vector, lower, upper,
        barrier_matrix, barrier_lower, slack_weight):
    """기존 least-squares soft-barrier API를 QP solver에 연결하는 호환 wrapper다."""
    hessian, linear = least_squares_to_qp(matrix, vector)
    return bounded_quadratic_program_with_barriers(
        hessian, linear, lower, upper,
        barrier_matrix, barrier_lower, slack_weight)
