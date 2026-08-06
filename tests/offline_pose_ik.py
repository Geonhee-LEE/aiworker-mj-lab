"""Phase pick 회귀에서만 목표 관절 자세를 만드는 오프라인 IK helper.

실시간 제품 API가 아니다. 런타임은 ``WholeBodyIK``의 differential solver만 사용한다.
"""

import numpy as np

from ffw_sh5_grasp.kinematics.tasks import pose_error


def _clamp(solver, q):
    result = np.asarray(q, dtype=float).copy()
    result[solver.joint_limited] = np.clip(
        result[solver.joint_limited],
        solver.joint_ranges[solver.joint_limited, 0],
        solver.joint_ranges[solver.joint_limited, 1])
    return result


def solve_offline_pose(solver, q_init, target_pos, target_quat, *,
                       max_iter=70, damping=0.05, max_joint_delta=0.07,
                       pos_tol=1e-4, ori_tol=1e-3, ori_weight=0.3,
                       context_qpos=None):
    """Position 우선 DLS/backtracking으로 한 오프라인 pose를 푼다."""
    q = _clamp(solver, q_init)
    if q.shape != (solver.n,):
        raise ValueError(f"expected {solver.n} initial joint positions, got {q.shape}")
    state = solver.forward(q, context_qpos)
    error = pose_error(state.position, state.quaternion, target_pos, target_quat)
    damping_squared = float(damping) ** 2

    for _ in range(max(0, int(max_iter))):
        if error.position_norm < pos_tol and error.orientation_norm < ori_tol:
            break
        position_jacobian = state.jacobian[:3]
        rotation_jacobian = state.jacobian[3:]
        system = position_jacobian @ position_jacobian.T + damping_squared * np.eye(3)
        position_delta = position_jacobian.T @ np.linalg.solve(
            system, error.position)
        orientation_gradient = rotation_jacobian.T @ error.orientation
        orientation_delta = orientation_gradient - position_jacobian.T @ np.linalg.solve(
            system, position_jacobian @ orientation_gradient)
        delta = np.clip(
            position_delta + orientation_delta, -max_joint_delta, max_joint_delta)

        current_cost = error.position_norm + ori_weight * error.orientation_norm
        best = None
        step = 1.0
        for _ in range(6):
            candidate_q = _clamp(solver, q + step * delta)
            candidate_state = solver.forward(candidate_q, context_qpos)
            candidate_error = pose_error(
                candidate_state.position, candidate_state.quaternion,
                target_pos, target_quat)
            cost = (
                candidate_error.position_norm
                + ori_weight * candidate_error.orientation_norm)
            if best is None or cost < best[0]:
                best = cost, candidate_q, candidate_state, candidate_error
            if cost < current_cost:
                break
            step *= 0.5
        _, q, state, error = best
    return q, error.position_norm, error.orientation_norm


def solve_offline_pose_multistart(
        solver, q_init, target_pos, target_quat, rng, *, n_restarts=8,
        max_iter=250, success_pos_tol=0.005,
        success_ori_tol=np.radians(5.0), context_qpos=None):
    """여러 초기값에서 test-only pose IK를 풀고 가장 좋은 해를 반환한다."""
    initial = np.asarray(q_init, dtype=float)
    if initial.shape != (solver.n,):
        raise ValueError(
            f"expected {solver.n} initial joint positions, got {initial.shape}")
    lower = np.where(
        solver.joint_limited, solver.joint_ranges[:, 0], initial - np.pi)
    upper = np.where(
        solver.joint_limited, solver.joint_ranges[:, 1], initial + np.pi)
    candidates = [initial]
    candidates.extend(
        rng.uniform(lower, upper) for _ in range(max(0, int(n_restarts))))
    best = None
    for candidate in candidates:
        q, position_error, orientation_error = solve_offline_pose(
            solver, candidate, target_pos, target_quat,
            max_iter=max_iter, context_qpos=context_qpos)
        if position_error < success_pos_tol and orientation_error < success_ori_tol:
            return q, position_error, orientation_error, True
        if best is None or position_error + orientation_error < best[1] + best[2]:
            best = q, position_error, orientation_error
    return best[0], best[1], best[2], False
