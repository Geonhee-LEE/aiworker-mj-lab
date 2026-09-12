"""Cartesian(site) pose 목표 → 관절공간 목표(``q_goal``) 변환.

``plan_rrt_connect``/``plan_rrt_star``는 관절공간 목표만 받는다 — site pose
목표는 이 모듈에서 먼저 IK로 풀어야 한다. 단일 시드 position-우선 DLS는 홈
자세 근처 특이점에서 수렴하지 않는 경우가 있어(연구 근거: GNN warm-start
성공률이 100%→93%로 떨어지는 사례), ``space.sample()``로 뽑은 무작위 시드로
순차 폴백 재시도한다.
"""

from dataclasses import dataclass

import numpy as np

from ..kinematics.tasks import pose_error


@dataclass(frozen=True)
class PoseGoalResult:
    """``solve_pose_goal_multistart``의 결과 — 최선 해와 수렴 여부."""

    q: np.ndarray
    position_error: float
    orientation_error: float
    converged: bool
    restarts_used: int


def solve_pose_goal(
    solver,
    space,
    q_seed,
    target_pos,
    target_quat,
    *,
    max_iter=70,
    damping=0.05,
    max_joint_delta=0.07,
    pos_tol=1e-4,
    ori_tol=1e-3,
    ori_weight=0.3,
    context_qpos=None,
):
    """Position 우선 DLS/backtracking으로 한 초기값에서 pose를 푼다."""
    q = space.clip(q_seed)
    if q.shape != (space.n,):
        raise ValueError(f"expected {space.n} initial joint positions, got {q.shape}")
    state = solver.forward(q, context_qpos)
    error = pose_error(state.position, state.quaternion, target_pos, target_quat)
    damping_squared = float(damping) ** 2

    for _ in range(max(0, int(max_iter))):
        if error.position_norm < pos_tol and error.orientation_norm < ori_tol:
            break
        position_jacobian = state.jacobian[:3]
        rotation_jacobian = state.jacobian[3:]
        system = position_jacobian @ position_jacobian.T + damping_squared * np.eye(3)
        position_delta = position_jacobian.T @ np.linalg.solve(system, error.position)
        orientation_gradient = rotation_jacobian.T @ error.orientation
        orientation_delta = (
            orientation_gradient
            - position_jacobian.T
            @ np.linalg.solve(system, position_jacobian @ orientation_gradient)
        )
        delta = np.clip(
            position_delta + orientation_delta, -max_joint_delta, max_joint_delta
        )

        current_cost = error.position_norm + ori_weight * error.orientation_norm
        best = None
        step = 1.0
        for _ in range(6):
            candidate_q = space.clip(q + step * delta)
            candidate_state = solver.forward(candidate_q, context_qpos)
            candidate_error = pose_error(
                candidate_state.position,
                candidate_state.quaternion,
                target_pos,
                target_quat,
            )
            cost = (
                candidate_error.position_norm
                + ori_weight * candidate_error.orientation_norm
            )
            if best is None or cost < best[0]:
                best = cost, candidate_q, candidate_state, candidate_error
            if cost < current_cost:
                break
            step *= 0.5
        _, q, state, error = best
    return q, error.position_norm, error.orientation_norm


def solve_pose_goal_multistart(
    solver,
    space,
    q_seed,
    target_pos,
    target_quat,
    rng,
    *,
    n_restarts=8,
    max_iter=250,
    success_pos_tol=0.005,
    success_ori_tol=np.radians(5.0),
    context_qpos=None,
):
    """``q_seed`` 우선, 이후 ``space.sample(rng)`` 무작위 시드로 pose 목표를 푼다.

    성공 허용오차를 만족하는 첫 해를 즉시 반환하고, 끝까지 실패하면 오차
    합이 가장 작은 해를 ``converged=False``로 반환한다.
    """
    seed = space.clip(q_seed)
    if seed.shape != (space.n,):
        raise ValueError(f"expected {space.n} initial joint positions, got {seed.shape}")
    candidates = [seed]
    candidates.extend(space.sample(rng) for _ in range(max(0, int(n_restarts))))
    best = None
    for attempt, candidate in enumerate(candidates):
        q, position_error, orientation_error = solve_pose_goal(
            solver,
            space,
            candidate,
            target_pos,
            target_quat,
            max_iter=max_iter,
            context_qpos=context_qpos,
        )
        if position_error < success_pos_tol and orientation_error < success_ori_tol:
            return PoseGoalResult(q, position_error, orientation_error, True, attempt)
        if best is None or position_error + orientation_error < best[1] + best[2]:
            best = (q, position_error, orientation_error)
    return PoseGoalResult(best[0], best[1], best[2], False, len(candidates) - 1)


__all__ = ["PoseGoalResult", "solve_pose_goal", "solve_pose_goal_multistart"]
