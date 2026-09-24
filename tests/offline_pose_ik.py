"""Phase pick 회귀에서만 목표 관절 자세를 만드는 오프라인 IK helper.

실시간 제품 API가 아니다. 런타임은 ``WholeBodyIK``의 differential solver만 사용한다.
수치 알고리즘 자체는 ``planning.goals``(``solve_pose_goal``/``solve_pose_goal_multistart``)
로 위임한다 — 여기서는 solver의 관절 범위로 임시 ``RightArmSpace``를 만들어 넘기는
얇은 어댑터만 유지해 두 곳에 같은 DLS/backtracking 로직이 중복되지 않게 한다.
"""

import numpy as np

from ffw_sh5_grasp.planning import (
    RightArmSpace,
    solve_pose_goal,
    solve_pose_goal_multistart,
)


def _space_for(solver):
    """``solver``(``JointSpaceKinematics``)의 관절 범위로 임시 ``RightArmSpace``를 만든다."""
    indices = np.arange(solver.n, dtype=int)
    return RightArmSpace(
        solver.joint_names,
        indices,
        indices,
        indices,
        solver.joint_ranges[:, 0],
        solver.joint_ranges[:, 1],
        solver.joint_limited,
    )


def solve_offline_pose(solver, q_init, target_pos, target_quat, **kwargs):
    """Position 우선 DLS/backtracking으로 한 오프라인 pose를 푼다."""
    return solve_pose_goal(solver, _space_for(solver), q_init, target_pos, target_quat, **kwargs)


def solve_offline_pose_multistart(solver, q_init, target_pos, target_quat, rng, **kwargs):
    """여러 초기값에서 test-only pose IK를 풀고 가장 좋은 해를 반환한다."""
    result = solve_pose_goal_multistart(
        solver, _space_for(solver), q_init, target_pos, target_quat, rng, **kwargs
    )
    return result.q, result.position_error, result.orientation_error, result.converged
