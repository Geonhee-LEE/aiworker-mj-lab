"""Cartesian pose 목표 → 관절공간 목표(``planning.goals``) 검증.

실제 can-sort 장면에서 알려진 관절공간 목표의 순기구학 pose를 되찾는지
검사한다 — 그래야 RRT-Connect/RRT*에 넘길 ``q_goal``이 실제로 요청한
site pose에 대응함을 보장한다.

Headless 단독 실행: ``python3 tests/test_planning_goals.py``
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.kinematics import JointSpaceKinematics  # noqa: E402
from ffw_sh5_grasp.paths import MODEL_PATH  # noqa: E402
from ffw_sh5_grasp.planning import (  # noqa: E402
    RIGHT_ARM_JOINTS,
    RightArmSpace,
    solve_pose_goal,
    solve_pose_goal_multistart,
)

SITE_NAME = "grasp_target_r"
# 일반 teleop ``home`` 자세와 다른, 도달 가능한 두 관절공간 자세(P0에서 유효성
# 확인한 값과 동일 계열 — test_planning_rrt_scene.py의 START_Q/GOAL_Q 참고).
START_Q = np.array([0.0, -1.4, 0.0, -0.5, 0.0, 0.3, 0.0])
GOAL_Q = np.array([-0.3, -0.9, 0.0, -1.8, 0.0, 0.5, 0.0])
POS_TOL = 0.005
ORI_TOL = np.radians(5.0)


def _scene():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, home_key)
    mujoco.mj_forward(model, data)
    space = RightArmSpace.from_model(model)
    solver = JointSpaceKinematics(model, SITE_NAME, RIGHT_ARM_JOINTS)
    return model, data, space, solver


def _site_pose_at(model, data, space, q):
    scratch = mujoco.MjData(model)
    scratch.qpos[:] = data.qpos
    scratch.qpos[space.qpos_adrs] = q
    mujoco.mj_forward(model, scratch)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, SITE_NAME)
    target_pos = scratch.site_xpos[site_id].copy()
    target_quat = np.zeros(4)
    mujoco.mju_mat2Quat(target_quat, scratch.site_xmat[site_id])
    return target_pos, target_quat


def test_solve_pose_goal_converges_from_nearby_seed():
    model, data, space, solver = _scene()
    target_pos, target_quat = _site_pose_at(model, data, space, GOAL_Q)
    seed = np.clip(GOAL_Q + 0.05, space.lower, space.upper)
    q, pos_err, ori_err = solve_pose_goal(
        solver, space, seed, target_pos, target_quat, context_qpos=data.qpos
    )
    assert pos_err < POS_TOL
    assert ori_err < ORI_TOL
    assert space.contains(q, tolerance=1e-6)


def test_multistart_converges_from_distant_seed():
    model, data, space, solver = _scene()
    target_pos, target_quat = _site_pose_at(model, data, space, GOAL_Q)
    result = solve_pose_goal_multistart(
        solver,
        space,
        START_Q,
        target_pos,
        target_quat,
        np.random.default_rng(0),
        success_pos_tol=POS_TOL,
        success_ori_tol=ORI_TOL,
        context_qpos=data.qpos,
    )
    assert result.converged
    assert result.position_error < POS_TOL
    assert result.orientation_error < ORI_TOL
    assert space.contains(result.q, tolerance=1e-6)
    achieved_pos, achieved_quat = _site_pose_at(model, data, space, result.q)
    assert np.linalg.norm(achieved_pos - target_pos) < POS_TOL


def test_multistart_reports_best_effort_on_unreachable_target():
    model, data, space, solver = _scene()
    unreachable_pos = np.array([10.0, 10.0, 10.0])
    unreachable_quat = np.array([1.0, 0.0, 0.0, 0.0])
    result = solve_pose_goal_multistart(
        solver,
        space,
        START_Q,
        unreachable_pos,
        unreachable_quat,
        np.random.default_rng(1),
        n_restarts=2,
        context_qpos=data.qpos,
    )
    assert not result.converged
    assert space.contains(result.q, tolerance=1e-6)


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
