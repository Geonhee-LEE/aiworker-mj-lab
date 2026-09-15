"""``demo_plan_right_arm._solve_valid_ik``의 재시도 시드 전략(R-F-009) 검증.

관절범위 전체 균등 무작위 대신 ``q_init`` 주변 반경을 10%→30%→100%로
단계적으로 넓히는 ``_staged_retry_seeds``가 (1) 각 단계 반경을 실제로
지키고 (2) 요청한 개수를 정확히 채우며 (3) 실제 장면에서 여전히 충돌 없는
IK 해를 찾는지 확인한다.

Headless 단독 실행: ``python3 tests/test_planning_ik_retry_seeding.py``
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
    ArmCollisionChecker,
    RightArmSpace,
)
from scripts.demo_plan_right_arm import (  # noqa: E402
    _RETRY_RADIUS_FRACTIONS,
    _solve_valid_ik,
    _staged_retry_seeds,
)

SITE_NAME = "grasp_target_r"
START_Q = np.array([0.0, -1.4, 0.0, -0.5, 0.0, 0.3, 0.0])
GOAL_Q = np.array([-0.3, -0.9, 0.0, -1.8, 0.0, 0.5, 0.0])


def _scene():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, home_key)
    mujoco.mj_forward(model, data)
    space = RightArmSpace.from_model(model)
    solver = JointSpaceKinematics(model, SITE_NAME, RIGHT_ARM_JOINTS)
    checker = ArmCollisionChecker(model, space, padding_m=0.012)
    return model, data, space, solver, checker


def test_staged_retry_seeds_respect_expanding_radius():
    joint_ranges = np.array([[-1.0, 1.0]] * 4)
    q_init = np.zeros(4)
    rng = np.random.default_rng(0)
    n_restarts = 30
    seeds = _staged_retry_seeds(q_init, joint_ranges, rng, n_restarts)
    assert len(seeds) == n_restarts

    n_stages = len(_RETRY_RADIUS_FRACTIONS)
    per_stage = n_restarts // n_stages
    offset = 0
    for stage_index, fraction in enumerate(_RETRY_RADIUS_FRACTIONS):
        count = per_stage + (n_restarts - per_stage * n_stages if stage_index == n_stages - 1 else 0)
        stage_seeds = seeds[offset : offset + count]
        offset += count
        radius = fraction * (joint_ranges[:, 1] - joint_ranges[:, 0])
        for seed in stage_seeds:
            assert np.all(np.abs(seed - q_init) <= radius + 1e-9)


def test_staged_retry_seeds_clip_to_joint_limits():
    joint_ranges = np.array([[-1.0, 1.0]])
    q_init = np.array([0.95])
    rng = np.random.default_rng(1)
    seeds = _staged_retry_seeds(q_init, joint_ranges, rng, 10)
    for seed in seeds:
        assert np.all(seed >= joint_ranges[:, 0])
        assert np.all(seed <= joint_ranges[:, 1])


def test_solve_valid_ik_finds_collision_free_solution():
    model, data, space, solver, checker = _scene()
    target_state = solver.forward(GOAL_Q, data.qpos)
    rng = np.random.default_rng(2)
    q, pos_err, valid = _solve_valid_ik(
        solver, checker, START_Q, target_state.position, data.qpos, rng
    )
    assert valid
    assert pos_err < 0.01
    assert checker.is_valid(q)


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
