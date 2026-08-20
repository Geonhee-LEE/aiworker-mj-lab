import pathlib
import sys

import numpy as np
import mujoco

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.mujoco_env import AIWorkerMujocoEnv  # noqa: E402


def test_arm_only_env_reset_and_step():
    with AIWorkerMujocoEnv(render_images=False, seed=11) as env:
        first = env.reset(seed=11)
        first_can = env.initial_can_position.copy()
        robot_home = first["debug"]["full_qpos"][:env.task.can_qpos].copy()
        assert first["qpos"].shape == (16,)
        assert first["images"] == {}
        assert np.allclose(first["qpos"][:7], env.left_arm_park_position)
        assert first["qpos"][7] == env.left_grasp_fixed
        assert np.all(env.model.geom_contype[env.target_bin_geom_ids] == 1)
        assert np.all(env.model.geom_conaffinity[env.target_bin_geom_ids] == 1)
        left_site = mujoco.mj_name2id(
            env.model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l")
        palm_normal = env.data.site_xmat[left_site].reshape(3, 3)[:, 0]
        assert np.dot(palm_normal, [0.0, 0.0, 1.0]) > 0.999
        requested = first["qpos"].copy()
        requested[:8] = 0.5
        prepared = env.prepare_action(requested)
        assert np.allclose(prepared[:7], env.left_arm_park_position)
        assert prepared[7] == env.left_grasp_fixed
        env.step(first["qpos"])
        reset = env.reset(seed=11)
        assert np.allclose(env.initial_can_position, first_can)
        assert np.allclose(
            reset["debug"]["full_qpos"][:env.task.can_qpos], robot_home)
        env.reset(seed=12)
        assert not np.allclose(env.initial_can_position, first_can)
        assert env.steps_per_control * env.model.opt.timestep == 1 / 25
        assert "whole_body_solver" not in vars(env)
        target = env.data.site_xpos[env.task.target_site].copy()
        env.data.qpos[env.task.can_qpos:env.task.can_qpos + 3] = target
        env.data.qvel[env.task.can_dof:env.task.can_dof + 6] = 0.0
        mujoco.mj_forward(env.model, env.data)
        assert env.task.metrics(env.data).success

        # A slight floor penetration must create a real can/bin contact, not
        # merely satisfy the task's geometric success test.
        env.data.qpos[env.task.can_qpos + 2] -= 0.002
        mujoco.mj_forward(env.model, env.data)
        can_geom = mujoco.mj_name2id(
            env.model, mujoco.mjtObj.mjOBJ_GEOM, "can_geom")
        floor_geom = mujoco.mj_name2id(
            env.model, mujoco.mjtObj.mjOBJ_GEOM, "target_bin_floor")
        contact_pairs = {
            frozenset((int(contact.geom1), int(contact.geom2)))
            for contact in env.data.contact
        }
        assert frozenset((can_geom, floor_geom)) in contact_pairs


if __name__ == "__main__":
    test_arm_only_env_reset_and_step()
    print("PASS")
