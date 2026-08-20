import pathlib
import sys

import numpy as np

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
        import mujoco
        mujoco.mj_forward(env.model, env.data)
        assert env.task.metrics(env.data).success


if __name__ == "__main__":
    test_arm_only_env_reset_and_step()
    print("PASS")
