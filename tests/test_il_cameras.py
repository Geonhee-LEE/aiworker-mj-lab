import os
import pathlib
import sys

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.mujoco_env import AIWorkerMujocoEnv  # noqa: E402


def test_policy_cameras():
    with AIWorkerMujocoEnv(camera_width=160, camera_height=120, seed=3) as env:
        images = env.get_images()
        assert set(images) == {
            "cam_high", "cam_left_wrist", "cam_right_wrist"}
        for image in images.values():
            assert image.shape == (120, 160, 3)
            assert image.dtype == np.uint8 and float(image.std()) > 5.0
        camera_id = mujoco.mj_name2id(
            env.model, mujoco.mjtObj.mjOBJ_CAMERA, "cam_left_wrist")
        before = env.data.cam_xpos[camera_id].copy()
        joint_id = mujoco.mj_name2id(
            env.model, mujoco.mjtObj.mjOBJ_JOINT, "arm_l_joint2")
        env.data.qpos[env.model.jnt_qposadr[joint_id]] += 0.2
        mujoco.mj_forward(env.model, env.data)
        after = env.data.cam_xpos[camera_id].copy()
        assert np.linalg.norm(after - before) > 1e-3


if __name__ == "__main__":
    test_policy_cameras()
    print("PASS")
