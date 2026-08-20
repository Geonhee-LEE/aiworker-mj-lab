import os
import pathlib
import sys

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.mujoco_env import AIWorkerMujocoEnv  # noqa: E402


def _id(model, kind, name):
    object_id = mujoco.mj_name2id(model, kind, name)
    assert object_id >= 0
    return object_id


def test_camera_bracket_extrinsics():
    model = mujoco.MjModel.from_xml_path(
        str(REPO_ROOT / "models/full_scene.xml"))
    expected_bracket_quat = [
        0.475435128973, -0.475435128973,
        0.523413257512, 0.523413257512,
    ]
    expected_camera_quat = [-0.5, -0.5, 0.5, 0.5]

    assert np.allclose(model.body_pos[_id(
        model, mujoco.mjtObj.mjOBJ_BODY, "zedm_camera_link")],
        [0.0238122, -0.00651797, -0.0242094])
    assert np.allclose(model.body_pos[_id(
        model, mujoco.mjtObj.mjOBJ_BODY, "zedm_camera_center")],
        [0.0, 0.0, 0.01325])
    zedm_left_frame = _id(
        model, mujoco.mjtObj.mjOBJ_BODY, "zedm_left_camera_frame")
    assert np.allclose(
        model.body_pos[zedm_left_frame], [0.0, 0.0315, 0.0])
    high_camera = _id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, "cam_high")
    assert model.cam_bodyid[high_camera] == zedm_left_frame
    assert np.allclose(model.cam_pos[high_camera], 0.0)
    assert np.allclose(model.cam_quat[high_camera], expected_camera_quat)

    for side in ("l", "r"):
        bracket = _id(
            model, mujoco.mjtObj.mjOBJ_BODY,
            f"camera_{side}_bottom_screw_frame")
        camera_link = _id(
            model, mujoco.mjtObj.mjOBJ_BODY, f"camera_{side}_link")
        camera_name = "cam_left_wrist" if side == "l" else "cam_right_wrist"
        camera = _id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        assert np.allclose(
            model.body_pos[bracket], [0.108236, -0.021, -0.062552])
        assert np.allclose(model.body_quat[bracket], expected_bracket_quat)
        assert np.allclose(
            model.body_pos[camera_link], [0.01085, 0.009, 0.021])
        assert model.cam_bodyid[camera] == camera_link
        assert np.allclose(model.cam_pos[camera], 0.0)
        assert np.allclose(model.cam_quat[camera], expected_camera_quat)


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
