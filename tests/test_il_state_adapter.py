import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.control import grasp  # noqa: E402
from ffw_sh5_grasp.imitation.state_adapter import PolicyStateAdapter  # noqa: E402


def test_policy_state_adapter():
    model = mujoco.MjModel.from_xml_path(str(REPO_ROOT / "models/full_scene.xml"))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key)
    for side in ("l", "r"):
        grasp.apply_grasp(model, data, grasp=0.65, thumb=0.65, side=side)
    # Position actuators encode their target in ctrl, so mirror only finger qpos
    # for this observer unit test; policy execution itself never performs this write.
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name and name.startswith("finger_"):
            data.qpos[model.jnt_qposadr[joint_id]] = data.ctrl[actuator_id]
    state = PolicyStateAdapter(model)
    qpos, qvel = state.get_qpos(data), state.get_qvel(data)
    assert qpos.shape == (16,) and qvel.shape == (16,)
    assert np.all(np.isfinite(qpos)) and np.all(np.isfinite(qvel))
    assert abs(float(qpos[7]) - 0.65) < 1e-6
    assert abs(float(qpos[15]) - 0.65) < 1e-6


if __name__ == "__main__":
    test_policy_state_adapter()
    print("PASS")
