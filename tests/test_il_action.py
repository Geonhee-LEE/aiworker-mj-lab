import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.action import (  # noqa: E402
    ACTION_DIM, ACTION_NAMES, ActionAdapter)
from ffw_sh5_grasp.imitation.leader import GizmoLeader  # noqa: E402
from ffw_sh5_grasp.imitation.mujoco_env import AIWorkerMujocoEnv  # noqa: E402


def test_action_contract():
    model = mujoco.MjModel.from_xml_path(str(REPO_ROOT / "models/full_scene.xml"))
    adapter = ActionAdapter(model)
    action = adapter.encode(np.zeros(7), 0.25, np.zeros(7), 0.75)
    decoded = adapter.decode(action)
    assert ACTION_DIM == 16 and len(ACTION_NAMES) == 16
    assert np.allclose(decoded.arm_positions["l"], 0.0)
    assert decoded.grasp == {"l": 0.25, "r": 0.75}
    assert adapter.validate(np.full(16, 1e6), clip=True).shape == (16,)
    try:
        adapter.validate(np.zeros(15))
    except ValueError:
        pass
    else:
        raise AssertionError("invalid action shape was accepted")


def test_locked_left_leader_action():
    with AIWorkerMujocoEnv(render_images=False, seed=3) as env:
        leader = GizmoLeader(env)
        action = leader.get_action()
        assert np.allclose(action[:7], env.left_arm_park_position)
        assert action[7] == env.left_grasp_fixed


if __name__ == "__main__":
    test_action_contract()
    test_locked_left_leader_action()
    print("PASS")
