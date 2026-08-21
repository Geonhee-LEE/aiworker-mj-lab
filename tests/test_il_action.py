import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.simulation.action import (  # noqa: E402
    ACTION_DIM, ACTION_NAMES, ActionAdapter)
from ffw_sh5_grasp.imitation.apps.leader import GizmoLeader  # noqa: E402
from ffw_sh5_grasp.imitation.simulation.environment import AIWorkerMujocoEnv  # noqa: E402


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
        assert leader.linear_speed == 1.0
        assert leader.angular_speed == 3.0
        assert leader.joint_speed == 4.8
        assert leader.position_gain == 12.0
        assert leader.orientation_gain == 9.0
        action = leader.get_action()
        assert np.allclose(action[:7], env.left_arm_park_position)
        assert action[7] == env.left_grasp_fixed
        assert leader.toggle_grasp("r") == 1.0
        assert leader.get_action()[15] == 1.0
        assert leader.toggle_grasp("r") == 0.0
        assert leader.get_action()[15] == 0.0


def test_right_arm_bounded_home_return():
    with AIWorkerMujocoEnv(render_images=False, seed=3) as env:
        right_qpos = env.state_adapter.arm_qpos["r"]
        env.data.qpos[right_qpos] += np.asarray(
            [0.2, -0.2, 0.2, -0.2, 0.2, -0.2, 0.2])
        mujoco.mj_forward(env.model, env.data)
        leader = GizmoLeader(env)
        current = env.data.qpos[right_qpos].copy()
        before = np.abs(leader.home_arms["r"] - current)

        leader.return_home("r")
        action = leader.get_action()
        after = np.abs(leader.home_arms["r"] - action[8:15])
        max_step = leader.joint_speed / env.actual_control_hz
        assert leader.returning_home["r"]
        assert np.all(after < before)
        assert np.all(np.abs(action[8:15] - current) <= max_step + 1e-12)

        position, quaternion = leader.targets["r"]
        leader.set_target_pose("r", position, quaternion)
        assert not leader.returning_home["r"]


if __name__ == "__main__":
    test_action_contract()
    test_locked_left_leader_action()
    print("PASS")
