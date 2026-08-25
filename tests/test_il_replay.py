import pathlib
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.data.episode import EpisodeData  # noqa: E402
from ffw_sh5_grasp.imitation.data.replay import replay_episode  # noqa: E402
from ffw_sh5_grasp.imitation.simulation.environment import (
    AIWorkerMujocoEnv,  # noqa: E402
)


def test_deterministic_replay():
    with AIWorkerMujocoEnv(render_images=False, seed=19) as env:
        observation = env.reset(seed=19)
        qpos, qvel, actions = [], [], []
        for _ in range(5):
            action = observation["qpos"].copy()
            qpos.append(observation["qpos"])
            qvel.append(observation["qvel"])
            actions.append(action)
            observation = env.step(action)
        episode = EpisodeData(
            qpos=np.stack(qpos),
            qvel=np.stack(qvel),
            images={},
            action=np.stack(actions),
            debug={},
            attrs={"seed": 19},
        )
        result = replay_episode(env, episode, atol=1e-7)
    assert result["reproduced"]
    assert result["maximum_qpos_error"] <= 1e-7


if __name__ == "__main__":
    test_deterministic_replay()
    print("PASS")
