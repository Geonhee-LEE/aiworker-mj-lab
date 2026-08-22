import pathlib
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.data.recording import EpisodeBuffer  # noqa: E402


def test_obs_action_alignment():
    buffer = EpisodeBuffer()
    for frame in range(4):
        observation = {
            "qpos": np.full(16, frame, dtype=np.float32),
            "qvel": np.zeros(16, dtype=np.float32),
            "ee_pose": {
                "left": np.full(7, frame, dtype=np.float32),
                "right": np.full(7, frame + 10, dtype=np.float32),
            },
            "images": {"cam_high": np.full(
                (2, 3, 3), frame, dtype=np.uint8)},
            "debug": {},
        }
        action = np.full(16, 100 + frame, dtype=np.float32)
        buffer.append(observation, action)
    episode = buffer.freeze()
    assert np.array_equal(episode.qpos[:, 0], np.arange(4))
    assert np.array_equal(episode.action[:, 0], 100 + np.arange(4))
    assert np.array_equal(episode.ee_pose["left"][:, 0], np.arange(4))
    assert np.array_equal(
        episode.ee_pose["right"][:, 0], 10 + np.arange(4))
    assert np.array_equal(episode.images["cam_high"][:, 0, 0, 0], np.arange(4))


if __name__ == "__main__":
    test_obs_action_alignment()
    print("PASS")
