import pathlib
import sys
import tempfile

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.data.episode import (  # noqa: E402
    EpisodeData, load_episode, write_episode)


def test_hdf5_round_trip():
    episode = EpisodeData(
        qpos=np.zeros((3, 16), np.float32),
        qvel=np.ones((3, 16), np.float32),
        images={"cam_high": np.zeros((3, 4, 5, 3), np.uint8)},
        action=np.full((3, 16), 2.0, np.float32),
        debug={"task_object_pose": np.zeros((3, 7))},
        attrs={"seed": 7, "camera_names": ["cam_high"]},
    )
    with tempfile.TemporaryDirectory() as directory:
        path = write_episode(pathlib.Path(directory) / "episode_000000.hdf5", episode)
        loaded = load_episode(path)
    assert loaded.length == 3 and loaded.attrs["seed"] == 7
    assert loaded.attrs["camera_names"] == ["cam_high"]
    assert np.array_equal(loaded.action, episode.action)


if __name__ == "__main__":
    test_hdf5_round_trip()
    print("PASS")
