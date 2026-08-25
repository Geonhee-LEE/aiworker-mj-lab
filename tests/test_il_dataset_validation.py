import pathlib
import sys
import tempfile

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.data.episode import EpisodeData, write_episode
from ffw_sh5_grasp.imitation.data.validation import inspect_dataset


def test_dataset_inspection_reports_valid_episode():
    with tempfile.TemporaryDirectory() as directory:
        root = pathlib.Path(directory)
        write_episode(
            root / "episode_000000.hdf5",
            EpisodeData(
                qpos=np.zeros((2, 16), np.float32),
                qvel=np.zeros((2, 16), np.float32),
                action=np.zeros((2, 16), np.float32),
                images={"cam_high": np.zeros((2, 4, 5, 3), np.uint8)},
                debug={},
                attrs={"success": True},
                ee_pose={
                    "left": np.zeros((2, 7), np.float32),
                    "right": np.zeros((2, 7), np.float32),
                },
            ),
        )
        report = inspect_dataset(root, required_cameras=("cam_high",))
    assert report.valid
    assert report.as_dict()["success_count"] == 1


if __name__ == "__main__":
    test_dataset_inspection_reports_valid_episode()
    print("PASS")
