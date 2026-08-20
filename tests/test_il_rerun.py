import importlib.util
import pathlib
import sys
import tempfile

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.visualization.rerun_dataset import _rerun  # noqa: E402
from ffw_sh5_grasp.imitation.dataset import EpisodeData  # noqa: E402
from ffw_sh5_grasp.imitation.visualization.rerun_dataset import log_episode  # noqa: E402
from ffw_sh5_grasp.imitation.visualization.rerun_rollout import (  # noqa: E402
    RolloutRerunLogger)
from ffw_sh5_grasp.imitation.visualization.rerun_training import (  # noqa: E402
    TrainingRerunLogger)


def test_rerun_dependency_boundary():
    if importlib.util.find_spec("rerun") is None:
        try:
            _rerun()
        except RuntimeError as error:
            assert "pip install rerun-sdk" in str(error)
        else:
            raise AssertionError("missing Rerun dependency was not explained")
    else:
        assert _rerun().__name__ == "rerun"
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            episode = EpisodeData(
                qpos=np.zeros((1, 16), np.float32),
                qvel=np.zeros((1, 16), np.float32),
                images={"cam_high": np.zeros((1, 8, 8, 3), np.uint8)},
                action=np.zeros((1, 16), np.float32), debug={}, attrs={})
            log_episode(episode, root / "dataset.rrd")
            with TrainingRerunLogger(root / "training.rrd") as logger:
                logger.log_epoch({
                    "epoch": 0, "train/loss": 1.0, "val/loss": 0.9,
                    "train/l1": 0.5, "val/l1": 0.4,
                    "train/kl": 0.1, "val/kl": 0.1,
                    "train/pad": 0.2, "val/pad": 0.2,
                    "learning_rate": 1e-4,
                })
            observation = {
                "qpos": np.zeros(16),
                "images": {"cam_high": np.zeros((8, 8, 3), np.uint8)},
                "task": {"success": False, "object_position_error": 0.2},
            }
            with RolloutRerunLogger(
                    root / "rollout.rrd", ("cam_high",)) as logger:
                logger.log(
                    0, observation, np.zeros(16),
                    predicted_chunk=np.zeros((2, 16)))
            assert all((root / name).stat().st_size > 0 for name in (
                "dataset.rrd", "training.rrd", "rollout.rrd"))


if __name__ == "__main__":
    test_rerun_dependency_boundary()
    print("PASS")
