import pathlib
import sys
import tempfile

import numpy as np
import yaml

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.act.trainer import train  # noqa: E402
from ffw_sh5_grasp.imitation.dataset import EpisodeData, write_episode  # noqa: E402


def test_one_epoch_training_outputs():
    with tempfile.TemporaryDirectory() as directory:
        root = pathlib.Path(directory)
        dataset_dir = root / "datasets"
        images = {
            name: np.zeros((3, 32, 32, 3), np.uint8)
            for name in ("cam_high", "cam_left_wrist", "cam_right_wrist")
        }
        write_episode(dataset_dir / "episode_000000.hdf5", EpisodeData(
            qpos=np.zeros((3, 16), np.float32),
            qvel=np.zeros((3, 16), np.float32), images=images,
            action=np.zeros((3, 16), np.float32), debug={}, attrs={"seed": 1}))
        config = {
            "run_name": "smoke", "dataset_dir": str(dataset_dir),
            "output_dir": str(root / "outputs"),
            "camera_names": list(images), "state_dim": 16, "action_dim": 16,
            "chunk_size": 2, "hidden_dim": 32, "latent_dim": 8,
            "transformer_layers": 1, "attention_heads": 4, "dropout": 0.0,
            "batch_size": 3, "epochs": 1, "learning_rate": 1e-4,
            "weight_decay": 0.0, "kl_weight": 1.0,
            "validation_fraction": 0.1, "split_seed": 1,
            "test_fraction": 0.1,
            "num_workers": 0, "device": "cpu", "rerun": False,
        }
        config_path = root / "act.yaml"
        with config_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(config, stream)
        run_dir = train(config_path)
        expected = (
            "checkpoints/policy_best.ckpt", "checkpoints/policy_last.ckpt",
            "metrics/metrics.jsonl", "metrics/metrics.csv",
            "plots/loss.png", "plots/l1.png", "plots/kl.png",
            "plots/learning_rate.png", "dataset_stats.pkl", "config.yaml",
            "episode_splits.json",
        )
        assert all((run_dir / path).is_file() for path in expected)


if __name__ == "__main__":
    test_one_epoch_training_outputs()
    print("PASS")
