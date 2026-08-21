import pathlib
import sys
import tempfile

import numpy as np
import torch

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.act.dataset_loader import (
    DatasetStats,
    save_stats,
)
from ffw_sh5_grasp.imitation.act.policy import ACTPolicy, ACTPolicyConfig
from ffw_sh5_grasp.imitation.runtime.runner import (
    ACTPolicyRunner,
    TemporalAggregator,
)


def test_act_shapes_and_loss():
    config = ACTPolicyConfig(
        chunk_size=4, hidden_dim=32, latent_dim=8,
        encoder_layers=1, decoder_layers=1, feedforward_dim=64,
        attention_heads=4, camera_count=3, pretrained_backbone=False)
    policy = ACTPolicy(config, load_backbone_weights=False)
    batch = {
        "qpos": torch.zeros(2, 16),
        "images": torch.rand(2, 3, 3, 32, 32),
        "actions": torch.rand(2, 4, 16),
        "is_pad": torch.tensor([[False, False, False, True],
                                [False, False, True, True]]),
    }
    output = policy(batch["qpos"], batch["images"])
    assert output["actions"].shape == (2, 4, 16)
    losses = policy.loss(batch)
    assert set(losses) == {"loss", "l1", "kl"}
    assert all(torch.isfinite(value) for value in losses.values())
    losses["loss"].backward()


def test_checkpoint_runner_and_temporal_aggregation():
    config = ACTPolicyConfig(
        chunk_size=3, hidden_dim=32, latent_dim=8,
        encoder_layers=1, decoder_layers=1, feedforward_dim=64,
        attention_heads=4, camera_count=3, pretrained_backbone=False)
    policy = ACTPolicy(config, load_backbone_weights=False)
    stats = DatasetStats(
        qpos_mean=np.zeros(16, np.float32), qpos_std=np.ones(16, np.float32),
        action_mean=np.zeros(16, np.float32), action_std=np.ones(16, np.float32))
    with tempfile.TemporaryDirectory() as directory:
        checkpoint = pathlib.Path(directory) / "checkpoints/policy.ckpt"
        checkpoint.parent.mkdir()
        torch.save({
            "model": policy.state_dict(), "policy_config": config.as_dict(),
            "camera_names": ["a", "b", "c"]}, checkpoint)
        stats_path = pathlib.Path(directory) / "dataset_stats.pkl"
        save_stats(stats, stats_path)
        runner = ACTPolicyRunner(checkpoint, stats_path, device="cpu")
        observation = {
            "qpos": np.zeros(16, np.float32),
            "images": {name: np.zeros((32, 32, 3), np.uint8)
                       for name in ("a", "b", "c")},
        }
        action, info = runner.get_action(observation)
        assert action.shape == (16,)
        assert info["predicted_chunk"].shape == (3, 16)
    aggregator = TemporalAggregator(decay=0.0)
    aggregator.add(0, np.stack((np.zeros(16), np.ones(16))))
    aggregator.add(1, np.stack((np.full(16, 3.0), np.full(16, 4.0))))
    assert np.allclose(aggregator.action(1), 2.0)


def test_temporal_aggregation_prioritizes_older_predictions():
    aggregator = TemporalAggregator(decay=1.0)
    aggregator.add(0, np.stack((np.zeros(16), np.ones(16))))
    aggregator.add(1, np.stack((np.full(16, 10.0), np.full(16, 20.0))))
    action = aggregator.action(1)
    assert np.all(action < 5.0)
    assert 1 not in aggregator.predictions


if __name__ == "__main__":
    test_act_shapes_and_loss()
    test_checkpoint_runner_and_temporal_aggregation()
    test_temporal_aggregation_prioritizes_older_predictions()
    print("PASS")
