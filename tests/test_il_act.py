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
    TaskSpaceTemporalAggregator,
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


def test_temporal_aggregation_prioritizes_newer_predictions():
    aggregator = TemporalAggregator(decay=1.0)
    aggregator.add(0, np.stack((np.zeros(16), np.ones(16))))
    aggregator.add(1, np.stack((np.full(16, 10.0), np.full(16, 20.0))))
    action = aggregator.action(1)
    assert np.all(action > 5.0)
    assert 1 not in aggregator.predictions


def test_proleptic_aggregation_selects_future_column_and_discards_skipped():
    aggregator = TemporalAggregator(decay=0.0)
    aggregator.add(0, np.stack([
        np.full(16, value) for value in (0.0, 1.0, 2.0, 3.0)]))
    aggregator.add(1, np.stack([
        np.full(16, value) for value in (10.0, 11.0, 12.0, 13.0)]))

    # At execution t=1 with f=2, target t+f=3 receives predictions 3 and 12.
    assert np.allclose(aggregator.action(3), 7.5)
    assert aggregator.last_candidate_count == 2
    assert all(timestep > 3 for timestep in aggregator.predictions)


def test_task_checkpoint_uses_right_ee_pose_and_quaternion_ensemble():
    config = ACTPolicyConfig(
        state_dim=8, action_dim=8, chunk_size=3, camera_count=1,
        hidden_dim=32, latent_dim=8, encoder_layers=1, decoder_layers=1,
        feedforward_dim=64, attention_heads=4,
        pretrained_backbone=False)
    policy = ACTPolicy(config, load_backbone_weights=False)
    stats = DatasetStats(
        qpos_mean=np.zeros(8, np.float32),
        qpos_std=np.ones(8, np.float32),
        action_mean=np.zeros(8, np.float32),
        action_std=np.ones(8, np.float32))
    with tempfile.TemporaryDirectory() as directory:
        checkpoint = pathlib.Path(directory) / "checkpoints/policy.ckpt"
        checkpoint.parent.mkdir()
        torch.save({
            "model": policy.state_dict(),
            "policy_config": config.as_dict(),
            "camera_names": ["cam"],
            "representation": "task",
            "representation_metadata": {
                "ee_pose_frame": "world",
                "ee_pose_quaternion_order": "wxyz",
            },
        }, checkpoint)
        stats_path = pathlib.Path(directory) / "dataset_stats.pkl"
        save_stats(stats, stats_path)
        runner = ACTPolicyRunner(
            checkpoint, stats_path, device="cpu", representation="auto",
            proleptic_steps=2)
        observation = {
            "qpos": np.arange(16, dtype=np.float32) / 15.0,
            "ee_pose": {
                "right": np.array(
                    [0.4, -0.1, 0.9, 2.0, 0.0, 0.0, 0.0],
                    dtype=np.float32),
            },
            "images": {"cam": np.zeros((32, 32, 3), np.uint8)},
        }
        state, _images = runner._inputs(observation)
        assert runner.representation == "task"
        assert np.allclose(
            state[0].numpy(),
            [0.4, -0.1, 0.9, 1.0, 0.0, 0.0, 0.0, 1.0])
        action, info = runner.get_action(observation)
        assert action.shape == (8,)
        assert info["predicted_chunk"].shape == (3, 8)
        assert info["target_timestep"] == 2
        assert info["proleptic_steps"] == 2
        assert info["ensemble_candidate_count"] == 1
        assert np.isclose(np.linalg.norm(action[3:7]), 1.0)

        runner.set_proleptic_steps(1)
        assert runner._force_query
        assert not runner.aggregator.predictions
        _action, changed_info = runner.get_action(observation)
        assert changed_info["timestep"] == 1
        assert changed_info["target_timestep"] == 2
        assert changed_info["predicted_chunk"] is not None

        try:
            runner.set_proleptic_steps(3)
        except ValueError as error:
            assert "between 0 and 2" in str(error)
        else:
            raise AssertionError("PTE look-ahead beyond chunk coverage accepted")

        try:
            ACTPolicyRunner(
                checkpoint, stats_path, device="cpu",
                representation="joint")
        except ValueError as error:
            assert "does not match" in str(error)
        else:
            raise AssertionError("representation mismatch must be rejected")

    aggregator = TaskSpaceTemporalAggregator(decay=0.0)
    first = np.array([0.4, 0.0, 0.9, 1.0, 0.0, 0.0, 0.0, 0.2])
    antipodal = first.copy()
    antipodal[3:7] *= -1.0
    aggregator.add(0, np.stack((np.zeros(8), first)))
    aggregator.add(1, np.stack((antipodal, np.ones(8))))
    averaged = aggregator.action(1)
    assert np.allclose(averaged[3:7], [1.0, 0.0, 0.0, 0.0])


if __name__ == "__main__":
    test_act_shapes_and_loss()
    test_checkpoint_runner_and_temporal_aggregation()
    test_temporal_aggregation_prioritizes_newer_predictions()
    test_proleptic_aggregation_selects_future_column_and_discards_skipped()
    test_task_checkpoint_uses_right_ee_pose_and_quaternion_ensemble()
    print("PASS")
