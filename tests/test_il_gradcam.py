from types import SimpleNamespace

import numpy as np
import torch

from ffw_sh5_grasp.imitation.act.policy import ACTPolicy, ACTPolicyConfig
from ffw_sh5_grasp.imitation.visualization.gradcam import compute_gradcam


def _runner():
    config = ACTPolicyConfig(
        state_dim=8,
        action_dim=8,
        chunk_size=3,
        camera_count=2,
        hidden_dim=32,
        latent_dim=8,
        encoder_layers=1,
        decoder_layers=1,
        feedforward_dim=64,
        attention_heads=4,
        dropout=0.0,
        pretrained_backbone=False,
    )
    policy = ACTPolicy(config, load_backbone_weights=False).eval()
    qpos = torch.zeros((1, 8), dtype=torch.float32)
    images = torch.rand((1, 2, 3, 64, 64), dtype=torch.float32)
    return SimpleNamespace(
        policy=policy,
        config=config,
        stats=SimpleNamespace(
            action_mean=np.zeros(8, np.float32),
            action_std=np.ones(8, np.float32),
        ),
        _inputs=lambda _observation: (qpos, images),
    )


def test_gradcam_returns_one_finite_map_per_camera():
    runner = _runner()
    heatmaps, predicted, score, raw_max, gradient_mean = compute_gradcam(
        runner, {}, target="chunk", chunk_step=1
    )
    assert heatmaps.shape == (2, 64, 64)
    assert predicted.shape == (3, 8)
    assert np.all(np.isfinite(heatmaps))
    assert np.all((0.0 <= heatmaps) & (heatmaps <= 1.0))
    assert np.all(np.isfinite(predicted))
    assert np.isfinite(score)
    assert raw_max.shape == (2,)
    assert gradient_mean.shape == (2,)
    assert np.all(np.isfinite(raw_max))
    assert np.all(np.isfinite(gradient_mean))


def test_gradcam_can_target_one_action_scalar():
    runner = _runner()
    heatmaps, predicted, score, raw_max, gradient_mean = compute_gradcam(
        runner, {}, target="action", chunk_step=2, action_index=1, target_sign=-1.0
    )
    assert heatmaps.shape == (2, 64, 64)
    assert predicted.shape == (3, 8)
    assert np.isfinite(score)
    assert np.all(raw_max >= 0.0)
    assert np.all(gradient_mean >= 0.0)


def test_gradcam_can_target_a_linear_action_direction():
    runner = _runner()
    weights = np.zeros(8, dtype=np.float32)
    weights[:3] = (0.5, -1.0, 0.25)
    heatmaps, predicted, score, raw_max, gradient_mean = compute_gradcam(
        runner, {}, target="linear", chunk_step=2, action_weights=weights
    )
    assert heatmaps.shape == (2, 64, 64)
    assert predicted.shape == (3, 8)
    assert np.isfinite(score)
    assert np.all(np.isfinite(raw_max))
    assert np.all(np.isfinite(gradient_mean))
