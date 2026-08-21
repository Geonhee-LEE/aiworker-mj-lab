"""Checkpoint loading, action-chunk inference and temporal aggregation."""

from pathlib import Path

import numpy as np
import torch

from ..act.dataset_loader import load_stats
from ..act.policy import ACTPolicy, ACTPolicyConfig
from ..data.schema import ACTION_DIM


class TemporalAggregator:
    """Combine predictions for one target time using ACT's temporal ensemble.

    The paper indexes candidates from oldest to newest and assigns
    ``exp(-decay * index)``. Thus the oldest prediction has the greatest weight,
    dampening sudden changes while every observation still contributes.
    """

    def __init__(self, decay=0.01):
        self.decay = float(decay)
        if self.decay < 0:
            raise ValueError("temporal decay must be non-negative")
        self.predictions = {}

    def reset(self):
        self.predictions.clear()

    def add(self, source_timestep, chunk):
        chunk = np.asarray(chunk)
        if chunk.ndim != 2 or not len(chunk):
            raise ValueError("ACT chunk must be a non-empty [K,A] array")
        for offset, action in enumerate(chunk):
            target = int(source_timestep) + offset
            self.predictions.setdefault(target, []).append(
                (int(source_timestep), np.asarray(action, dtype=float).copy()))

    def action(self, timestep):
        candidates = self.predictions.pop(int(timestep), ())
        if not candidates:
            raise RuntimeError(f"no ACT prediction available for timestep {timestep}")
        candidates = sorted(candidates, key=lambda item: item[0])
        weights = np.exp(-self.decay * np.arange(len(candidates)))
        values = np.stack([action for _, action in candidates])
        return np.average(values, axis=0, weights=weights)


class ACTPolicyRunner:
    def __init__(self, checkpoint_path, stats_path=None, *, device="auto",
                 query_frequency=1, temporal_decay=0.01):
        checkpoint_path = Path(checkpoint_path)
        self.device = torch.device(
            "cuda" if device == "auto" and torch.cuda.is_available()
            else "cpu" if device == "auto" else device)
        checkpoint = torch.load(
            checkpoint_path, map_location=self.device, weights_only=False)
        try:
            self.config = ACTPolicyConfig(**checkpoint["policy_config"])
        except TypeError as error:
            raise ValueError(
                "checkpoint predates the paper-faithful ACT architecture; "
                "retrain it with the current config") from error
        self.policy = ACTPolicy(
            self.config, load_backbone_weights=False).to(self.device)
        self.policy.load_state_dict(checkpoint["model"])
        self.policy.eval()
        self.camera_names = tuple(checkpoint["camera_names"])
        self.policy_indices = np.asarray(
            checkpoint.get("policy_indices", tuple(range(ACTION_DIM))), dtype=int)
        if self.policy_indices.shape != (self.config.state_dim,):
            raise ValueError("checkpoint policy indices do not match state_dim")
        if self.config.action_dim != len(self.policy_indices):
            raise ValueError("checkpoint action_dim does not match policy indices")
        if stats_path is None:
            stats_path = checkpoint_path.parent.parent / "dataset_stats.pkl"
        self.stats = load_stats(stats_path)
        for name in ("qpos_mean", "qpos_std", "action_mean", "action_std"):
            expected = (self.config.state_dim if name.startswith("qpos")
                        else self.config.action_dim)
            if np.asarray(getattr(self.stats, name)).shape != (expected,):
                raise ValueError(
                    f"dataset statistic {name} does not match checkpoint")
        self.query_frequency = int(query_frequency)
        if self.query_frequency <= 0:
            raise ValueError("query_frequency must be positive")
        if self.query_frequency > self.config.chunk_size:
            raise ValueError("query_frequency cannot exceed ACT chunk_size")
        self.aggregator = TemporalAggregator(temporal_decay)
        self.reset()

    def reset(self):
        self.timestep = 0
        self.aggregator.reset()

    def _inputs(self, observation):
        full_qpos = np.asarray(observation["qpos"], dtype=np.float32)
        if full_qpos.shape != (ACTION_DIM,):
            raise ValueError(
                f"policy qpos must have shape ({ACTION_DIM},), "
                f"got {full_qpos.shape}")
        qpos = (
            full_qpos[self.policy_indices]
            - self.stats.qpos_mean
        ) / self.stats.qpos_std
        missing = set(self.camera_names) - set(observation["images"])
        if missing:
            raise ValueError(f"policy observation is missing cameras: {sorted(missing)}")
        camera_images = [np.asarray(observation["images"][name])
                         for name in self.camera_names]
        invalid = [
            name for name, image in zip(self.camera_names, camera_images)
            if image.ndim != 3 or image.shape[-1] != 3
        ]
        if invalid:
            raise ValueError(f"policy cameras must be HxWx3: {invalid}")
        if len({image.shape for image in camera_images}) != 1:
            raise ValueError("all policy cameras must share one image shape")
        images = np.stack([
            image.transpose(2, 0, 1) for image in camera_images
        ]).astype(np.float32) / 255.0
        return (
            torch.from_numpy(qpos)[None].to(self.device),
            torch.from_numpy(images)[None].to(self.device),
        )

    @torch.inference_mode()
    def predict_chunk(self, observation):
        qpos, images = self._inputs(observation)
        output = self.policy(qpos, images)
        normalized = output["actions"][0].cpu().numpy()
        predicted = normalized * self.stats.action_std + self.stats.action_mean
        chunk = np.tile(
            np.asarray(observation["qpos"], dtype=np.float32),
            (predicted.shape[0], 1))
        chunk[:, self.policy_indices] = predicted
        return chunk, output["is_pad"][0].sigmoid().cpu().numpy()

    def get_action(self, observation):
        predicted_chunk = None
        predicted_pad = None
        if self.timestep % self.query_frequency == 0:
            predicted_chunk, predicted_pad = self.predict_chunk(observation)
            self.aggregator.add(self.timestep, predicted_chunk)
        action = self.aggregator.action(self.timestep)
        info = {
            "timestep": self.timestep,
            "predicted_chunk": predicted_chunk,
            "predicted_pad": predicted_pad,
            "executed_action": action.copy(),
        }
        self.timestep += 1
        return action, info


__all__ = ["ACTPolicyRunner", "TemporalAggregator"]
