"""Checkpoint loading, action-chunk inference and temporal aggregation."""

from pathlib import Path

import numpy as np
import torch

from .act.dataset_loader import load_stats
from .act.policy import ACTPolicy, ACTPolicyConfig


class TemporalAggregator:
    """Combine overlapping ACT predictions for the current execution timestep."""

    def __init__(self, decay=0.05):
        self.decay = float(decay)
        self.predictions = {}

    def reset(self):
        self.predictions.clear()

    def add(self, source_timestep, chunk):
        for offset, action in enumerate(np.asarray(chunk)):
            target = int(source_timestep) + offset
            self.predictions.setdefault(target, []).append(
                (int(source_timestep), np.asarray(action, dtype=float).copy()))

    def action(self, timestep):
        candidates = self.predictions.get(int(timestep), ())
        if not candidates:
            raise RuntimeError(f"no ACT prediction available for timestep {timestep}")
        ages = np.asarray([timestep - source for source, _ in candidates], dtype=float)
        weights = np.exp(-self.decay * ages)
        values = np.stack([action for _, action in candidates])
        return np.average(values, axis=0, weights=weights)


class ACTPolicyRunner:
    def __init__(self, checkpoint_path, stats_path=None, *, device="auto",
                 query_frequency=1, temporal_decay=0.05):
        checkpoint_path = Path(checkpoint_path)
        self.device = torch.device(
            "cuda" if device == "auto" and torch.cuda.is_available()
            else "cpu" if device == "auto" else device)
        checkpoint = torch.load(
            checkpoint_path, map_location=self.device, weights_only=False)
        self.config = ACTPolicyConfig(**checkpoint["policy_config"])
        self.policy = ACTPolicy(self.config).to(self.device)
        self.policy.load_state_dict(checkpoint["model"])
        self.policy.eval()
        self.camera_names = tuple(checkpoint["camera_names"])
        if stats_path is None:
            stats_path = checkpoint_path.parent.parent / "dataset_stats.pkl"
        self.stats = load_stats(stats_path)
        self.query_frequency = int(query_frequency)
        if self.query_frequency <= 0:
            raise ValueError("query_frequency must be positive")
        self.aggregator = TemporalAggregator(temporal_decay)
        self.reset()

    def reset(self):
        self.timestep = 0
        self.aggregator.reset()

    def _inputs(self, observation):
        qpos = (
            np.asarray(observation["qpos"], dtype=np.float32)
            - self.stats.qpos_mean
        ) / self.stats.qpos_std
        missing = set(self.camera_names) - set(observation["images"])
        if missing:
            raise ValueError(f"policy observation is missing cameras: {sorted(missing)}")
        images = np.stack([
            observation["images"][name].transpose(2, 0, 1)
            for name in self.camera_names
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
        chunk = normalized * self.stats.action_std + self.stats.action_mean
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
