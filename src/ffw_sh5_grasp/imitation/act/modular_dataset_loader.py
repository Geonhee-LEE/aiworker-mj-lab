"""ACT dataset loader parameterized by a joint- or task-space adapter."""

from pathlib import Path

import h5py
import numpy as np
import torch
from torch.utils.data import Dataset

from .dataset_loader import DatasetStats


def compute_modular_stats(paths, representation):
    """Compute train-only statistics after representation conversion."""
    states = []
    actions = []
    for path in paths:
        features = representation.episode_features(path)
        states.append(features.state)
        actions.append(features.action)
    state = np.concatenate(states).astype(np.float32)
    action = np.concatenate(actions).astype(np.float32)
    epsilon = 1e-2
    return DatasetStats(
        qpos_mean=state.mean(0),
        qpos_std=np.maximum(state.std(0), epsilon),
        action_mean=action.mean(0),
        action_std=np.maximum(action.std(0), epsilon),
    )


class ModularACTEpisodeDataset(Dataset):
    """Sample aligned images and action chunks in a selected representation."""

    def __init__(self, paths, stats, *, representation, camera_names,
                 chunk_size):
        self.paths = tuple(Path(path) for path in paths)
        self.stats = stats
        self.camera_names = tuple(camera_names)
        self.chunk_size = int(chunk_size)
        self.representation_name = representation.name
        if self.chunk_size <= 0:
            raise ValueError("chunk_size must be positive")
        self.features = tuple(
            representation.episode_features(path) for path in self.paths)
        for path, features in zip(self.paths, self.features):
            if features.state.shape[1] != len(self.stats.qpos_mean):
                raise ValueError(
                    f"state statistics do not match {path}")
            if features.action.shape[1] != len(self.stats.action_mean):
                raise ValueError(
                    f"action statistics do not match {path}")
            with h5py.File(path, "r") as root:
                missing = set(self.camera_names) - set(
                    root["observations/images"])
                if missing:
                    raise ValueError(
                        f"episode {path} is missing cameras: {sorted(missing)}")
                for camera_name in self.camera_names:
                    if len(root[f"observations/images/{camera_name}"]) != len(
                            features.action):
                        raise ValueError(
                            f"camera {camera_name} is misaligned in {path}")

    def __len__(self):
        return len(self.paths)

    def __getitem__(self, index):
        episode_index = int(index)
        path = self.paths[episode_index]
        features = self.features[episode_index]
        episode_length = len(features.action)
        timestep = int(np.random.randint(episode_length))
        end = min(episode_length, timestep + self.chunk_size)

        with h5py.File(path, "r") as root:
            images = np.stack([
                root[f"observations/images/{name}"][timestep].transpose(
                    2, 0, 1)
                for name in self.camera_names
            ]).astype(np.float32) / 255.0

        state = (
            features.state[timestep] - self.stats.qpos_mean
        ) / self.stats.qpos_std
        action_chunk = features.action[timestep:end]
        valid = end - timestep
        actions = np.zeros(
            (self.chunk_size, features.action.shape[1]), dtype=np.float32)
        actions[:valid] = (
            action_chunk - self.stats.action_mean
        ) / self.stats.action_std
        is_pad = np.ones(self.chunk_size, dtype=bool)
        is_pad[:valid] = False
        return {
            "qpos": torch.from_numpy(state.astype(np.float32)),
            "images": torch.from_numpy(images),
            "actions": torch.from_numpy(actions),
            "is_pad": torch.from_numpy(is_pad),
        }


__all__ = ["ModularACTEpisodeDataset", "compute_modular_stats"]
