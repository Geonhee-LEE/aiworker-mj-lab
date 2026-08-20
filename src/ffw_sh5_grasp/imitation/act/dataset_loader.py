"""Episode-level splits and ACT action-chunk construction."""

from dataclasses import dataclass
from pathlib import Path
import pickle

import numpy as np
import torch
from torch.utils.data import Dataset

from ..action import ACTION_DIM
from ..dataset import load_episode


@dataclass(frozen=True)
class DatasetStats:
    qpos_mean: np.ndarray
    qpos_std: np.ndarray
    action_mean: np.ndarray
    action_std: np.ndarray

    def as_dict(self):
        return {name: getattr(self, name) for name in (
            "qpos_mean", "qpos_std", "action_mean", "action_std")}


def episode_paths(dataset_dir):
    paths = sorted(Path(dataset_dir).glob("episode_*.hdf5"))
    if not paths:
        raise FileNotFoundError(f"no episode_*.hdf5 files in {dataset_dir}")
    return paths


def split_episodes(paths, validation_fraction=0.1, test_fraction=0.1, seed=42):
    paths = list(paths)
    if len(paths) == 1:
        # The explicit one-episode overfit gate intentionally reuses its sole
        # demonstration for all three views. Normal runs remain disjoint below.
        return paths, paths, paths
    if not (0.0 <= validation_fraction < 1.0
            and 0.0 <= test_fraction < 1.0
            and validation_fraction + test_fraction < 1.0):
        raise ValueError("validation/test fractions must be non-negative and sum below 1")
    rng = np.random.default_rng(seed)
    order = rng.permutation(len(paths))
    validation_count = max(1, round(len(paths) * validation_fraction))
    test_count = max(1, round(len(paths) * test_fraction))
    if validation_count + test_count >= len(paths):
        test_count = 0 if len(paths) == 2 else 1
        validation_count = 1
    validation = [paths[index] for index in order[:validation_count]]
    test = [paths[index] for index in order[
        validation_count:validation_count + test_count]]
    train = [paths[index] for index in order[validation_count + test_count:]]
    return train, validation, test


def compute_stats(paths):
    qpos = []
    actions = []
    for path in paths:
        episode = load_episode(path)
        qpos.append(episode.qpos)
        actions.append(episode.action)
    qpos = np.concatenate(qpos).astype(np.float32)
    actions = np.concatenate(actions).astype(np.float32)
    epsilon = 1e-6
    return DatasetStats(
        qpos_mean=qpos.mean(0), qpos_std=np.maximum(qpos.std(0), epsilon),
        action_mean=actions.mean(0),
        action_std=np.maximum(actions.std(0), epsilon),
    )


def save_stats(stats, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        pickle.dump(stats.as_dict(), stream)


def load_stats(path):
    with Path(path).open("rb") as stream:
        values = pickle.load(stream)
    return DatasetStats(**values)


class ACTEpisodeDataset(Dataset):
    """One item per episode timestep with a padded future action chunk."""

    def __init__(self, paths, stats, *, camera_names, chunk_size):
        self.paths = tuple(Path(path) for path in paths)
        self.stats = stats
        self.camera_names = tuple(camera_names)
        self.chunk_size = int(chunk_size)
        if self.chunk_size <= 0:
            raise ValueError("chunk_size must be positive")
        self.episodes = [load_episode(path) for path in self.paths]
        self.index = [
            (episode_index, timestep)
            for episode_index, episode in enumerate(self.episodes)
            for timestep in range(episode.length)
        ]
        for episode in self.episodes:
            missing = set(self.camera_names) - set(episode.images)
            if missing:
                raise ValueError(f"episode is missing cameras: {sorted(missing)}")

    def __len__(self):
        return len(self.index)

    def __getitem__(self, index):
        episode_index, timestep = self.index[index]
        episode = self.episodes[episode_index]
        qpos = (episode.qpos[timestep] - self.stats.qpos_mean) / self.stats.qpos_std
        images = np.stack([
            episode.images[name][timestep].transpose(2, 0, 1)
            for name in self.camera_names
        ]).astype(np.float32) / 255.0
        end = min(episode.length, timestep + self.chunk_size)
        valid = end - timestep
        actions = np.zeros((self.chunk_size, ACTION_DIM), dtype=np.float32)
        actions[:valid] = (
            episode.action[timestep:end] - self.stats.action_mean
        ) / self.stats.action_std
        is_pad = np.ones(self.chunk_size, dtype=bool)
        is_pad[:valid] = False
        return {
            "qpos": torch.from_numpy(qpos.astype(np.float32)),
            "images": torch.from_numpy(images),
            "actions": torch.from_numpy(actions),
            "is_pad": torch.from_numpy(is_pad),
        }


__all__ = [
    "ACTEpisodeDataset", "DatasetStats", "compute_stats", "episode_paths",
    "load_stats", "save_stats", "split_episodes",
]
