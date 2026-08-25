"""Episode splits, normalization and representation-aware ACT samples."""

import pickle
from dataclasses import dataclass
from pathlib import Path

import h5py
import numpy as np
import torch
from torch.utils.data import Dataset


@dataclass(frozen=True)
class DatasetStats:
    qpos_mean: np.ndarray
    qpos_std: np.ndarray
    action_mean: np.ndarray
    action_std: np.ndarray

    def as_dict(self):
        return {
            name: getattr(self, name)
            for name in ("qpos_mean", "qpos_std", "action_mean", "action_std")
        }


def episode_paths(dataset_dir, episode_count=None):
    paths = sorted(Path(dataset_dir).glob("episode_*.hdf5"))
    if not paths:
        raise FileNotFoundError(f"no episode_*.hdf5 files in {dataset_dir}")
    if episode_count is not None:
        episode_count = int(episode_count)
        if episode_count <= 0:
            raise ValueError("episode_count must be positive")
        if len(paths) < episode_count:
            raise ValueError(
                f"requested {episode_count} episodes from {dataset_dir}, "
                f"but only {len(paths)} are available"
            )
        paths = paths[:episode_count]
    return paths


def split_episodes(paths, validation_fraction=0.1, test_fraction=0.1, seed=42):
    paths = list(paths)
    if len(paths) == 1:
        # The explicit one-episode overfit gate intentionally reuses its sole
        # demonstration for all three views. Normal runs remain disjoint below.
        return paths, paths, paths
    if not (
        0.0 <= validation_fraction < 1.0
        and 0.0 <= test_fraction < 1.0
        and validation_fraction + test_fraction < 1.0
    ):
        raise ValueError(
            "validation/test fractions must be non-negative and sum below 1"
        )
    rng = np.random.default_rng(seed)
    order = rng.permutation(len(paths))
    validation_count = (
        0
        if validation_fraction == 0
        else max(1, round(len(paths) * validation_fraction))
    )
    test_count = 0 if test_fraction == 0 else max(1, round(len(paths) * test_fraction))
    if validation_count + test_count >= len(paths):
        test_count = 0 if len(paths) == 2 else 1
        validation_count = 1
    validation = [paths[index] for index in order[:validation_count]]
    test = [
        paths[index]
        for index in order[validation_count : validation_count + test_count]
    ]
    train = [paths[index] for index in order[validation_count + test_count :]]
    return train, validation, test


def compute_stats(paths, representation):
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
    """Sample aligned images and chunks in a selected representation.

    An epoch visits every episode once at a random timestep. Non-image policy
    features are cached, while camera tensors remain lazy HDF5 reads.
    """

    def __init__(self, paths, stats, *, representation, camera_names, chunk_size):
        self.paths = tuple(Path(path) for path in paths)
        self.stats = stats
        self.camera_names = tuple(camera_names)
        self.chunk_size = int(chunk_size)
        self.representation_name = representation.name
        if self.chunk_size <= 0:
            raise ValueError("chunk_size must be positive")
        self.features = tuple(
            representation.episode_features(path) for path in self.paths
        )
        for path, features in zip(self.paths, self.features):
            if features.state.shape[1] != len(self.stats.qpos_mean):
                raise ValueError(f"state statistics do not match {path}")
            if features.action.shape[1] != len(self.stats.action_mean):
                raise ValueError(f"action statistics do not match {path}")
            with h5py.File(path, "r") as root:
                missing = set(self.camera_names) - set(root["observations/images"])
                if missing:
                    raise ValueError(
                        f"episode {path} is missing cameras: {sorted(missing)}"
                    )
                for camera_name in self.camera_names:
                    if len(root[f"observations/images/{camera_name}"]) != len(
                        features.action
                    ):
                        raise ValueError(
                            f"camera {camera_name} is misaligned in {path}"
                        )

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
            images = (
                np.stack(
                    [
                        root[f"observations/images/{name}"][timestep].transpose(2, 0, 1)
                        for name in self.camera_names
                    ]
                ).astype(np.float32)
                / 255.0
            )

        state = (features.state[timestep] - self.stats.qpos_mean) / self.stats.qpos_std
        action_chunk = features.action[timestep:end]
        valid = end - timestep
        actions = np.zeros(
            (self.chunk_size, features.action.shape[1]), dtype=np.float32
        )
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


__all__ = [
    "ACTEpisodeDataset",
    "DatasetStats",
    "compute_stats",
    "episode_paths",
    "load_stats",
    "save_stats",
    "split_episodes",
]
