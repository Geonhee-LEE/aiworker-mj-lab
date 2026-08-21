"""Episode-level splits and ACT action-chunk construction."""

import pickle
from dataclasses import dataclass
from pathlib import Path

import h5py
import numpy as np
import torch
from torch.utils.data import Dataset

from ..data.schema import ACTION_DIM


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
    validation_count = (0 if validation_fraction == 0 else
                        max(1, round(len(paths) * validation_fraction)))
    test_count = (0 if test_fraction == 0 else
                  max(1, round(len(paths) * test_fraction)))
    if validation_count + test_count >= len(paths):
        test_count = 0 if len(paths) == 2 else 1
        validation_count = 1
    validation = [paths[index] for index in order[:validation_count]]
    test = [paths[index] for index in order[
        validation_count:validation_count + test_count]]
    train = [paths[index] for index in order[validation_count + test_count:]]
    return train, validation, test


def compute_stats(paths, *, qpos_indices=None, action_indices=None):
    """Compute train-only normalization without loading camera frames."""
    qpos_indices = _feature_indices(qpos_indices, "qpos_indices")
    action_indices = _feature_indices(action_indices, "action_indices")
    qpos = []
    actions = []
    for path in paths:
        with h5py.File(path, "r") as root:
            qpos.append(root["observations/qpos"][:, qpos_indices])
            actions.append(root["action"][:, action_indices])
    qpos = np.concatenate(qpos).astype(np.float32)
    actions = np.concatenate(actions).astype(np.float32)
    # This is the lower bound used by the released ACT data loader. A larger
    # floor prevents almost-constant joints from amplifying tiny sensor noise.
    epsilon = 1e-2
    return DatasetStats(
        qpos_mean=qpos.mean(0), qpos_std=np.maximum(qpos.std(0), epsilon),
        action_mean=actions.mean(0),
        action_std=np.maximum(actions.std(0), epsilon),
    )


def _feature_indices(indices, name):
    values = np.arange(ACTION_DIM) if indices is None else np.asarray(indices)
    if values.ndim != 1 or not len(values):
        raise ValueError(f"{name} must be a non-empty 1D sequence")
    if not np.issubdtype(values.dtype, np.integer):
        raise ValueError(f"{name} must contain integer indices")
    if np.any(values < 0) or np.any(values >= ACTION_DIM):
        raise ValueError(f"{name} must be within [0,{ACTION_DIM})")
    if len(np.unique(values)) != len(values):
        raise ValueError(f"{name} must not contain duplicates")
    return values.astype(int)


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
    """Sample one random timestep from each episode on every epoch.

    This follows the released ALOHA loader: an epoch visits every episode once,
    while repeated epochs expose different observation/action-chunk pairs.
    Camera tensors are sliced lazily so dataset growth does not consume several
    gigabytes of host memory.
    """

    def __init__(self, paths, stats, *, camera_names, chunk_size,
                 qpos_indices=None, action_indices=None):
        self.paths = tuple(Path(path) for path in paths)
        self.stats = stats
        self.camera_names = tuple(camera_names)
        self.chunk_size = int(chunk_size)
        if self.chunk_size <= 0:
            raise ValueError("chunk_size must be positive")
        self.qpos_indices = _feature_indices(qpos_indices, "qpos_indices")
        self.action_indices = _feature_indices(action_indices, "action_indices")
        if len(self.qpos_indices) != len(self.stats.qpos_mean):
            raise ValueError("qpos indices must match qpos statistics")
        if len(self.action_indices) != len(self.stats.action_mean):
            raise ValueError("action indices must match action statistics")
        self.episode_lengths = []
        for path in self.paths:
            with h5py.File(path, "r") as root:
                length = int(root["action"].shape[0])
                if root["observations/qpos"].shape != (length, ACTION_DIM):
                    raise ValueError(f"invalid qpos shape in {path}")
                missing = set(self.camera_names) - set(
                    root["observations/images"])
                if missing:
                    raise ValueError(
                        f"episode {path} is missing cameras: {sorted(missing)}")
                self.episode_lengths.append(length)

    def __len__(self):
        return len(self.paths)

    def __getitem__(self, index):
        episode_index = int(index)
        episode_path = self.paths[episode_index]
        episode_length = self.episode_lengths[episode_index]
        timestep = int(np.random.randint(episode_length))
        end = min(episode_length, timestep + self.chunk_size)
        with h5py.File(episode_path, "r") as root:
            qpos = root["observations/qpos"][timestep, self.qpos_indices]
            images = np.stack([
                root[f"observations/images/{name}"][timestep].transpose(2, 0, 1)
                for name in self.camera_names
            ]).astype(np.float32) / 255.0
            action_chunk = root["action"][timestep:end, self.action_indices]
        qpos = (qpos - self.stats.qpos_mean) / self.stats.qpos_std
        valid = end - timestep
        actions = np.zeros((self.chunk_size, len(self.action_indices)), dtype=np.float32)
        actions[:valid] = (
            action_chunk - self.stats.action_mean
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
