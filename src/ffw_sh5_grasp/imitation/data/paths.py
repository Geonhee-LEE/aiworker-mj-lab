"""Path rules shared by ALOHA-style episode command-line tools."""

from pathlib import Path


def resolve_episode_path(episode=None, dataset_dir=None, episode_idx=0):
    """Resolve an explicit episode or the canonical dataset/index path."""
    if episode is not None:
        return Path(episode)
    if dataset_dir is None:
        raise ValueError("provide --episode or --dataset-dir")
    return Path(dataset_dir) / f"episode_{int(episode_idx):06d}.hdf5"


__all__ = ["resolve_episode_path"]
