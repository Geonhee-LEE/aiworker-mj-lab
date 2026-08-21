"""Fast structural checks for a directory of recorded HDF5 episodes."""

from dataclasses import asdict, dataclass
from pathlib import Path

import h5py
import numpy as np

from .episode import SCHEMA_VERSION
from .schema import ACTION_DIM


@dataclass(frozen=True)
class EpisodeInspection:
    path: str
    frames: int
    success: bool
    camera_names: tuple[str, ...]
    errors: tuple[str, ...]


@dataclass(frozen=True)
class DatasetInspection:
    dataset_dir: str
    episodes: tuple[EpisodeInspection, ...]

    @property
    def valid(self):
        return bool(self.episodes) and not any(
            episode.errors for episode in self.episodes)

    def as_dict(self):
        return {
            "dataset_dir": self.dataset_dir,
            "valid": self.valid,
            "episode_count": len(self.episodes),
            "success_count": sum(item.success for item in self.episodes),
            "total_frames": sum(item.frames for item in self.episodes),
            "episodes": [asdict(item) for item in self.episodes],
        }


def _finite(dataset, block_size=1024):
    for start in range(0, len(dataset), block_size):
        if not np.all(np.isfinite(dataset[start:start + block_size])):
            return False
    return True


def inspect_episode(path, *, required_cameras=()):
    """Inspect shapes, dtypes and finite values without loading RGB into RAM."""
    path = Path(path)
    errors = []
    frames = 0
    success = False
    camera_names = ()
    try:
        with h5py.File(path, "r") as root:
            required_paths = (
                "observations/qpos", "observations/qvel",
                "observations/images", "action")
            missing_paths = [name for name in required_paths if name not in root]
            if missing_paths:
                errors.append(f"missing datasets: {missing_paths}")
                return EpisodeInspection(
                    str(path), frames, success, camera_names, tuple(errors))
            action = root["action"]
            qpos = root["observations/qpos"]
            qvel = root["observations/qvel"]
            images = root["observations/images"]
            frames = int(action.shape[0])
            success = bool(root.attrs.get("success", False))
            camera_names = tuple(sorted(images.keys()))
            expected_shape = (frames, ACTION_DIM)
            for name, dataset in (
                    ("qpos", qpos), ("qvel", qvel), ("action", action)):
                if dataset.shape != expected_shape:
                    errors.append(
                        f"{name} shape {dataset.shape}, expected {expected_shape}")
                elif not _finite(dataset):
                    errors.append(f"{name} contains NaN or infinity")
            missing_cameras = set(required_cameras) - set(camera_names)
            if missing_cameras:
                errors.append(
                    f"missing required cameras: {sorted(missing_cameras)}")
            for name, image in images.items():
                if (image.ndim != 4 or image.shape[0] != frames
                        or image.shape[-1] != 3 or image.dtype != np.uint8):
                    errors.append(
                        f"camera {name} must be uint8 [T,H,W,3], "
                        f"got {image.shape} {image.dtype}")
            schema = str(root.attrs.get("schema_version", "missing"))
            if schema != SCHEMA_VERSION:
                errors.append(
                    f"schema_version is {schema}, expected {SCHEMA_VERSION}")
    except (OSError, KeyError, ValueError) as error:
        errors.append(str(error))
    return EpisodeInspection(
        str(path), frames, success, camera_names, tuple(errors))


def inspect_dataset(dataset_dir, *, required_cameras=()):
    dataset_dir = Path(dataset_dir)
    episodes = tuple(
        inspect_episode(path, required_cameras=required_cameras)
        for path in sorted(dataset_dir.glob("episode_*.hdf5"))
    )
    return DatasetInspection(str(dataset_dir), episodes)


__all__ = [
    "DatasetInspection", "EpisodeInspection", "inspect_dataset",
    "inspect_episode",
]
