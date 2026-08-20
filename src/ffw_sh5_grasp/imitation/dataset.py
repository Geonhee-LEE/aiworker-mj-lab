"""ALOHA-compatible HDF5 episode I/O and validation."""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import tempfile

import h5py
import numpy as np

from .action import ACTION_DIM, ACTION_NAMES


SCHEMA_VERSION = "1.0"


@dataclass(frozen=True)
class EpisodeData:
    qpos: np.ndarray
    qvel: np.ndarray
    images: dict[str, np.ndarray]
    action: np.ndarray
    debug: dict[str, np.ndarray]
    attrs: dict

    @property
    def length(self):
        return int(self.action.shape[0])


def _decoded_attr(value):
    if isinstance(value, bytes):
        value = value.decode("utf-8")
    if isinstance(value, str) and value[:1] in ("[", "{"):
        try:
            return json.loads(value)
        except json.JSONDecodeError:
            pass
    return value.item() if isinstance(value, np.generic) else value


def validate_episode(episode):
    """Reject shifted, malformed or non-finite policy arrays before use."""
    lengths = {
        "qpos": len(episode.qpos), "qvel": len(episode.qvel),
        "action": len(episode.action),
        **{f"images/{name}": len(image)
           for name, image in episode.images.items()},
    }
    if not lengths or len(set(lengths.values())) != 1:
        raise ValueError(f"episode arrays are not aligned: {lengths}")
    if episode.qpos.ndim != 2 or episode.qpos.shape[1] != ACTION_DIM:
        raise ValueError(f"qpos must have shape [T,{ACTION_DIM}]")
    if episode.qvel.shape != episode.qpos.shape:
        raise ValueError("qvel shape must match qpos")
    if episode.action.shape != episode.qpos.shape:
        raise ValueError("action shape must match qpos")
    for name, values in (("qpos", episode.qpos), ("qvel", episode.qvel),
                         ("action", episode.action)):
        if not np.all(np.isfinite(values)):
            raise ValueError(f"{name} contains NaN or infinity")
    for name, image in episode.images.items():
        if image.ndim != 4 or image.shape[-1] != 3 or image.dtype != np.uint8:
            raise ValueError(f"camera {name} must be uint8 [T,H,W,3]")
    return lengths.get("action", 0)


def write_episode(path, episode, *, compression="gzip"):
    """Atomically write one validated episode in canonical ALOHA layout."""
    validate_episode(episode)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    os.close(descriptor)
    try:
        with h5py.File(temporary, "w") as root:
            observations = root.create_group("observations")
            observations.create_dataset("qpos", data=episode.qpos)
            observations.create_dataset("qvel", data=episode.qvel)
            image_group = observations.create_group("images")
            for name, values in episode.images.items():
                image_group.create_dataset(
                    name, data=values, compression=compression,
                    chunks=(1, *values.shape[1:]))
            root.create_dataset("action", data=episode.action)
            if episode.debug:
                debug_group = root.create_group("debug")
                for name, values in episode.debug.items():
                    debug_group.create_dataset(name, data=values)
            attrs = {"sim": True, "schema_version": SCHEMA_VERSION,
                     "action_names": list(ACTION_NAMES), **episode.attrs}
            for name, value in attrs.items():
                root.attrs[name] = (json.dumps(value) if isinstance(
                    value, (list, tuple, dict)) else value)
            root.flush()
        os.replace(temporary, path)
    except BaseException:
        if os.path.exists(temporary):
            os.unlink(temporary)
        raise
    return path


def load_episode(path, *, validate=True):
    path = Path(path)
    with h5py.File(path, "r") as root:
        images = {
            name: dataset[:]
            for name, dataset in root["observations/images"].items()
        }
        debug = ({name: dataset[:] for name, dataset in root["debug"].items()}
                 if "debug" in root else {})
        result = EpisodeData(
            qpos=root["observations/qpos"][:],
            qvel=root["observations/qvel"][:],
            images=images,
            action=root["action"][:],
            debug=debug,
            attrs={name: _decoded_attr(value)
                   for name, value in root.attrs.items()},
        )
    if validate:
        validate_episode(result)
    return result


def next_episode_path(dataset_dir):
    dataset_dir = Path(dataset_dir)
    existing = sorted(dataset_dir.glob("episode_*.hdf5"))
    indices = []
    for path in existing:
        try:
            indices.append(int(path.stem.rsplit("_", 1)[1]))
        except (IndexError, ValueError):
            continue
    return dataset_dir / f"episode_{max(indices, default=-1) + 1:06d}.hdf5"


__all__ = [
    "EpisodeData", "SCHEMA_VERSION", "load_episode", "next_episode_path",
    "validate_episode", "write_episode",
]
