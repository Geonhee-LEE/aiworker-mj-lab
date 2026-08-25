"""In-memory transition recorder that guarantees obs/action timestep alignment."""

import subprocess
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ...paths import REPO_ROOT
from .episode import EpisodeData, next_episode_path, write_episode
from .schema import ACTION_DIM


def _git_commit(repository):
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=repository,
            check=True,
            capture_output=True,
            text=True,
        )
        return result.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return "unknown"


@dataclass
class EpisodeBuffer:
    qpos: list = field(default_factory=list)
    qvel: list = field(default_factory=list)
    ee_pose: dict[str, list] = field(default_factory=dict)
    images: dict[str, list] = field(default_factory=dict)
    actions: list = field(default_factory=list)
    debug: dict[str, list] = field(default_factory=dict)

    def append(self, observation, action):
        """Store precisely ``(obs_t, action_t)`` before the caller steps the env."""
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (ACTION_DIM,) or not np.all(np.isfinite(action)):
            raise ValueError(f"action must be finite shape ({ACTION_DIM},)")
        qpos = np.asarray(observation["qpos"], dtype=np.float32)
        qvel = np.asarray(observation["qvel"], dtype=np.float32)
        if qpos.shape != (ACTION_DIM,) or qvel.shape != (ACTION_DIM,):
            raise ValueError("observation policy state must be 16D")
        ee_pose = {
            name: np.asarray(values, dtype=np.float32)
            for name, values in observation["ee_pose"].items()
        }
        if set(ee_pose) != {"left", "right"} or any(
            values.shape != (7,) or not np.all(np.isfinite(values))
            for values in ee_pose.values()
        ):
            raise ValueError(
                "observation ee_pose must contain finite left/right 7D poses"
            )
        image_names = tuple(observation["images"])
        if self.images and set(image_names) != set(self.images):
            raise ValueError("camera set changed during the episode")
        self.qpos.append(qpos.copy())
        self.qvel.append(qvel.copy())
        for name, values in ee_pose.items():
            self.ee_pose.setdefault(name, []).append(values.copy())
        self.actions.append(action.copy())
        for name, image in observation["images"].items():
            self.images.setdefault(name, []).append(np.asarray(image).copy())
        for name, values in observation.get("debug", {}).items():
            self.debug.setdefault(name, []).append(np.asarray(values).copy())

    @property
    def length(self):
        return len(self.actions)

    def freeze(self, attrs=None):
        if not self.actions:
            raise ValueError("cannot save an empty episode")
        return EpisodeData(
            qpos=np.stack(self.qpos),
            qvel=np.stack(self.qvel),
            ee_pose={name: np.stack(values) for name, values in self.ee_pose.items()},
            images={name: np.stack(values) for name, values in self.images.items()},
            action=np.stack(self.actions),
            debug={name: np.stack(values) for name, values in self.debug.items()},
            attrs={} if attrs is None else dict(attrs),
        )


class EpisodeRecorder:
    """Manage start/finish/discard controls and canonical episode numbering."""

    def __init__(self, dataset_dir, env, *, task_name="can_to_box"):
        self.dataset_dir = Path(dataset_dir)
        self.env = env
        self.task_name = str(task_name)
        self.buffer = None
        self.dropped = 0

    @property
    def recording(self):
        return self.buffer is not None

    @property
    def frame(self):
        return 0 if self.buffer is None else self.buffer.length

    def start(self):
        if self.recording:
            raise RuntimeError("episode recording is already active")
        self.buffer = EpisodeBuffer()

    def record(self, observation, action):
        if not self.recording:
            return
        self.buffer.append(observation, action)

    def discard(self):
        if self.recording:
            self.buffer = None
            self.dropped += 1

    def finish(self, *, success=None, extra_attrs=None):
        if not self.recording:
            raise RuntimeError("no active episode recording")
        metrics = self.env.task.metrics(self.env.data)
        attrs = {
            "episode_id": int(
                next_episode_path(self.dataset_dir).stem.rsplit("_", 1)[1]
            ),
            "seed": -1 if self.env.last_seed is None else self.env.last_seed,
            "control_hz": self.env.actual_control_hz,
            "model_hash": self.env.model_hash,
            "git_commit": _git_commit(REPO_ROOT),
            "camera_names": list(self.env.camera_names),
            "ee_pose_names": ["left", "right"],
            "ee_pose_frame": "world",
            "ee_pose_quaternion_order": "wxyz",
            "task_name": self.task_name,
            "success": metrics.success if success is None else bool(success),
            "initial_can_position": self.env.initial_can_position.tolist(),
        }
        metadata = getattr(self.env.task, "episode_metadata", None)
        if metadata is not None:
            attrs.update(metadata())
        if extra_attrs:
            attrs.update(extra_attrs)
        path = next_episode_path(self.dataset_dir)
        episode = self.buffer.freeze(attrs)
        write_episode(path, episode)
        self.buffer = None
        return path


__all__ = ["EpisodeBuffer", "EpisodeRecorder"]
