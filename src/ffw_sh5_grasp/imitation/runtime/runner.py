"""Checkpoint loading, action-chunk inference and temporal aggregation."""

from pathlib import Path

import numpy as np
import torch

from ...kinematics.rotations import normalize_quaternion
from ..act.dataset_loader import load_stats
from ..act.policy import ACTPolicy, ACTPolicyConfig
from ..data.schema import ACTION_DIM

POLICY_REPRESENTATIONS = ("auto", "joint", "task")
RIGHT_GRASP_INDEX = 15


class TemporalAggregator:
    """Combine predictions for one target time using ACT's temporal ensemble.

    PTE equations (2)--(4) index candidates from the newest inference to the
    oldest and assign ``exp(-decay * index)``.  Thus recent observations carry
    the greatest weight while every still-covered chunk contributes.
    """

    def __init__(self, decay=0.05):
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

    def _take_candidates(self, target_timestep):
        """Take one target column and discard columns skipped by PTE."""
        target_timestep = int(target_timestep)
        for stale_timestep in tuple(self.predictions):
            if stale_timestep < target_timestep:
                del self.predictions[stale_timestep]
        candidates = self.predictions.pop(target_timestep, ())
        if not candidates:
            raise RuntimeError(
                f"no ACT prediction available for timestep {target_timestep}")
        return sorted(candidates, key=lambda item: item[0], reverse=True)

    def action(self, timestep):
        candidates = self._take_candidates(timestep)
        self.last_candidate_count = len(candidates)
        weights = np.exp(-self.decay * np.arange(len(candidates)))
        values = np.stack([action for _, action in candidates])
        return np.average(values, axis=0, weights=weights)


class TaskSpaceTemporalAggregator(TemporalAggregator):
    """Temporal ensemble with sign-safe averaging for wxyz quaternions."""

    def action(self, timestep):
        candidates = self._take_candidates(timestep)
        self.last_candidate_count = len(candidates)
        weights = np.exp(-self.decay * np.arange(len(candidates)))
        values = np.stack([action for _, action in candidates])
        result = np.average(values, axis=0, weights=weights)

        quaternions = np.stack([
            normalize_quaternion(value[3:7]) for value in values
        ])
        reference = quaternions[0]
        quaternions[np.sum(quaternions * reference, axis=1) < 0.0] *= -1.0
        result[3:7] = normalize_quaternion(
            np.average(quaternions, axis=0, weights=weights))
        return result


class ACTPolicyRunner:
    def __init__(self, checkpoint_path, stats_path=None, *, device="auto",
                 query_frequency=1, temporal_decay=0.05,
                 representation="auto", proleptic_steps=0):
        checkpoint_path = Path(checkpoint_path)
        self.device = torch.device(
            "cuda" if device == "auto" and torch.cuda.is_available()
            else "cpu" if device == "auto" else device)
        checkpoint = torch.load(
            checkpoint_path, map_location=self.device, weights_only=False)
        requested_representation = str(representation).strip().lower()
        if requested_representation not in POLICY_REPRESENTATIONS:
            raise ValueError(
                "policy representation must be auto, joint, or task")
        checkpoint_representation = str(
            checkpoint.get("representation", "joint")).strip().lower()
        if checkpoint_representation not in ("joint", "task"):
            raise ValueError(
                f"unsupported checkpoint representation: "
                f"{checkpoint_representation!r}")
        if (requested_representation != "auto"
                and requested_representation != checkpoint_representation):
            raise ValueError(
                f"requested {requested_representation!r} representation "
                f"does not match {checkpoint_representation!r} checkpoint")
        self.representation = checkpoint_representation
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
        self.policy_indices = None
        if self.representation == "joint":
            self.policy_indices = np.asarray(
                checkpoint.get(
                    "policy_indices", tuple(range(ACTION_DIM))), dtype=int)
            if self.policy_indices.shape != (self.config.state_dim,):
                raise ValueError(
                    "checkpoint policy indices do not match state_dim")
            if self.config.action_dim != len(self.policy_indices):
                raise ValueError(
                    "checkpoint action_dim does not match policy indices")
        elif self.config.state_dim != 8 or self.config.action_dim != 8:
            raise ValueError(
                "task policy must use 8D EE pose plus grasp state/action")
        else:
            metadata = checkpoint.get("representation_metadata", {})
            if (metadata.get("ee_pose_frame") != "world"
                    or metadata.get("ee_pose_quaternion_order") != "wxyz"):
                raise ValueError(
                    "task checkpoint must declare world-frame wxyz EE poses")
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
        self.proleptic_steps = 0
        self._force_query = False
        self.aggregator = (
            TaskSpaceTemporalAggregator(temporal_decay)
            if self.representation == "task"
            else TemporalAggregator(temporal_decay))
        self.set_proleptic_steps(proleptic_steps)
        self.reset()

    def reset(self):
        self.timestep = 0
        self.aggregator.reset()
        self._force_query = False

    @property
    def max_proleptic_steps(self):
        """Largest look-ahead that remains covered between policy queries."""
        return self.config.chunk_size - self.query_frequency

    def set_proleptic_steps(self, steps):
        """Select a future action column and reset incompatible candidates."""
        if isinstance(steps, bool) or int(steps) != steps:
            raise ValueError("PTE steps must be an integer")
        steps = int(steps)
        if not 0 <= steps <= self.max_proleptic_steps:
            raise ValueError(
                "PTE steps must be between 0 and "
                f"{self.max_proleptic_steps} for this checkpoint")
        changed = steps != self.proleptic_steps
        self.proleptic_steps = steps
        if changed:
            self.aggregator.reset()
            # A UI change may happen between scheduled queries. Guarantee that
            # the next control frame has a chunk covering t + f.
            self._force_query = True

    def _inputs(self, observation):
        full_qpos = np.asarray(observation["qpos"], dtype=np.float32)
        if full_qpos.shape != (ACTION_DIM,):
            raise ValueError(
                f"policy qpos must have shape ({ACTION_DIM},), "
                f"got {full_qpos.shape}")
        if self.representation == "joint":
            state = full_qpos[self.policy_indices]
        else:
            try:
                right_pose = np.asarray(
                    observation["ee_pose"]["right"], dtype=np.float32)
            except KeyError as error:
                raise ValueError(
                    "task policy observation requires ee_pose/right") from error
            if right_pose.shape != (7,) or not np.all(np.isfinite(right_pose)):
                raise ValueError(
                    "task policy right EE pose must be a finite 7D vector")
            right_pose = right_pose.copy()
            right_pose[3:7] = normalize_quaternion(right_pose[3:7])
            state = np.concatenate((
                right_pose, full_qpos[[RIGHT_GRASP_INDEX]],
            ))
        qpos = (state - self.stats.qpos_mean) / self.stats.qpos_std
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
        if self.representation == "task":
            if not np.all(np.isfinite(predicted)):
                raise ValueError("task policy predicted NaN or infinity")
            predicted[:, 3:7] = np.stack([
                normalize_quaternion(quaternion)
                for quaternion in predicted[:, 3:7]
            ])
            return predicted, output["is_pad"][0].sigmoid().cpu().numpy()
        chunk = np.tile(
            np.asarray(observation["qpos"], dtype=np.float32),
            (predicted.shape[0], 1))
        chunk[:, self.policy_indices] = predicted
        return chunk, output["is_pad"][0].sigmoid().cpu().numpy()

    def get_action(self, observation):
        predicted_chunk = None
        predicted_pad = None
        if self._force_query or self.timestep % self.query_frequency == 0:
            predicted_chunk, predicted_pad = self.predict_chunk(observation)
            self.aggregator.add(self.timestep, predicted_chunk)
            self._force_query = False
        target_timestep = self.timestep + self.proleptic_steps
        action = self.aggregator.action(target_timestep)
        info = {
            "timestep": self.timestep,
            "target_timestep": target_timestep,
            "proleptic_steps": self.proleptic_steps,
            "ensemble_candidate_count": self.aggregator.last_candidate_count,
            "predicted_chunk": predicted_chunk,
            "predicted_pad": predicted_pad,
            "executed_action": action.copy(),
        }
        self.timestep += 1
        return action, info


__all__ = [
    "ACTPolicyRunner", "POLICY_REPRESENTATIONS",
    "TaskSpaceTemporalAggregator", "TemporalAggregator",
]
