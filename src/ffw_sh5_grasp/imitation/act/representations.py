"""Interchangeable joint- and task-space features for ACT training.

The HDF5 episode remains the source of truth. Joint mode selects canonical
joint/grasp fields, while task mode consumes the recorded world-frame right EE
pose and derives target EE actions with FK on each recorded joint target.
"""

import hashlib
from dataclasses import dataclass
from pathlib import Path

import h5py
import mujoco
import numpy as np

from ...kinematics import KinematicTree
from ...paths import MODEL_PATH
from ..data.schema import ACTION_NAMES, RIGHT_POLICY_INDICES
from ..simulation.action import ARM_JOINTS

REPRESENTATION_NAMES = ("joint", "task")
RIGHT_GRASP_INDEX = 15
RIGHT_ARM_ACTION_SLICE = slice(8, 15)


@dataclass(frozen=True)
class EpisodePolicyFeatures:
    """Small non-image arrays consumed by one policy representation."""

    state: np.ndarray
    action: np.ndarray

    def __post_init__(self):
        if self.state.ndim != 2 or self.action.ndim != 2:
            raise ValueError("policy state/action features must be matrices")
        if self.state.shape != self.action.shape:
            raise ValueError("policy state/action feature shapes must match")
        if not np.all(np.isfinite(self.state)) or not np.all(np.isfinite(self.action)):
            raise ValueError("policy features contain NaN or infinity")


def _canonical_quaternions(values):
    """Normalize wxyz quaternions and select the non-negative-w hemisphere."""
    quaternions = np.asarray(values, dtype=np.float64).copy()
    if quaternions.ndim != 2 or quaternions.shape[1] != 4:
        raise ValueError("quaternion array must have shape [T,4]")
    norms = np.linalg.norm(quaternions, axis=1, keepdims=True)
    if np.any(norms < 1e-12):
        raise ValueError("task-space episode contains a zero quaternion")
    quaternions /= norms
    quaternions[quaternions[:, 0] < 0.0] *= -1.0
    return quaternions


class PolicyRepresentation:
    """Common cached feature-extraction contract."""

    name = ""
    state_names = ()
    action_names = ()

    def __init__(self):
        self._cache = {}

    @property
    def state_dim(self):
        return len(self.state_names)

    @property
    def action_dim(self):
        return len(self.action_names)

    def episode_features(self, path):
        path = Path(path)
        key = str(path.resolve())
        features = self._cache.get(key)
        if features is None:
            features = self._load_episode(path)
            self._cache[key] = features
        return features

    def _load_episode(self, path):
        raise NotImplementedError

    def metadata(self):
        return {
            "name": self.name,
            "state_names": list(self.state_names),
            "action_names": list(self.action_names),
        }


class JointRepresentation(PolicyRepresentation):
    """Canonical joint positions and hand synergies for one or both arms."""

    name = "joint"

    def __init__(self, policy_side="right"):
        super().__init__()
        if policy_side not in ("right", "both"):
            raise ValueError("joint policy_side must be 'right' or 'both'")
        self.policy_side = policy_side
        self.indices = (
            RIGHT_POLICY_INDICES
            if policy_side == "right"
            else tuple(range(len(ACTION_NAMES)))
        )
        self.state_names = tuple(ACTION_NAMES[index] for index in self.indices)
        self.action_names = self.state_names

    def _load_episode(self, path):
        with h5py.File(path, "r") as root:
            state = root["observations/qpos"][:, self.indices]
            action = root["action"][:, self.indices]
        return EpisodePolicyFeatures(
            state=np.asarray(state, dtype=np.float32),
            action=np.asarray(action, dtype=np.float32),
        )

    def metadata(self):
        return {
            **super().metadata(),
            "policy_side": self.policy_side,
            "policy_indices": list(self.indices),
        }


class RightTaskRepresentation(PolicyRepresentation):
    """World-frame right EE pose plus hand synergy.

    Task actions are FK projections of the exact right joint target stored in
    ``/action``. This preserves the recorder's obs_t/action_t contract.
    """

    name = "task"
    state_names = (
        "right_ee_x",
        "right_ee_y",
        "right_ee_z",
        "right_ee_qw",
        "right_ee_qx",
        "right_ee_qy",
        "right_ee_qz",
        "right_grasp",
    )
    action_names = tuple(f"target_{name}" for name in state_names)

    def __init__(self, model_path=MODEL_PATH):
        super().__init__()
        self.model_path = Path(model_path)
        model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.model_hash = hashlib.sha256(self.model_path.read_bytes()).hexdigest()
        self.tree = KinematicTree(model)
        self.joint_ids = np.asarray(
            [
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                for joint_name in ARM_JOINTS["r"]
            ],
            dtype=int,
        )
        if np.any(self.joint_ids < 0):
            raise ValueError("model is missing a right-arm joint")
        self.qpos_addresses = np.asarray(model.jnt_qposadr[self.joint_ids], dtype=int)
        self.site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r"
        )
        if self.site_id < 0:
            raise ValueError("model is missing grasp_target_r")

    @staticmethod
    def _decoded_attr(value):
        return value.decode("utf-8") if isinstance(value, bytes) else str(value)

    def _load_episode(self, path):
        with h5py.File(path, "r") as root:
            required = (
                "observations/ee_pose/right",
                "observations/qpos",
                "action",
                "debug/full_qpos",
            )
            missing = [name for name in required if name not in root]
            if missing:
                raise ValueError(f"task representation requires {missing} in {path}")
            frame = self._decoded_attr(root.attrs.get("ee_pose_frame", ""))
            order = self._decoded_attr(root.attrs.get("ee_pose_quaternion_order", ""))
            if frame != "world" or order != "wxyz":
                raise ValueError(
                    f"unsupported EE pose convention in {path}: "
                    f"frame={frame!r}, quaternion={order!r}"
                )
            episode_model_hash = self._decoded_attr(root.attrs.get("model_hash", ""))
            if episode_model_hash != self.model_hash:
                raise ValueError(f"task FK model hash does not match episode {path}")

            measured_pose = np.asarray(
                root["observations/ee_pose/right"][:], dtype=np.float64
            )
            policy_qpos = np.asarray(root["observations/qpos"][:], dtype=np.float64)
            actions = np.asarray(root["action"][:], dtype=np.float64)
            full_qpos = np.asarray(root["debug/full_qpos"][:], dtype=np.float64)

        length = len(actions)
        if (
            measured_pose.shape != (length, 7)
            or policy_qpos.shape != (length, 16)
            or full_qpos.shape != (length, self.tree.nq)
        ):
            raise ValueError(f"misaligned task-space source arrays in {path}")

        measured_pose[:, 3:] = _canonical_quaternions(measured_pose[:, 3:])
        target_pose = np.empty((length, 7), dtype=np.float64)
        for timestep in range(length):
            target_qpos = full_qpos[timestep].copy()
            target_qpos[self.qpos_addresses] = actions[timestep, RIGHT_ARM_ACTION_SLICE]
            target = self.tree.forward_site(target_qpos, self.site_id, self.joint_ids)
            target_pose[timestep, :3] = target.position
            target_pose[timestep, 3:] = target.quaternion
        target_pose[:, 3:] = _canonical_quaternions(target_pose[:, 3:])

        state = np.concatenate(
            (
                measured_pose,
                policy_qpos[:, [RIGHT_GRASP_INDEX]],
            ),
            axis=1,
        )
        action = np.concatenate(
            (
                target_pose,
                actions[:, [RIGHT_GRASP_INDEX]],
            ),
            axis=1,
        )
        return EpisodePolicyFeatures(
            state=state.astype(np.float32),
            action=action.astype(np.float32),
        )

    def metadata(self):
        return {
            **super().metadata(),
            "policy_side": "right",
            "ee_pose_frame": "world",
            "ee_pose_quaternion_order": "wxyz",
            "quaternion_canonicalization": "unit_norm_and_nonnegative_w",
            "action_pose_source": "fk_of_recorded_right_joint_action",
            "model_hash": self.model_hash,
        }


def create_representation(name, *, policy_side="right", model_path=MODEL_PATH):
    name = str(name).strip().lower()
    if name == "joint":
        return JointRepresentation(policy_side=policy_side)
    if name == "task":
        if policy_side != "right":
            raise ValueError("task representation supports the right arm only")
        return RightTaskRepresentation(model_path=model_path)
    raise ValueError(
        f"unknown policy representation {name!r}; choose one of {REPRESENTATION_NAMES}"
    )


__all__ = [
    "REPRESENTATION_NAMES",
    "EpisodePolicyFeatures",
    "JointRepresentation",
    "PolicyRepresentation",
    "RightTaskRepresentation",
    "create_representation",
]
