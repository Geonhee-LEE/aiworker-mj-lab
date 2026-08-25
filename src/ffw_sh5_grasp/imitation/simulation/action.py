"""Canonical 16D joint-space action contract shared by leaders and policies."""

from dataclasses import dataclass

import mujoco
import numpy as np

from ..data.schema import (
    ACTION_DIM,
    ACTION_NAMES,
    ARM_JOINTS,
    RIGHT_POLICY_INDICES,
    SIDES,
)


def _joint_id(model, name):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise ValueError(f"required policy joint is missing: {name}")
    return joint_id


@dataclass(frozen=True)
class DecodedAction:
    """Validated arm targets and hand synergies for a single control frame."""

    arm_positions: dict[str, np.ndarray]
    grasp: dict[str, float]


class ActionAdapter:
    """Validate, bound and split the canonical left-first 16D action vector."""

    def __init__(self, model):
        self.model = model
        self.arm_ranges = {
            side: np.asarray(
                [model.jnt_range[_joint_id(model, name)] for name in ARM_JOINTS[side]],
                dtype=float,
            )
            for side in SIDES
        }
        self.lower = np.concatenate(
            (
                self.arm_ranges["l"][:, 0],
                [0.0],
                self.arm_ranges["r"][:, 0],
                [0.0],
            )
        )
        self.upper = np.concatenate(
            (
                self.arm_ranges["l"][:, 1],
                [1.0],
                self.arm_ranges["r"][:, 1],
                [1.0],
            )
        )

    def validate(self, action, *, clip=False):
        values = np.asarray(action, dtype=float)
        if values.shape != (ACTION_DIM,):
            raise ValueError(
                f"action must have shape ({ACTION_DIM},), got {values.shape}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("action contains NaN or infinity")
        if clip:
            return np.clip(values, self.lower, self.upper)
        invalid = np.flatnonzero((values < self.lower) | (values > self.upper))
        if invalid.size:
            names = ", ".join(ACTION_NAMES[index] for index in invalid)
            raise ValueError(f"action values outside policy limits: {names}")
        return values.copy()

    def decode(self, action, *, clip=False):
        values = self.validate(action, clip=clip)
        return DecodedAction(
            arm_positions={"l": values[:7].copy(), "r": values[8:15].copy()},
            grasp={"l": float(values[7]), "r": float(values[15])},
        )

    @staticmethod
    def encode(left_arm, left_grasp, right_arm, right_grasp):
        result = np.concatenate(
            (
                np.asarray(left_arm, dtype=float),
                [float(left_grasp)],
                np.asarray(right_arm, dtype=float),
                [float(right_grasp)],
            )
        )
        if result.shape != (ACTION_DIM,):
            raise ValueError(f"encoded action must have shape ({ACTION_DIM},)")
        return result


__all__ = [
    "ACTION_DIM",
    "ACTION_NAMES",
    "ARM_JOINTS",
    "RIGHT_POLICY_INDICES",
    "SIDES",
    "ActionAdapter",
    "DecodedAction",
]
