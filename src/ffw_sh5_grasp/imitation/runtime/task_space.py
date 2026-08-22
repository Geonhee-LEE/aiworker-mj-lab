"""Shared task-space policy action to joint-controller bridge."""

import math
from dataclasses import dataclass

import numpy as np

from ...kinematics import rotations


@dataclass(frozen=True)
class TaskIKDiagnostics:
    position_error_mm: float
    orientation_error_rad: float
    minimum_collision_distance_m: float
    collision_constraint_violation: float
    active_collision_pairs: tuple


def task_action_to_joint(env, solver, task_action, *, speed_scale=1.0):
    """Convert one world-frame right EE pose action through arm-only IK.

    This is the single policy-to-IK boundary shared by interactive Teleop and
    headless evaluation, so representation comparisons execute the same
    controller, joint limits, velocity limits, and collision CBF.
    """
    task_action = np.asarray(task_action, dtype=float)
    if task_action.shape != (8,) or not np.all(np.isfinite(task_action)):
        raise ValueError(
            "task policy action must be a finite 8D pose+grasp vector")
    quaternion_norm = float(np.linalg.norm(task_action[3:7]))
    if quaternion_norm < 1e-8:
        raise ValueError("task policy predicted a zero quaternion")
    speed_scale = float(speed_scale)
    if not np.isfinite(speed_scale) or speed_scale <= 0.0:
        raise ValueError("policy IK speed scale must be finite and positive")

    target_quaternion = rotations.normalize_quaternion(task_action[3:7])
    dt = 1.0 / env.actual_control_hz
    original_position_gain = solver.position_gain
    original_orientation_gain = solver.orientation_gain
    try:
        solver.position_gain = original_position_gain * speed_scale
        solver.orientation_gain = original_orientation_gain * speed_scale
        command = solver.solve(
            env.data,
            {"r": (task_action[:3], target_quaternion)},
            dt,
            active_sides=("r",),
            whole_body_enabled=False,
        )
    finally:
        solver.position_gain = original_position_gain
        solver.orientation_gain = original_orientation_gain

    joint_action = env.get_qpos()
    joint_action[8:15] = command.arm_positions["r"]
    joint_action[15] = task_action[7]
    diagnostics = TaskIKDiagnostics(
        position_error_mm=float(command.position_errors["r"] * 1000.0),
        orientation_error_rad=float(command.orientation_errors["r"]),
        minimum_collision_distance_m=float(
            command.minimum_collision_distance
            if np.isfinite(command.minimum_collision_distance) else math.inf),
        collision_constraint_violation=float(
            command.collision_constraint_violation),
        active_collision_pairs=tuple(command.active_collision_pairs),
    )
    return joint_action, diagnostics


__all__ = [
    "TaskIKDiagnostics", "task_action_to_joint",
]
