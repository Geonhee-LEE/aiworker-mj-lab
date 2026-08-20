"""Can-to-bin task reset and success metrics."""

from dataclasses import dataclass

import mujoco
import numpy as np

from ..config import SETTINGS


@dataclass(frozen=True)
class TaskMetrics:
    success: bool
    object_position_error: float
    object_speed: float
    can_position: np.ndarray
    target_position: np.ndarray


class CanInBoxTask:
    """Randomize the can on reset and detect a settled can inside the target bin."""

    def __init__(self, model):
        self.model = model
        self.can_joint = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
        self.can_body = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "can")
        target_name = SETTINGS.get("imitation.task.target_site")
        self.target_site = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, target_name)
        if min(self.can_joint, self.can_body, self.target_site) < 0:
            raise ValueError("can-to-bin task entities are missing from the model")
        self.can_qpos = int(model.jnt_qposadr[self.can_joint])
        self.can_dof = int(model.jnt_dofadr[self.can_joint])
        self.spawn_anchor_offset = np.asarray(SETTINGS.get(
            "imitation.reset.can_anchor_offset_from_target_xy_m"),
            dtype=float)
        self.spawn_jitter_radius = SETTINGS.number(
            "imitation.reset.can_spawn_jitter_radius_m", minimum=0.0)
        self.can_z = SETTINGS.number("imitation.reset.can_z_m")
        self.inner_half_extents = np.asarray(SETTINGS.get(
            "imitation.task.inner_half_extents_m"), dtype=float)
        self.height_range = tuple(float(v) for v in SETTINGS.get(
            "imitation.task.success_height_range_m"))
        self.settle_speed = SETTINGS.number(
            "imitation.task.settle_speed_m_s", minimum=0.0)
        if self.spawn_anchor_offset.shape != (2,):
            raise ValueError(
                "can_anchor_offset_from_target_xy_m must contain 2 values")
        if not self.height_range[0] < self.height_range[1]:
            raise ValueError("imitation task height range must be increasing")

    def reset(self, data, rng):
        """Reset only the free task object; robot reset is owned by the environment."""
        # sqrt(U) makes area, rather than radius, uniform over the disk. The
        # configured radius is therefore also a strict Euclidean error bound.
        radius = self.spawn_jitter_radius * np.sqrt(rng.uniform())
        angle = rng.uniform(0.0, 2.0 * np.pi)
        jitter = radius * np.asarray([np.cos(angle), np.sin(angle)])
        target_xy = np.asarray(data.site_xpos[self.target_site, :2])
        can_xy = target_xy + self.spawn_anchor_offset + jitter
        data.qpos[self.can_qpos:self.can_qpos + 3] = [
            can_xy[0], can_xy[1], self.can_z]
        data.qpos[self.can_qpos + 3:self.can_qpos + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[self.can_dof:self.can_dof + 6] = 0.0
        return np.asarray(data.qpos[self.can_qpos:self.can_qpos + 3]).copy()

    def metrics(self, data):
        can_position = np.asarray(data.xpos[self.can_body], dtype=float).copy()
        target_position = np.asarray(
            data.site_xpos[self.target_site], dtype=float).copy()
        delta = can_position - target_position
        speed = float(np.linalg.norm(data.qvel[self.can_dof:self.can_dof + 3]))
        inside_xy = bool(np.all(np.abs(delta[:2]) <= self.inner_half_extents))
        inside_height = self.height_range[0] <= can_position[2] <= self.height_range[1]
        return TaskMetrics(
            success=inside_xy and inside_height and speed <= self.settle_speed,
            object_position_error=float(np.linalg.norm(delta)),
            object_speed=speed,
            can_position=can_position,
            target_position=target_position,
        )


__all__ = ["CanInBoxTask", "TaskMetrics"]
