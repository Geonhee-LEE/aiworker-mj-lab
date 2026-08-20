"""Replaceable 16D leader sources for demonstration collection."""

from abc import ABC, abstractmethod

import mujoco
import numpy as np

from ..kinematics import DifferentialIKSolver, KinematicTree
from ..kinematics.tasks import pose_error, pose_velocity_command
from .action import ARM_JOINTS, ActionAdapter, SIDES


class Leader(ABC):
    @abstractmethod
    def reset(self):
        """Synchronize the leader with the current follower state."""

    @abstractmethod
    def get_action(self):
        """Return one canonical 16D absolute joint target."""


class ReplayLeader(Leader):
    def __init__(self, actions):
        self.actions = np.asarray(actions, dtype=float)
        self.index = 0

    def reset(self):
        self.index = 0

    def get_action(self):
        if self.index >= len(self.actions):
            raise StopIteration
        action = self.actions[self.index].copy()
        self.index += 1
        return action


class GizmoLeader(Leader):
    """Arm-only differential IK leader driven by externally edited EE targets.

    A UI, SpaceMouse or VR adapter only needs to call :meth:`set_target_pose`;
    the leader converts those targets to the same 16D joint action consumed by
    ACT. No whole-body solver is imported or invoked.
    """

    def __init__(self, env, *, linear_speed=0.6, angular_speed=2.0,
                 joint_speed=4.5):
        self.env = env
        self.model = env.model
        self.tree = KinematicTree(self.model)
        self.solver = DifferentialIKSolver(method="dls")
        self.adapter = ActionAdapter(self.model)
        self.linear_speed = float(linear_speed)
        self.angular_speed = float(angular_speed)
        self.joint_speed = float(joint_speed)
        self.site_ids = {
            side: mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, f"grasp_target_{side}")
            for side in SIDES
        }
        self.joint_ids = {
            side: np.array([
                mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                for name in ARM_JOINTS[side]
            ], dtype=int)
            for side in SIDES
        }
        self.qpos_adrs = {
            side: np.asarray(self.model.jnt_qposadr[ids], dtype=int)
            for side, ids in self.joint_ids.items()
        }
        self.ranges = {
            side: np.asarray(self.model.jnt_range[ids], dtype=float)
            for side, ids in self.joint_ids.items()
        }
        self.targets = {}
        self.grasp = {"l": 0.0, "r": 0.0}
        self.reset()

    def _current_site(self, side):
        return self.tree.forward_site(
            self.env.data.qpos, self.site_ids[side], self.joint_ids[side])

    def reset(self):
        self.targets = {
            side: (state.position.copy(), state.quaternion.copy())
            for side in SIDES
            for state in (self._current_site(side),)
        }
        state = self.env.get_qpos()
        self.grasp = {"l": float(state[7]), "r": float(state[15])}

    def set_target_pose(self, side, position, quaternion):
        if side not in SIDES:
            raise ValueError(f"unknown side: {side}")
        position = np.asarray(position, dtype=float)
        quaternion = np.asarray(quaternion, dtype=float)
        if position.shape != (3,) or quaternion.shape != (4,):
            raise ValueError("target pose must be position[3], quaternion[4]")
        if not np.all(np.isfinite(np.concatenate((position, quaternion)))):
            raise ValueError("target pose contains NaN or infinity")
        norm = np.linalg.norm(quaternion)
        if norm < 1e-12:
            raise ValueError("target quaternion cannot be zero")
        self.targets[side] = (position.copy(), quaternion / norm)

    def set_grasp(self, side, value):
        if side not in SIDES:
            raise ValueError(f"unknown side: {side}")
        self.grasp[side] = float(np.clip(value, 0.0, 1.0))

    def get_action(self):
        arms = {}
        dt = 1.0 / self.env.actual_control_hz
        for side in SIDES:
            state = self._current_site(side)
            target_position, target_quaternion = self.targets[side]
            error = pose_error(
                state.position, state.quaternion,
                target_position, target_quaternion)
            velocity = pose_velocity_command(
                error, position_gain=8.0, orientation_gain=6.0,
                max_linear_speed=self.linear_speed,
                max_angular_speed=self.angular_speed)
            lower = np.full(7, -self.joint_speed)
            upper = np.full(7, self.joint_speed)
            qdot = self.solver.solve(state.jacobian, velocity, lower, upper)
            current = np.asarray(
                self.env.data.qpos[self.qpos_adrs[side]], dtype=float)
            arms[side] = np.clip(
                current + dt * qdot,
                self.ranges[side][:, 0], self.ranges[side][:, 1])
        return self.adapter.encode(
            arms["l"], self.grasp["l"], arms["r"], self.grasp["r"])


__all__ = ["GizmoLeader", "Leader", "ReplayLeader"]
