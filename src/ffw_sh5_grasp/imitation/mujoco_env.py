"""Arm-only ALOHA-style environment backed by the existing MuJoCo controllers."""

import hashlib
from pathlib import Path

import mujoco
import numpy as np

from ..config import SETTINGS
from ..control import arm, grasp
from ..paths import MODEL_PATH
from .action import ARM_JOINTS, SIDES, ActionAdapter
from .cameras import MujocoCameraManager
from .state_adapter import PolicyStateAdapter
from .task import CanInBoxTask


class AIWorkerMujocoEnv:
    """Execute 16D absolute joint targets without whole-body IK.

    Base, lift and head are held at their home references. Only the two arm
    torque controllers and the existing hand-synergy controller receive policy
    commands. Robot qpos is never overwritten during :meth:`step`.
    """

    def __init__(self, model_path=None, *, control_hz=None, camera_width=None,
                 camera_height=None, camera_names=None, render_images=True,
                 seed=None):
        self.model_path = Path(MODEL_PATH if model_path is None else model_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.home_key = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_KEY,
            SETTINGS.get("application.home_keyframe"))
        if self.home_key < 0:
            raise ValueError("configured home keyframe does not exist")
        self.control_hz = float(
            SETTINGS.number("imitation.control_hz", positive=True)
            if control_hz is None else control_hz)
        if self.control_hz <= 0:
            raise ValueError("control_hz must be positive")
        self.steps_per_control = max(
            1, round((1.0 / self.control_hz) / self.model.opt.timestep))
        self.actual_control_hz = 1.0 / (
            self.steps_per_control * self.model.opt.timestep)

        self.action_adapter = ActionAdapter(self.model)
        self.state_adapter = PolicyStateAdapter(self.model)
        self.arm_controllers = {
            side: arm.ArmTorqueController(self.model, ARM_JOINTS[side])
            for side in SIDES
        }
        self.task = CanInBoxTask(self.model)
        self._bind_fixed_actuators()
        self._configure_passive_base_hold()

        self.render_images = bool(render_images)
        names = (SETTINGS.get("imitation.camera.names")
                 if camera_names is None else camera_names)
        width = (SETTINGS.integer("imitation.camera.width", minimum=1)
                 if camera_width is None else camera_width)
        height = (SETTINGS.integer("imitation.camera.height", minimum=1)
                  if camera_height is None else camera_height)
        self.cameras = (MujocoCameraManager(
            self.model, self.data, width=width, height=height,
            camera_names=names) if self.render_images else None)
        self.camera_names = tuple(names)

        self.rng = np.random.default_rng(seed)
        self.last_seed = seed
        self.last_action = None
        self.initial_can_position = None
        self.reset(seed=seed)

    def _name_id(self, kind, name):
        object_id = mujoco.mj_name2id(self.model, kind, name)
        if object_id < 0:
            raise ValueError(f"required MuJoCo object is missing: {name}")
        return object_id

    def _bind_fixed_actuators(self):
        fixed_names = (
            "lift_joint", "head_joint1", "head_joint2",
            "left_wheel_steer", "right_wheel_steer", "rear_wheel_steer",
            "left_wheel_drive", "right_wheel_drive", "rear_wheel_drive",
        )
        self.fixed_actuators = np.asarray([
            self._name_id(mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in fixed_names
        ], dtype=int)
        self.fixed_ctrl = np.asarray(
            self.model.key_ctrl[self.home_key, self.fixed_actuators],
            dtype=float).copy()

    def _configure_passive_base_hold(self):
        """Hold unactuated planar base joints with physical spring/damping forces."""
        for name in ("base_x", "base_y", "base_yaw"):
            joint_id = self._name_id(mujoco.mjtObj.mjOBJ_JOINT, name)
            dof = int(self.model.jnt_dofadr[joint_id])
            self.model.jnt_stiffness[joint_id] = 2.0e4
            self.model.dof_damping[dof] = 2.0e3

    @property
    def model_hash(self):
        return hashlib.sha256(self.model_path.read_bytes()).hexdigest()

    def reset(self, seed=None):
        """Restore the entire robot home state and randomize only the can pose."""
        if seed is None:
            seed = int(self.rng.integers(0, np.iinfo(np.int32).max))
        self.rng = np.random.default_rng(seed)
        self.last_seed = int(seed)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.home_key)
        self.initial_can_position = self.task.reset(self.data, self.rng)
        self.data.ctrl[self.fixed_actuators] = self.fixed_ctrl
        mujoco.mj_forward(self.model, self.data)
        self.last_action = self.get_qpos().astype(float)
        return self.get_observation()

    def _apply_action_once(self, decoded):
        for side in SIDES:
            self.arm_controllers[side].apply(
                self.data, decoded.arm_positions[side])
            grasp.apply_grasp(
                self.model, self.data,
                grasp=decoded.grasp[side], thumb=decoded.grasp[side], side=side)
        self.data.ctrl[self.fixed_actuators] = self.fixed_ctrl

    def step(self, action):
        """Advance one 25 Hz control frame through actuator-level physics."""
        bounded = self.action_adapter.validate(action, clip=True)
        decoded = self.action_adapter.decode(bounded)
        for _ in range(self.steps_per_control):
            self._apply_action_once(decoded)
            mujoco.mj_step(self.model, self.data)
        self.last_action = bounded
        return self.get_observation()

    def get_qpos(self):
        return self.state_adapter.get_qpos(self.data)

    def get_qvel(self):
        return self.state_adapter.get_qvel(self.data)

    def get_images(self):
        if self.cameras is None:
            return {}
        return self.cameras.render()

    def get_observation(self):
        metrics = self.task.metrics(self.data)
        return {
            "qpos": self.get_qpos(),
            "qvel": self.get_qvel(),
            "images": self.get_images(),
            "task": {
                "success": metrics.success,
                "object_position_error": metrics.object_position_error,
                "object_speed": metrics.object_speed,
            },
            "debug": {
                "full_qpos": np.asarray(self.data.qpos).copy(),
                "full_qvel": np.asarray(self.data.qvel).copy(),
                "task_object_pose": np.concatenate((
                    metrics.can_position,
                    self.data.qpos[
                        self.task.can_qpos + 3:self.task.can_qpos + 7],
                )).copy(),
                "target_position": metrics.target_position.copy(),
                "ee_pose_left": self._site_pose("grasp_target_l"),
                "ee_pose_right": self._site_pose("grasp_target_r"),
            },
        }

    def _site_pose(self, name):
        site_id = self._name_id(mujoco.mjtObj.mjOBJ_SITE, name)
        quaternion = np.zeros(4)
        mujoco.mju_mat2Quat(quaternion, self.data.site_xmat[site_id])
        return np.concatenate((self.data.site_xpos[site_id], quaternion))

    def close(self):
        if self.cameras is not None:
            self.cameras.close()

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        self.close()


__all__ = ["AIWorkerMujocoEnv"]
