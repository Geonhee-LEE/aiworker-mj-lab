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
        self.head_fixed_position = np.asarray(
            SETTINGS.get("imitation.head_fixed_position_rad"), dtype=float)
        if self.head_fixed_position.shape != (2,):
            raise ValueError(
                "imitation.head_fixed_position_rad must contain 2 values")
        head_joint_ids = np.asarray([
            self._name_id(mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in ("head_joint1", "head_joint2")
        ], dtype=int)
        self.head_qpos = self.model.jnt_qposadr[head_joint_ids].copy()
        self.head_dofs = self.model.jnt_dofadr[head_joint_ids].copy()
        head_ranges = self.model.jnt_range[head_joint_ids]
        if np.any(self.head_fixed_position < head_ranges[:, 0]) or np.any(
                self.head_fixed_position > head_ranges[:, 1]):
            raise ValueError("configured head position exceeds joint limits")
        self.left_arm_fixed = bool(SETTINGS.get("imitation.left_arm_fixed"))
        self.left_arm_park_position = np.asarray(
            SETTINGS.get("imitation.left_arm_park_position_rad"), dtype=float)
        self.left_grasp_fixed = SETTINGS.number(
            "imitation.left_grasp_fixed", minimum=0.0)
        if self.left_arm_park_position.shape != (7,):
            raise ValueError("imitation.left_arm_park_position_rad must contain 7 values")
        left_ranges = self.action_adapter.arm_ranges["l"]
        if np.any(self.left_arm_park_position < left_ranges[:, 0]) or np.any(
                self.left_arm_park_position > left_ranges[:, 1]):
            raise ValueError("configured left-arm park position exceeds joint limits")
        if self.left_grasp_fixed > 1.0:
            raise ValueError("imitation.left_grasp_fixed must not exceed 1")
        self.arm_controllers = {
            side: arm.ArmTorqueController(self.model, ARM_JOINTS[side])
            for side in SIDES
        }
        self.task = CanInBoxTask(self.model)
        self._enable_target_bin_collisions()
        self._enable_task_hand_world_collisions()
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
        for name, target in zip(
                ("head_joint1", "head_joint2"), self.head_fixed_position):
            actuator_id = self._name_id(
                mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            fixed_index = np.flatnonzero(self.fixed_actuators == actuator_id)
            if fixed_index.size != 1:
                raise ValueError(
                    f"head actuator is not fixed exactly once: {name}")
            self.fixed_ctrl[fixed_index[0]] = target

    def _enable_target_bin_collisions(self):
        """Enable physical robot/can contacts for the task-local target bin.

        The shared MJCF keeps these geoms on an isolated collision bit so legacy
        teleop regression tasks are unchanged. This environment owns its model
        instance and promotes the bin geoms to the normal contact group.
        """
        body_id = self._name_id(mujoco.mjtObj.mjOBJ_BODY, "target_bin")
        self.target_bin_geom_ids = np.flatnonzero(
            self.model.geom_bodyid == body_id).astype(int)
        if self.target_bin_geom_ids.size != 5:
            raise ValueError("target_bin must contain one floor and four collision walls")
        self.model.geom_contype[self.target_bin_geom_ids] = 1
        self.model.geom_conaffinity[self.target_bin_geom_ids] = 1
        # MuJoCo caches the union of geom masks per body at compile time and
        # uses it during broad-phase filtering. Updating only geom_* lets the
        # can (whose affinity includes bit 2) hit the bin but still filters out
        # robot/bin pairs. Keep the body-level cache consistent as well.
        self.model.body_contype[body_id] = 1
        self.model.body_conaffinity[body_id] = 1

    def _enable_task_hand_world_collisions(self):
        """Restore right ring/pinky contacts needed by the fixed task bin.

        The shared model excludes these cosmetic fingers from ``world`` to
        preserve older table-grasp regressions. The fixed target bin is welded
        to world, so those exclusions also (unintentionally) let the two
        fingers pass through its walls. Disable only those eight exclusions on
        this environment's private model instance and keep the signature array
        sorted as required by MuJoCo's contact filter.
        """
        disabled_signature = np.iinfo(self.model.exclude_signature.dtype).max
        for index in range(13, 21):
            body_id = self._name_id(
                mujoco.mjtObj.mjOBJ_BODY, f"finger_r_link{index}")
            matches = np.flatnonzero(self.model.exclude_signature == body_id)
            if matches.size != 1:
                raise ValueError(
                    f"expected one world exclusion for finger_r_link{index}")
            self.model.exclude_signature[matches[0]] = disabled_signature
        self.model.exclude_signature.sort()

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
        self.data.qpos[self.head_qpos] = self.head_fixed_position
        self.data.qvel[self.head_dofs] = 0.0
        if self.left_arm_fixed:
            self.data.qpos[self.state_adapter.arm_qpos["l"]] = (
                self.left_arm_park_position)
            self.data.qvel[self.state_adapter.arm_dofs["l"]] = 0.0
        self.initial_can_position = self.task.reset(self.data, self.rng)
        self.data.ctrl[self.fixed_actuators] = self.fixed_ctrl
        mujoco.mj_forward(self.model, self.data)
        self.last_action = self.prepare_action(self.get_qpos())
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
        bounded = self.prepare_action(action)
        decoded = self.action_adapter.decode(bounded)
        for _ in range(self.steps_per_control):
            self._apply_action_once(decoded)
            mujoco.mj_step(self.model, self.data)
        self.last_action = bounded
        return self.get_observation()

    def prepare_action(self, action):
        """Clip policy output and replace the locked-left fields with constants."""
        bounded = self.action_adapter.validate(action, clip=True)
        if self.left_arm_fixed:
            bounded[:7] = self.left_arm_park_position
            bounded[7] = self.left_grasp_fixed
        return bounded

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
