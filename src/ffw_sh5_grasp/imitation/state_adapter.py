"""Project the full MuJoCo state onto the canonical 16D policy state."""

import mujoco
import numpy as np

from ..control import grasp
from .action import ACTION_DIM, ARM_JOINTS, SIDES


class PolicyStateAdapter:
    """Read arm joints plus measured hand-synergy position and velocity.

    The grasp coordinate is reconstructed by least-squares projection of the
    actuated finger positions onto the same linear synergy used by
    :func:`ffw_sh5_grasp.control.grasp.apply_grasp`.
    """

    def __init__(self, model):
        self.model = model
        self.arm_qpos = {}
        self.arm_dofs = {}
        self.synergy = {}
        for side in SIDES:
            joint_ids = np.array([
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                for name in ARM_JOINTS[side]
            ], dtype=int)
            if np.any(joint_ids < 0):
                raise ValueError(f"missing {side} arm joint for policy state")
            self.arm_qpos[side] = np.asarray(
                model.jnt_qposadr[joint_ids], dtype=int)
            self.arm_dofs[side] = np.asarray(
                model.jnt_dofadr[joint_ids], dtype=int)

            actuator_ids, offsets, grasp_slopes, thumb_slopes = (
                grasp._command_coefficients(model, side))
            slopes = grasp_slopes + thumb_slopes
            joint_ids = np.asarray(model.actuator_trnid[actuator_ids, 0], dtype=int)
            qpos = np.asarray(model.jnt_qposadr[joint_ids], dtype=int)
            dofs = np.asarray(model.jnt_dofadr[joint_ids], dtype=int)
            active = np.abs(slopes) > 1e-12
            slopes = slopes[active]
            self.synergy[side] = (
                qpos[active], dofs[active], offsets[active], slopes,
                float(np.dot(slopes, slopes)),
            )

    def _read_synergy(self, data, side):
        qpos, dofs, offsets, slopes, denominator = self.synergy[side]
        position = float(np.dot(slopes, data.qpos[qpos] - offsets) / denominator)
        velocity = float(np.dot(slopes, data.qvel[dofs]) / denominator)
        return float(np.clip(position, 0.0, 1.0)), velocity

    def get_qpos(self, data):
        left_grasp, _ = self._read_synergy(data, "l")
        right_grasp, _ = self._read_synergy(data, "r")
        result = np.concatenate((
            data.qpos[self.arm_qpos["l"]], [left_grasp],
            data.qpos[self.arm_qpos["r"]], [right_grasp],
        )).astype(np.float32)
        assert result.shape == (ACTION_DIM,)
        return result

    def get_qvel(self, data):
        _, left_grasp = self._read_synergy(data, "l")
        _, right_grasp = self._read_synergy(data, "r")
        result = np.concatenate((
            data.qvel[self.arm_dofs["l"]], [left_grasp],
            data.qvel[self.arm_dofs["r"]], [right_grasp],
        )).astype(np.float32)
        assert result.shape == (ACTION_DIM,)
        return result


__all__ = ["PolicyStateAdapter"]
