"""ALOHA-style imitation-learning components for the FFW-SH5 simulation.

The package deliberately depends on the arm torque and hand synergy controllers,
but never on the whole-body IK or mobile-base control path.
"""

from .action import ACTION_DIM, ACTION_NAMES, ActionAdapter
from .mujoco_env import AIWorkerMujocoEnv

__all__ = ["ACTION_DIM", "ACTION_NAMES", "ActionAdapter", "AIWorkerMujocoEnv"]
