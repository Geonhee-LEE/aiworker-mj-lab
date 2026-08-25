"""Dependency-free observation and action schema for the IL pipeline."""

SIDES = ("l", "r")
ARM_JOINTS = {
    side: tuple(f"arm_{side}_joint{index}" for index in range(1, 8)) for side in SIDES
}
ACTION_NAMES = (
    *(f"left_arm_joint_{index}" for index in range(1, 8)),
    "left_grasp",
    *(f"right_arm_joint_{index}" for index in range(1, 8)),
    "right_grasp",
)
ACTION_DIM = len(ACTION_NAMES)
RIGHT_POLICY_INDICES = tuple(range(8, ACTION_DIM))


__all__ = [
    "ACTION_DIM",
    "ACTION_NAMES",
    "ARM_JOINTS",
    "RIGHT_POLICY_INDICES",
    "SIDES",
]
