"""양손 rigid-grasp reference와 상대 pose task 계산.

Whole-body solver의 weight나 actuator 상태는 알지 않는다. 두 손의 현재 pose/Jacobian과
캡처한 상대 pose만 받아 6차원 상대 Jacobian을 만들고, drift-correction 속도는 단일
팔·전신과 같은 ``kinematics.tasks`` 규칙으로 계산한다.
"""

import numpy as np

from ..kinematics import rotations
from ..kinematics import tasks as pose_tasks


def capture_reference(right, left):
    """왼손 pose를 현재 오른손 frame에 표현해 캡처한다."""
    right_rotation = rotations.rotation_from_quaternion(right.quaternion)
    left_rotation = rotations.rotation_from_quaternion(left.quaternion)
    return {
        "position_right": right_rotation.T @ (left.position - right.position),
        "rotation_right": right_rotation.T @ left_rotation,
    }


def rigid_grasp_task(reference, site_states, dt, max_linear_speed, max_angular_speed):
    """캡처한 오른손-왼손 상대 pose의 Jacobian과 복원 속도를 반환한다."""
    right, left = site_states["r"], site_states["l"]
    right_rotation = rotations.rotation_from_quaternion(right.quaternion)
    right_to_left_world = right_rotation @ reference["position_right"]

    transform = np.eye(6)
    transform[:3, 3:] = -rotations.skew(right_to_left_world)
    grasp_jacobian = left.jacobian - transform @ right.jacobian

    desired_left_position = right.position + right_to_left_world
    desired_left_rotation = right_rotation @ reference["rotation_right"]
    desired_left_quaternion = rotations.quaternion_from_rotation(desired_left_rotation)
    error = pose_tasks.pose_error(
        left.position,
        left.quaternion,
        desired_left_position,
        desired_left_quaternion,
    )
    correction_dt = max(float(dt), 1e-5)
    correction_velocity = pose_tasks.pose_velocity_command(
        error,
        position_gain=1.0 / correction_dt,
        orientation_gain=1.0 / correction_dt,
        max_linear_speed=max_linear_speed,
        max_angular_speed=max_angular_speed,
    )
    return grasp_jacobian, correction_velocity
