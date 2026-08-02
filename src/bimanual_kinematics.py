"""양손 rigid-grasp reference와 상대 pose task 계산.

Whole-body solver의 weight나 actuator 상태는 알지 않는다. 두 손의 현재 pose/Jacobian과
캡처한 상대 pose만 받아 6차원 상대 Jacobian과 drift-correction 속도를 만든다.
"""

import numpy as np

import kinematics_math


def capture_reference(right, left):
    """왼손 pose를 현재 오른손 frame에 표현해 캡처한다."""
    right_rotation = kinematics_math.rotation_from_quaternion(right.quaternion)
    left_rotation = kinematics_math.rotation_from_quaternion(left.quaternion)
    return {
        "position_right": right_rotation.T @ (left.position - right.position),
        "rotation_right": right_rotation.T @ left_rotation,
    }


def rigid_grasp_task(reference, site_states, dt,
                     max_linear_speed, max_angular_speed):
    """캡처 pose를 보존하는 상대 Jacobian과 보정 속도를 반환한다."""
    right, left = site_states["r"], site_states["l"]
    right_rotation = kinematics_math.rotation_from_quaternion(right.quaternion)
    right_to_left_world = right_rotation @ reference["position_right"]

    # 오른손 회전이 왼손 위치에 만드는 선속도까지 포함한 spatial transform이다.
    transform = np.eye(6)
    transform[:3, 3:] = -kinematics_math.skew(right_to_left_world)
    grasp_jacobian = left.jacobian - transform @ right.jacobian

    desired_left_position = right.position + right_to_left_world
    desired_left_rotation = right_rotation @ reference["rotation_right"]
    desired_left_quaternion = kinematics_math.quaternion_from_rotation(
        desired_left_rotation)

    # 1/dt 보정은 충돌 직후 지나치게 커질 수 있어 방향을 보존한 채 norm을 제한한다.
    correction_dt = max(float(dt), 1e-5)
    linear = kinematics_math.clip_norm(
        (desired_left_position - left.position) / correction_dt,
        max_linear_speed)
    angular = kinematics_math.clip_norm(
        kinematics_math.shortest_orientation_error(
            desired_left_quaternion, left.quaternion) / correction_dt,
        max_angular_speed)
    return grasp_jacobian, np.concatenate((linear, angular))
