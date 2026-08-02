"""기구학 모듈이 공통으로 사용하는 회전·쿼터니언 수학 유틸리티.

MuJoCo 쿼터니언 순서인 ``(w, x, y, z)``를 사용한다. 이 파일은 모델 트리나
solver 상태를 전혀 갖지 않으므로 좌표 변환 규칙을 독립적으로 읽고 테스트할 수 있다.
"""

import math

import mujoco
import numpy as np


def normalize_quaternion(quaternion):
    """유한한 단위 쿼터니언을 반환하고 부호를 ``w >= 0``으로 통일한다."""
    result = np.asarray(quaternion, dtype=float).copy()
    norm = float(np.linalg.norm(result))
    if not np.isfinite(norm) or norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    result /= norm
    if result[0] < 0.0:
        result *= -1.0
    return result


def multiply_quaternions(*quaternions):
    """MuJoCo 형식 쿼터니언을 왼쪽부터 순서대로 곱한다."""
    result = np.array([1.0, 0.0, 0.0, 0.0])
    for quaternion in quaternions:
        product = np.zeros(4)
        mujoco.mju_mulQuat(product, result, quaternion)
        result = product
    return result


def inverse_quaternion(quaternion):
    """단위 쿼터니언의 역회전인 켤레를 반환한다."""
    result = np.zeros(4)
    mujoco.mju_negQuat(result, quaternion)
    return result


def rpy_deg_to_quat(rpy_deg):
    """도 단위 roll, pitch, yaw를 MuJoCo 형식 쿼터니언으로 바꾼다."""
    roll, pitch, yaw = (math.radians(value) for value in rpy_deg)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_to_rpy_deg(quaternion):
    """MuJoCo 형식 쿼터니언을 도 단위 roll, pitch, yaw로 바꾼다."""
    w, x, y, z = normalize_quaternion(quaternion)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [math.degrees(value) for value in (roll, pitch, yaw)]


def shortest_orientation_error(target_quaternion, current_quaternion):
    """현재 자세에서 목표 자세로 가는 최단 world-frame 회전 벡터를 반환한다.

    ``target * inverse(current)`` 순서로 계산해야 결과 축이 world-frame Jacobian의
    회전 축과 일치한다. 두 쿼터니언의 내적을 확인해 q와 -q의 이중 표현도 제거한다.
    """
    target = normalize_quaternion(target_quaternion)
    current = normalize_quaternion(current_quaternion)
    if float(np.dot(target, current)) < 0.0:
        target *= -1.0
    error = multiply_quaternions(target, inverse_quaternion(current))
    error = normalize_quaternion(error)
    vector_norm = float(np.linalg.norm(error[1:]))
    if vector_norm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(vector_norm, max(error[0], 0.0))
    return error[1:] * (angle / vector_norm)


def rotation_from_quaternion(quaternion):
    """MuJoCo 쿼터니언을 3×3 회전 행렬로 변환한다."""
    w, x, y, z = normalize_quaternion(quaternion)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ])


def quaternion_from_rotation(rotation):
    """3×3 회전 행렬을 정규화된 MuJoCo 쿼터니언으로 변환한다."""
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(max(trace + 1.0, 0.0))
        quaternion = np.array([
            0.25 * scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
        ])
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = 2.0 * np.sqrt(max(
                1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2], 0.0))
            quaternion = np.array([
                (matrix[2, 1] - matrix[1, 2]) / scale,
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
            ])
        elif index == 1:
            scale = 2.0 * np.sqrt(max(
                1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2], 0.0))
            quaternion = np.array([
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
            ])
        else:
            scale = 2.0 * np.sqrt(max(
                1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1], 0.0))
            quaternion = np.array([
                (matrix[1, 0] - matrix[0, 1]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
            ])
    return normalize_quaternion(quaternion)


def axis_rotation(axis, angle):
    """관절 축과 회전각으로 Rodrigues 회전 행렬을 만든다."""
    axis = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        raise ValueError("kinematic joint axis must be non-zero")
    x, y, z = axis / norm
    sine, cosine = np.sin(angle), np.cos(angle)
    one_minus_cosine = 1.0 - cosine
    return np.array([
        [cosine + x * x * one_minus_cosine,
         x * y * one_minus_cosine - z * sine,
         x * z * one_minus_cosine + y * sine],
        [y * x * one_minus_cosine + z * sine,
         cosine + y * y * one_minus_cosine,
         y * z * one_minus_cosine - x * sine],
        [z * x * one_minus_cosine - y * sine,
         z * y * one_minus_cosine + x * sine,
         cosine + z * z * one_minus_cosine],
    ])


def clip_norm(vector, limit):
    """벡터 방향을 유지하면서 norm만 지정 상한으로 제한한다."""
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    return vector if norm <= limit or norm < 1e-12 else vector * (limit / norm)


def wrap_angle(angle):
    """각도를 ``[-pi, pi)`` 범위로 정규화한다."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def skew(vector):
    """3차원 벡터의 cross product를 나타내는 skew-symmetric 행렬."""
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
