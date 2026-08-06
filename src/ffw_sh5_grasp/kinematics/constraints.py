"""Differential IK의 hard velocity/position/barrier 제약 계산.

이 모듈은 로봇 상태를 소유하지 않는다. 현재 관절값과 한계 또는 collision distance
gradient를 받아 solver가 바로 사용할 box와 선형 부등식으로 변환한다. 충돌 최근접점
기하는 :mod:`.collision`, 제약을 만족시키는 수치 계산은 :mod:`.solver`의 책임이다.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VelocityBarrier:
    """``gradient @ qdot >= lower``인 한 개의 속도 CBF 제약."""

    name: str
    distance: float
    gradient: np.ndarray
    lower: float


def joint_velocity_bounds(current_position, velocity_limits, position_limited,
                          position_ranges, dt, *, margin, gain):
    """물리 속도 상한과 joint-limit CBF를 결합한 box bound를 만든다."""
    current_position = np.asarray(current_position, dtype=float)
    velocity_limits = np.asarray(velocity_limits, dtype=float)
    position_limited = np.asarray(position_limited, dtype=bool)
    position_ranges = np.asarray(position_ranges, dtype=float)
    count = current_position.size
    if current_position.shape != (count,) or velocity_limits.shape != (count,):
        raise ValueError("joint position and velocity limits must be vectors")
    if position_limited.shape != (count,) or position_ranges.shape != (count, 2):
        raise ValueError("incompatible joint-limit shapes")
    if np.any(velocity_limits <= 0.0):
        raise ValueError("joint velocity limits must be positive")
    margin, gain = float(margin), float(gain)
    if margin < 0.0 or gain <= 0.0:
        raise ValueError("joint-limit margin must be non-negative and gain positive")

    lower = -velocity_limits.copy()
    upper = velocity_limits.copy()
    barrier_gain = min(gain, 1.0 / max(float(dt), 1e-5))
    for index in np.flatnonzero(position_limited):
        lo, hi = position_ranges[index]
        safe_margin = min(margin, max(0.0, 0.25 * (hi - lo)))
        safe_lo, safe_hi = lo + safe_margin, hi - safe_margin
        lower[index] = max(
            lower[index], -barrier_gain * (current_position[index] - safe_lo))
        upper[index] = min(
            upper[index], barrier_gain * (safe_hi - current_position[index]))
        if lower[index] > upper[index]:
            recovery = (velocity_limits[index]
                        if current_position[index] < safe_lo
                        else -velocity_limits[index])
            lower[index] = upper[index] = recovery
    return lower, upper


def collision_velocity_barriers(distance_constraints, dt, *, safe_distance, gain):
    """거리/gradient 결과를 충돌 접근 속도 CBF 목록으로 변환한다."""
    safe_distance, gain = float(safe_distance), float(gain)
    if safe_distance < 0.0 or gain <= 0.0:
        raise ValueError("collision safe distance must be non-negative and gain positive")
    barrier_gain = min(gain, 1.0 / max(float(dt), 1e-5))
    barriers = []
    for constraint in distance_constraints:
        gradient = np.asarray(constraint.gradient, dtype=float)
        if gradient.ndim != 1:
            raise ValueError("collision gradient must be a vector")
        if np.linalg.norm(gradient) < 1e-10:
            continue
        distance = float(constraint.distance)
        barriers.append(VelocityBarrier(
            name=str(constraint.name),
            distance=distance,
            gradient=gradient,
            lower=-barrier_gain * (distance - safe_distance),
        ))
    return tuple(barriers)


def clip_joint_positions(position, position_limited, position_ranges):
    """위치 제한이 있는 자유도만 모델 범위로 자른 새 벡터를 반환한다."""
    result = np.asarray(position, dtype=float).copy()
    limited = np.asarray(position_limited, dtype=bool)
    ranges = np.asarray(position_ranges, dtype=float)
    if limited.shape != result.shape or ranges.shape != (result.size, 2):
        raise ValueError("incompatible joint position-limit shapes")
    result[limited] = np.clip(
        result[limited], ranges[limited, 0], ranges[limited, 1])
    return result


__all__ = [
    "VelocityBarrier",
    "clip_joint_positions",
    "collision_velocity_barriers",
    "joint_velocity_bounds",
]
