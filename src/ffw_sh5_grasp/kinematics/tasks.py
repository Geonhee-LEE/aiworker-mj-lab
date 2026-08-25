"""Differential IK가 공유하는 soft task와 pose residual 계산.

PyRoki의 residual/cost 카탈로그와 같은 경계다. 로봇 모델과 제어 상태는 모르며,
Jacobian과 목표 속도를 무차원 weighted least-squares 행으로 바꾸는 규칙만 소유한다.
하드 bound와 barrier는 :mod:`.constraints`, 실제 해법은 :mod:`.solver`에 둔다.
"""

from dataclasses import dataclass

import numpy as np

from .rotations import clip_norm, shortest_orientation_error


@dataclass(frozen=True)
class PoseError:
    """현재 pose에서 목표 pose까지의 world-frame 위치·자세 오차.

    ``position``은 미터 단위의 ``target_position - current_position`` 3차원 벡터다.
    ``orientation``은 ``target * inverse(current)``의 최단 축각 3차원 벡터이며 단위는
    rad이다. 두 벡터가 같은 world frame에 있으므로 world-aligned geometric
    Jacobian과 바로 조합할 수 있다.
    """

    position: np.ndarray
    orientation: np.ndarray

    @property
    def position_norm(self):
        """위치 오차의 유클리드 크기를 미터 단위 실수로 반환한다."""
        return float(np.linalg.norm(self.position))

    @property
    def orientation_norm(self):
        """최단 회전 오차의 크기를 rad 단위 실수로 반환한다."""
        return float(np.linalg.norm(self.orientation))


@dataclass(frozen=True)
class VelocityTask:
    """이미 단위 정규화된 ``||matrix @ qdot - target||²`` 목적식 하나."""

    name: str
    matrix: np.ndarray
    target: np.ndarray


def pose_error(
    current_position, current_quaternion, target_position, target_quaternion
):
    """현재와 목표 pose로부터 공통 world-frame :class:`PoseError`를 계산한다.

    위치는 목표에서 현재를 빼고, 회전은 quaternion double-cover와 최단 회전을
    처리하는 :func:`rotations.shortest_orientation_error`에 위임한다. 입력 배열은
    수정하지 않는다.
    """
    position = np.asarray(target_position, dtype=float) - np.asarray(
        current_position, dtype=float
    )
    orientation = shortest_orientation_error(target_quaternion, current_quaternion)
    if position.shape != (3,) or orientation.shape != (3,):
        raise ValueError("pose position/orientation error must have shape (3,)")
    return PoseError(position=position, orientation=orientation)


def pose_velocity_command(
    error,
    *,
    position_gain,
    orientation_gain,
    current_twist=None,
    linear_velocity_damping=0.0,
    angular_velocity_damping=0.0,
    max_linear_speed=np.inf,
    max_angular_speed=np.inf,
):
    """pose 오차를 norm 제한된 world-frame 6차원 목표 twist로 변환한다.

    선속도와 각속도에 각각 ``gain * error - damping * measured_velocity``를 적용한 뒤
    방향을 유지하며 크기를 제한한다. ``current_twist``를 생략하면 속도 감쇠 없이
    오차 피드백만 계산한다. 반환 순서는 geometric Jacobian과 같은
    ``[linear(3), angular(3)]``이다.
    """
    if not isinstance(error, PoseError):
        raise TypeError("error must be a PoseError")
    if current_twist is None:
        twist = np.zeros(6)
    else:
        twist = np.asarray(current_twist, dtype=float)
        if twist.shape != (6,):
            raise ValueError(f"expected current_twist shape (6,), got {twist.shape}")

    linear = clip_norm(
        float(position_gain) * error.position
        - float(linear_velocity_damping) * twist[:3],
        float(max_linear_speed),
    )
    angular = clip_norm(
        float(orientation_gain) * error.orientation
        - float(angular_velocity_damping) * twist[3:],
        float(max_angular_speed),
    )
    return np.concatenate((linear, angular))


def normalized_weights(strengths, velocity_scales):
    """무차원 strength를 물리 속도 residual의 계수로 변환한다.

    각 residual을 대표 속도로 나눈 뒤 strength를 적용하므로 비용은
    ``strength * (velocity_error / velocity_scale)²``가 된다.
    """
    strengths = np.asarray(strengths, dtype=float)
    velocity_scales = np.asarray(velocity_scales, dtype=float)
    if strengths.shape != velocity_scales.shape:
        raise ValueError("task strengths and velocity scales must have the same shape")
    if np.any(strengths < 0.0) or np.any(velocity_scales <= 0.0):
        raise ValueError("task strengths must be non-negative and scales positive")
    return strengths / np.square(velocity_scales)


def velocity_task(name, jacobian, target_velocity, strengths, velocity_scales):
    """Jacobian task를 단위가 제거된 weighted least-squares 행으로 만든다."""
    jacobian = np.asarray(jacobian, dtype=float)
    target_velocity = np.asarray(target_velocity, dtype=float)
    if jacobian.ndim != 2 or target_velocity.shape != (jacobian.shape[0],):
        raise ValueError("incompatible velocity task Jacobian/target shapes")
    weights = normalized_weights(strengths, velocity_scales)
    if weights.shape != target_velocity.shape:
        raise ValueError("one task strength and scale are required per residual row")
    scale = np.sqrt(weights)
    return VelocityTask(
        name=str(name),
        matrix=scale[:, None] * jacobian,
        target=scale * target_velocity,
    )


def regularization_task(name, target_velocity, strengths, velocity_limits):
    """자유도별 damping/posture를 무차원 velocity task로 만든다."""
    target_velocity = np.asarray(target_velocity, dtype=float)
    if target_velocity.ndim != 1:
        raise ValueError("regularization target must be a vector")
    return velocity_task(
        name,
        np.eye(target_velocity.size),
        target_velocity,
        strengths,
        velocity_limits,
    )


def stack_velocity_tasks(task_list, variable_count):
    """독립 task들을 solver 입력 행렬/벡터 하나로 결합한다."""
    task_list = tuple(task_list)
    if not task_list:
        raise ValueError("at least one velocity task is required")
    variable_count = int(variable_count)
    for task in task_list:
        if not isinstance(task, VelocityTask):
            raise TypeError("task_list must contain VelocityTask values")
        if task.matrix.ndim != 2 or task.matrix.shape[1] != variable_count:
            raise ValueError(f"task {task.name!r} has an incompatible variable count")
        if task.target.shape != (task.matrix.shape[0],):
            raise ValueError(
                f"task {task.name!r} has incompatible matrix/target shapes"
            )
    return (
        np.vstack([task.matrix for task in task_list]),
        np.concatenate([task.target for task in task_list]),
    )


__all__ = [
    "PoseError",
    "VelocityTask",
    "normalized_weights",
    "pose_error",
    "pose_velocity_command",
    "regularization_task",
    "stack_velocity_tasks",
    "velocity_task",
]
