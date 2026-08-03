"""단일 팔·전신·양손 IK가 공유하는 pose 오차와 속도 명령 계산.

IK마다 자유도와 제약 조건은 달라도 위치 오차는 ``target - current``, 자세 오차는
world frame의 최단 회전 벡터라는 같은 규칙을 사용해야 한다. 이 모듈은 그 규칙과
오차를 bounded Cartesian velocity로 바꾸는 계산을 한곳에 둔다. 로봇 모델이나
MuJoCo 상태를 직접 알지 않으므로 오프라인 IK와 실시간 제어에서 함께 사용할 수 있다.
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


def pose_error(current_position, current_quaternion,
               target_position, target_quaternion):
    """현재와 목표 pose로부터 공통 world-frame :class:`PoseError`를 계산한다.

    위치는 목표에서 현재를 빼고, 회전은 quaternion double-cover와 최단 회전을
    처리하는 :func:`rotations.shortest_orientation_error`에 위임한다. 입력 배열은
    수정하지 않는다.
    """
    position = (
        np.asarray(target_position, dtype=float)
        - np.asarray(current_position, dtype=float)
    )
    orientation = shortest_orientation_error(
        target_quaternion, current_quaternion)
    if position.shape != (3,) or orientation.shape != (3,):
        raise ValueError("pose position/orientation error must have shape (3,)")
    return PoseError(position=position, orientation=orientation)


def pose_velocity_command(error, *, position_gain, orientation_gain,
                          current_twist=None,
                          linear_velocity_damping=0.0,
                          angular_velocity_damping=0.0,
                          max_linear_speed=np.inf,
                          max_angular_speed=np.inf):
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


__all__ = ["PoseError", "pose_error", "pose_velocity_command"]
