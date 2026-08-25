"""MuJoCo 모델 주소와 애플리케이션 입출력 스냅샷.

``ModelBindings``는 MJCF 이름을 매 frame 다시 찾지 않도록 컴파일된 주소를 묶는다.
``RobotObservation``, ``TaskCommand``와 ``ControlCommand``는 각각 측정 상태, 작업 목표,
물리 적용 단계의 명령을 나타낸다. 배열은 생성할 때 복사하므로 이후 MuJoCo step이나 UI
변경의 영향을 받지 않는다.
"""

import math
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping

import numpy as np

from ..control.base import BodyTwist


def _readonly_array(values):
    """입력과 메모리를 공유하지 않는 읽기 전용 float 배열을 만든다."""
    result = np.asarray(values, dtype=float).copy()
    result.setflags(write=False)
    return result


def _readonly_arrays(values):
    """문자열 키 배열 mapping을 복사하고 읽기 전용 view로 감싼다."""
    return MappingProxyType(
        {name: _readonly_array(value) for name, value in values.items()}
    )


def _readonly_scalars(values):
    """문자열 키 수치 mapping을 독립된 읽기 전용 mapping으로 만든다."""
    return MappingProxyType({name: float(value) for name, value in values.items()})


@dataclass(frozen=True)
class BaseBindings:
    """평면 베이스 관절의 qpos와 qvel 주소."""

    x_qpos: int
    y_qpos: int
    yaw_qpos: int
    x_dof: int
    y_dof: int
    yaw_dof: int


@dataclass(frozen=True)
class WheelBinding:
    """한 스워브 모듈의 actuator와 feedback 주소."""

    steer_actuator: int
    drive_actuator: int
    steer_qpos: int
    drive_dof: int


@dataclass(frozen=True)
class MarkerBindings:
    """손 목표와 가상 물체 marker의 mocap·geom·site 주소 및 원래 색상."""

    hand_mocap_ids: Mapping[str, int]
    virtual_mocap_id: int
    virtual_geom_id: int
    virtual_site_id: int
    virtual_geom_rgba: np.ndarray
    virtual_site_rgba: np.ndarray

    def __post_init__(self):
        object.__setattr__(
            self,
            "hand_mocap_ids",
            MappingProxyType(
                {name: int(value) for name, value in self.hand_mocap_ids.items()}
            ),
        )
        object.__setattr__(
            self, "virtual_geom_rgba", _readonly_array(self.virtual_geom_rgba)
        )
        object.__setattr__(
            self, "virtual_site_rgba", _readonly_array(self.virtual_site_rgba)
        )


@dataclass(frozen=True)
class ModelBindings:
    """프레임 루프가 반복 사용하는 MuJoCo 객체 주소 모음."""

    lift_actuator: int
    lift_qpos: int
    base: BaseBindings
    wheels: Mapping[str, WheelBinding]
    monitor_qpos: Mapping[str, int]
    monitor_ranges: Mapping[str, np.ndarray]
    markers: MarkerBindings
    can_joint: int
    can_geom: int

    def __post_init__(self):
        object.__setattr__(self, "wheels", MappingProxyType(dict(self.wheels)))
        object.__setattr__(
            self,
            "monitor_qpos",
            MappingProxyType(
                {name: int(value) for name, value in self.monitor_qpos.items()}
            ),
        )
        object.__setattr__(
            self, "monitor_ranges", _readonly_arrays(self.monitor_ranges)
        )


@dataclass(frozen=True)
class RobotObservation:
    """한 시점의 로봇 상태. IL 데이터로 저장해도 live state와 공유되지 않는다."""

    time: float
    qpos: np.ndarray
    qvel: np.ndarray
    base_twist: BodyTwist
    hand_positions: Mapping[str, np.ndarray]
    hand_quaternions: Mapping[str, np.ndarray]

    def __post_init__(self):
        object.__setattr__(self, "time", float(self.time))
        object.__setattr__(self, "qpos", _readonly_array(self.qpos))
        object.__setattr__(self, "qvel", _readonly_array(self.qvel))
        object.__setattr__(
            self, "hand_positions", _readonly_arrays(self.hand_positions)
        )
        object.__setattr__(
            self, "hand_quaternions", _readonly_arrays(self.hand_quaternions)
        )

    @classmethod
    def capture(cls, app):
        """현재 MuJoCo 상태와 양손 site pose를 복사해 관측 스냅샷을 만든다."""
        bindings = app.bindings.base
        data = app.data
        yaw = float(data.qpos[bindings.yaw_qpos])
        cosine, sine = math.cos(yaw), math.sin(yaw)
        vx_world = float(data.qvel[bindings.x_dof])
        vy_world = float(data.qvel[bindings.y_dof])
        hand_states = {
            side: app.whole_body_solver.site_state(data, side)
            for side in app.whole_body_solver.site_ids
        }
        return cls(
            time=float(data.time),
            qpos=data.qpos,
            qvel=data.qvel,
            base_twist=BodyTwist(
                cosine * vx_world + sine * vy_world,
                -sine * vx_world + cosine * vy_world,
                float(data.qvel[bindings.yaw_dof]),
            ),
            hand_positions={
                side: state.position for side, state in hand_states.items()
            },
            hand_quaternions={
                side: state.quaternion for side, state in hand_states.items()
            },
        )


@dataclass(frozen=True)
class TaskCommand:
    """UI나 상위 정책이 요청한 task-space 목표 묶음."""

    hand_poses: Mapping[str, tuple]
    lift_position: float
    base_twist: BodyTwist = field(default_factory=BodyTwist)
    grasp: Mapping[str, float] = field(default_factory=dict)
    thumb: Mapping[str, float] = field(default_factory=dict)

    def __post_init__(self):
        object.__setattr__(
            self,
            "hand_poses",
            MappingProxyType(
                {
                    side: (_readonly_array(position), _readonly_array(quaternion))
                    for side, (position, quaternion) in self.hand_poses.items()
                }
            ),
        )
        object.__setattr__(self, "lift_position", float(self.lift_position))
        object.__setattr__(self, "grasp", _readonly_scalars(self.grasp))
        object.__setattr__(self, "thumb", _readonly_scalars(self.thumb))

    @classmethod
    def create(
        cls, hand_poses, lift_position, *, base_twist=None, grasp=None, thumb=None
    ):
        """pose와 손 명령을 복사해 이후 UI 변경과 분리된 task 명령을 만든다."""
        return cls(
            hand_poses=hand_poses,
            lift_position=lift_position,
            base_twist=BodyTwist() if base_twist is None else base_twist,
            grasp={} if grasp is None else grasp,
            thumb={} if thumb is None else thumb,
        )


@dataclass(frozen=True)
class ControlCommand:
    """controller 계산을 마친 물리 적용 단계의 한 frame 명령."""

    arm_positions: Mapping[str, np.ndarray]
    lift_position: float
    base_twist: BodyTwist
    wheel_commands: Mapping[str, tuple]
    grasp: Mapping[str, float]
    thumb: Mapping[str, float]

    def __post_init__(self):
        object.__setattr__(self, "arm_positions", _readonly_arrays(self.arm_positions))
        object.__setattr__(self, "lift_position", float(self.lift_position))
        object.__setattr__(
            self,
            "wheel_commands",
            MappingProxyType(
                {
                    name: (float(command[0]), float(command[1]))
                    for name, command in self.wheel_commands.items()
                }
            ),
        )
        object.__setattr__(self, "grasp", _readonly_scalars(self.grasp))
        object.__setattr__(self, "thumb", _readonly_scalars(self.thumb))

    @classmethod
    def create(
        cls, arm_positions, lift_position, base_twist, wheel_commands, grasp, thumb
    ):
        """모든 배열과 mapping을 복사해 물리 적용 중 바뀌지 않는 명령을 만든다."""
        return cls(
            arm_positions=arm_positions,
            lift_position=lift_position,
            base_twist=base_twist,
            wheel_commands=wheel_commands,
            grasp=grasp,
            thumb=thumb,
        )


__all__ = [
    "BaseBindings",
    "ControlCommand",
    "MarkerBindings",
    "ModelBindings",
    "RobotObservation",
    "TaskCommand",
    "WheelBinding",
]
