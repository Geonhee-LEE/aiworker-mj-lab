"""MJCF 트리 기구학 solver의 이전 API 호환 진입점.

새 코드는 :class:`ffw_sh5_grasp.kinematics.KinematicsSolver`를 직접 import한다.
Phase 테스트와 외부 스크립트를 깨뜨리지 않도록 기존 ``InverseKinematics`` 이름만
유지하며, FK·기하 Jacobian·IK 계산은 모두 같은 트리 solver에 위임한다.
"""

from . import solver as kinematics


DEFAULT_DAMPING = kinematics.DEFAULT_DAMPING
DEFAULT_MAX_JOINT_DELTA = kinematics.DEFAULT_MAX_JOINT_DELTA
DEFAULT_MAX_ITER = kinematics.DEFAULT_MAX_ITER
POS_TOL = kinematics.POSITION_TOLERANCE
ORI_TOL = kinematics.ORIENTATION_TOLERANCE


class InverseKinematics(kinematics.KinematicsSolver):
    """:class:`kinematics.KinematicsSolver`의 이전 이름 호환 클래스."""


__all__ = [
    "DEFAULT_DAMPING",
    "DEFAULT_MAX_JOINT_DELTA",
    "DEFAULT_MAX_ITER",
    "InverseKinematics",
    "ORI_TOL",
    "POS_TOL",
]
