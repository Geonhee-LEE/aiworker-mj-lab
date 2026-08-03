"""단일 site FK/Jacobian과 반복 IK solver의 공개 진입점.

세부 구현은 다음 파일로 분리되어 있다.

- :mod:`.rotations`: 회전 행렬과 쿼터니언 수학
- :mod:`.tasks`: 모든 IK 경로가 공유하는 pose 오차와 Cartesian 속도 명령
- :mod:`.tree`: MJCF body-joint-site 트리와 FK/Jacobian
- :mod:`.collision`: live MuJoCo geometry 거리와 충돌 gradient

기존 호출 코드가 계속 동작하도록 주요 타입과 함수는 이 모듈에서도 다시 노출한다.
"""

from pathlib import Path

import mujoco
import numpy as np

from ..config import SETTINGS

# 기존 ``import kinematics`` 호출부를 깨지 않도록 공개 타입과 함수만 재노출한다.
from .collision import (
    CollisionConstraint,
    CollisionPair,
    collision_distance_gradient,
    default_collision_pairs,
)
from .tree import (
    KinematicBody,
    KinematicJoint,
    KinematicSite,
    KinematicTree,
    SiteKinematics,
)
from .rotations import (
    normalize_quaternion,
    shortest_orientation_error,
)
from .tasks import PoseError, pose_error, pose_velocity_command


DEFAULT_DAMPING = SETTINGS.number("kinematics.damping", positive=True)
DEFAULT_MAX_JOINT_DELTA = SETTINGS.number(
    "kinematics.max_joint_delta_rad", positive=True)
DEFAULT_MAX_ITER = SETTINGS.integer("kinematics.max_iterations", minimum=1)
POSITION_TOLERANCE = SETTINGS.number(
    "kinematics.position_tolerance_m", positive=True)
ORIENTATION_TOLERANCE = SETTINGS.number(
    "kinematics.orientation_tolerance_rad", positive=True)
ORIENTATION_COST_WEIGHT = SETTINGS.number(
    "kinematics.orientation_cost_weight", minimum=0.0)
# Backtracking 규칙은 사용자 목표가 아니라 solver 수렴 구현의 일부다.
BACKTRACKING_STEPS = 6
BACKTRACKING_RATIO = 0.5
MULTISTART_RESTARTS = SETTINGS.integer(
    "kinematics.multistart.restarts", minimum=0)
MULTISTART_MAX_ITER = SETTINGS.integer(
    "kinematics.multistart.max_iterations", minimum=1)
MULTISTART_POSITION_TOLERANCE = SETTINGS.number(
    "kinematics.multistart.success_position_tolerance_m", positive=True)
MULTISTART_ORIENTATION_TOLERANCE = np.radians(SETTINGS.number(
    "kinematics.multistart.success_orientation_tolerance_deg", positive=True))
class KinematicsSolver:
    """트리 FK를 사용하는 position-priority damped least-squares IK solver."""

    def __init__(self, model, site_name, joint_names, damping=DEFAULT_DAMPING,
                 max_joint_delta=DEFAULT_MAX_JOINT_DELTA, *, tree=None):
        """제어할 site와 scalar joint를 검증하고 반복 IK에 필요한 배열을 준비한다.

        ``model``은 이름과 주소 확인에 사용한다. ``site_name``은 목표 말단 site,
        ``joint_names``는 해에 포함할 hinge/slide 관절 순서다. ``tree``를 생략하면
        모델에서 새 :class:`KinematicTree`를 만들며, 전달하면 기존 트리를 공유한다.
        """
        self.model = model
        self.tree = KinematicTree(model) if tree is None else tree
        joint_names = tuple(joint_names)
        try:
            site = self.tree.site_by_name[site_name]
            joints = tuple(self.tree.joint_by_name[name] for name in joint_names)
        except KeyError as error:
            raise ValueError(
                "kinematics solver references a site or joint absent from the model: "
                f"{error.args[0]!r}") from error

        scalar_joint_types = {
            int(mujoco.mjtJoint.mjJNT_HINGE),
            int(mujoco.mjtJoint.mjJNT_SLIDE),
        }
        unsupported = [
            joint.name for joint in joints if joint.kind not in scalar_joint_types]
        if unsupported:
            raise ValueError(
                "controlled joints must be scalar hinge/slide joints: "
                + ", ".join(unsupported))
        if len({joint.id for joint in joints}) != len(joints):
            raise ValueError("controlled joint names must be unique")

        self.site_name = site_name
        self.joint_names = joint_names
        self.site_id = site.id
        self.joint_ids = np.array([joint.id for joint in joints], dtype=int)
        self.dof_ids = np.array([joint.dof_adr for joint in joints], dtype=int)
        self.qpos_adrs = np.array([joint.qpos_adr for joint in joints], dtype=int)
        self.joint_ranges = np.array([joint.range for joint in joints], dtype=float)
        self.joint_limited = np.array(
            [joint.limited for joint in joints], dtype=bool)
        self.damping = float(damping)
        self.max_joint_delta = float(max_joint_delta)
        self.n = len(joints)
        if self.damping <= 0.0 or self.max_joint_delta <= 0.0:
            raise ValueError("damping and max_joint_delta must be positive")

    @classmethod
    def from_mjcf(cls, path, site_name, joint_names, **kwargs):
        """MJCF 파일을 컴파일하고 해당 모델의 solver를 생성한다."""
        model = mujoco.MjModel.from_xml_path(str(Path(path)))
        return cls(model, site_name, joint_names, **kwargs)

    def _configuration(self, q, context_qpos=None):
        """제어 joint 값과 나머지 모델 상태를 합친 전체 qpos를 만든다."""
        q = np.asarray(q, dtype=float)
        if q.shape != (self.n,):
            raise ValueError(f"expected {self.n} joint positions, got {q.shape}")
        if context_qpos is None:
            qpos = self.tree.qpos0.copy()
        else:
            context = np.asarray(context_qpos, dtype=float)
            if context.shape != (self.tree.nq,):
                raise ValueError(
                    f"expected context_qpos shape ({self.tree.nq},), got {context.shape}")
            qpos = context.copy()
        qpos[self.qpos_adrs] = self._clamp_to_limits(q)
        return qpos

    def forward(self, q, context_qpos=None):
        """site FK와 선택된 joint 열의 6×N world-frame Jacobian을 계산한다."""
        return self.tree.forward_site(
            self._configuration(q, context_qpos), self.site_id, self.joint_ids)

    def forward_kinematics(self, q, context_qpos=None):
        """기존 호출부를 위한 :meth:`forward` 호환 이름."""
        return self.forward(q, context_qpos)

    def _clamp_to_limits(self, q):
        """제한이 설정된 관절만 MJCF 범위로 자른 새 관절 벡터를 반환한다."""
        result = np.asarray(q, dtype=float).copy()
        result[self.joint_limited] = np.clip(
            result[self.joint_limited],
            self.joint_ranges[self.joint_limited, 0],
            self.joint_ranges[self.joint_limited, 1])
        return result

    def solve_pose(self, q_init, target_pos, target_quat, max_iter=DEFAULT_MAX_ITER,
                   pos_tol=POSITION_TOLERANCE, ori_tol=ORIENTATION_TOLERANCE,
                   ori_weight=ORIENTATION_COST_WEIGHT, context_qpos=None):
        """position 우선 DLS와 backtracking으로 목표 pose를 푼다.

        먼저 position을 풀고 orientation 보정은 position task의 null space 방향으로
        투영한다. 각 후보 step을 실제 트리 FK로 평가해 비용이 감소하는 값을 선택한다.
        """
        q = self._clamp_to_limits(np.asarray(q_init, dtype=float))
        if q.shape != (self.n,):
            raise ValueError(f"expected {self.n} initial joint positions, got {q.shape}")
        state = self.forward(q, context_qpos)
        error = pose_error(
            state.position, state.quaternion, target_pos, target_quat)
        position_error, orientation_error = error.position, error.orientation
        position_norm = error.position_norm
        orientation_norm = error.orientation_norm
        damping_squared = self.damping ** 2
        identity3 = np.eye(3)

        for _ in range(max(0, int(max_iter))):
            if position_norm < pos_tol and orientation_norm < ori_tol:
                break
            position_jacobian = state.jacobian[:3]
            rotation_jacobian = state.jacobian[3:]
            position_system = (
                position_jacobian @ position_jacobian.T
                + damping_squared * identity3)

            position_delta = position_jacobian.T @ np.linalg.solve(
                position_system, position_error)
            orientation_gradient = rotation_jacobian.T @ orientation_error
            projected_gradient = np.linalg.solve(
                position_system, position_jacobian @ orientation_gradient)
            orientation_delta = (
                orientation_gradient - position_jacobian.T @ projected_gradient)
            full_delta = np.clip(
                position_delta + orientation_delta,
                -self.max_joint_delta, self.max_joint_delta)

            current_cost = position_norm + ori_weight * orientation_norm
            best = None
            step = 1.0
            for _ in range(BACKTRACKING_STEPS):
                candidate_q = self._clamp_to_limits(q + step * full_delta)
                candidate_state = self.forward(candidate_q, context_qpos)
                candidate_error = pose_error(
                    candidate_state.position, candidate_state.quaternion,
                    target_pos, target_quat)
                candidate_position_error = candidate_error.position
                candidate_orientation_error = candidate_error.orientation
                candidate_position_norm = candidate_error.position_norm
                candidate_orientation_norm = candidate_error.orientation_norm
                cost = (candidate_position_norm
                        + ori_weight * candidate_orientation_norm)
                if best is None or cost < best[0]:
                    best = (
                        cost, candidate_q, candidate_state,
                        candidate_position_error, candidate_orientation_error,
                        candidate_position_norm, candidate_orientation_norm)
                if cost < current_cost:
                    break
                step *= BACKTRACKING_RATIO

            (_, q, state, position_error, orientation_error,
             position_norm, orientation_norm) = best
        return q, position_norm, orientation_norm

    def solve_pose_multistart(self, q_init, target_pos, target_quat, rng,
                              n_restarts=MULTISTART_RESTARTS,
                              max_iter=MULTISTART_MAX_ITER,
                              success_pos_tol=MULTISTART_POSITION_TOLERANCE,
                              success_ori_tol=MULTISTART_ORIENTATION_TOLERANCE,
                              context_qpos=None):
        """여러 유효 초기 자세에서 재시도해 local minimum을 피한다."""
        initial = np.asarray(q_init, dtype=float)
        if initial.shape != (self.n,):
            raise ValueError(
                f"expected {self.n} initial joint positions, got {initial.shape}")
        lower = np.where(
            self.joint_limited, self.joint_ranges[:, 0], initial - np.pi)
        upper = np.where(
            self.joint_limited, self.joint_ranges[:, 1], initial + np.pi)
        candidates = [initial]
        candidates.extend(
            rng.uniform(lower, upper) for _ in range(max(0, int(n_restarts))))
        best = None
        for candidate in candidates:
            q, position_error, orientation_error = self.solve_pose(
                candidate, target_pos, target_quat, max_iter=max_iter,
                context_qpos=context_qpos)
            if (position_error < success_pos_tol
                    and orientation_error < success_ori_tol):
                return q, position_error, orientation_error, True
            if (best is None
                    or position_error + orientation_error < best[1] + best[2]):
                best = (q, position_error, orientation_error)
        return best[0], best[1], best[2], False


__all__ = [
    "CollisionConstraint",
    "CollisionPair",
    "DEFAULT_DAMPING",
    "DEFAULT_MAX_ITER",
    "DEFAULT_MAX_JOINT_DELTA",
    "KinematicBody",
    "KinematicJoint",
    "KinematicSite",
    "KinematicTree",
    "KinematicsSolver",
    "ORIENTATION_TOLERANCE",
    "POSITION_TOLERANCE",
    "SiteKinematics",
    "PoseError",
    "collision_distance_gradient",
    "default_collision_pairs",
    "normalize_quaternion",
    "pose_error",
    "pose_velocity_command",
    "shortest_orientation_error",
]
