"""FFW-SH5 이동형 로봇의 MuJoCo/NumPy 기반 differential whole-body IK.

제어하는 generalized velocity 순서는 다음과 같다::

    [base_x, base_y, base_yaw, lift, right_arm(7), left_arm(7)]

이 파일은 task 조립과 로봇 상태 관리만 담당한다. 순수 bounded least-squares 계산은
``control.optimization``, 양손 상대 pose 계산은 ``control.bimanual``, 충돌 거리
계산은 ``kinematics.collision``에 분리되어 있다.
"""

from dataclasses import dataclass, field
import math

import mujoco
import numpy as np

from .base import BodyTwist
from . import bimanual, optimization
from ..config import SETTINGS
from ..kinematics import collision, rotations
from ..kinematics.solver import KinematicsSolver
from ..kinematics.tree import KinematicTree


BASE_JOINTS = ("base_x", "base_y", "base_yaw")
DEFAULT_VELOCITY_LIMITS = {
    name: SETTINGS.number(f"whole_body_ik.velocity_limits.{name}", positive=True)
    for name in ("base_x", "base_y", "base_yaw", "lift_joint")
}
DEFAULT_ARM_VELOCITY_LIMIT = SETTINGS.number(
    "whole_body_ik.velocity_limits.arm_default", positive=True)
DEFAULT_POSITION_WEIGHT = SETTINGS.number("whole_body_ik.position_weight", positive=True)
DEFAULT_ORIENTATION_WEIGHT = SETTINGS.number("whole_body_ik.orientation_weight", positive=True)
DEFAULT_POSITION_GAIN = SETTINGS.number("whole_body_ik.position_gain", positive=True)
DEFAULT_ORIENTATION_GAIN = SETTINGS.number("whole_body_ik.orientation_gain", positive=True)
DEFAULT_LINEAR_VELOCITY_DAMPING = SETTINGS.number(
    "whole_body_ik.linear_velocity_damping", minimum=0.0)
DEFAULT_ANGULAR_VELOCITY_DAMPING = SETTINGS.number(
    "whole_body_ik.angular_velocity_damping", minimum=0.0)
DEFAULT_POSTURE_GAIN = SETTINGS.number("whole_body_ik.posture_gain", minimum=0.0)
DEFAULT_JOINT_LIMIT_MARGIN = SETTINGS.number(
    "whole_body_ik.joint_limit_margin_rad", minimum=0.0)
DEFAULT_JOINT_LIMIT_GAIN = SETTINGS.number("whole_body_ik.joint_limit_gain", positive=True)
DEFAULT_RIGID_GRASP_WEIGHT = SETTINGS.number("whole_body_ik.rigid_grasp_weight", positive=True)
DEFAULT_COLLISION_AVOIDANCE = SETTINGS.get("whole_body_ik.collision_avoidance")
DEFAULT_COLLISION_BUFFER = SETTINGS.number("whole_body_ik.collision_buffer_m", positive=True)
DEFAULT_COLLISION_SAFE_DISTANCE = SETTINGS.number(
    "whole_body_ik.collision_safe_distance_m", minimum=0.0)
DEFAULT_COLLISION_BARRIER_GAIN = SETTINGS.number(
    "whole_body_ik.collision_barrier_gain", positive=True)
DEFAULT_COLLISION_SLACK_WEIGHT = SETTINGS.number(
    "whole_body_ik.collision_slack_weight", positive=True)
SIDES = ("r", "l")


@dataclass
class WholeBodyCommand:
    base_twist: BodyTwist = BodyTwist()
    arm_positions: dict = field(default_factory=dict)
    lift_position: float = 0.0
    position_errors: dict = field(default_factory=dict)
    orientation_errors: dict = field(default_factory=dict)
    generalized_velocity: np.ndarray = field(default_factory=lambda: np.zeros(0))
    minimum_collision_distance: float = math.inf
    active_collision_pairs: tuple = ()
    collision_constraint_violation: float = 0.0


class WholeBodyIK:
    """base, lift, 양팔을 함께 푸는 weighted bounded differential IK."""

    def __init__(self, model, site_names, arm_joint_names, *,
                 position_weight=DEFAULT_POSITION_WEIGHT,
                 orientation_weight=DEFAULT_ORIENTATION_WEIGHT,
                 position_gain=DEFAULT_POSITION_GAIN,
                 orientation_gain=DEFAULT_ORIENTATION_GAIN,
                 linear_velocity_damping=DEFAULT_LINEAR_VELOCITY_DAMPING,
                 angular_velocity_damping=DEFAULT_ANGULAR_VELOCITY_DAMPING,
                 posture_gain=DEFAULT_POSTURE_GAIN,
                 joint_limit_margin=DEFAULT_JOINT_LIMIT_MARGIN,
                 joint_limit_gain=DEFAULT_JOINT_LIMIT_GAIN,
                 rigid_grasp_weight=DEFAULT_RIGID_GRASP_WEIGHT,
                 collision_avoidance=DEFAULT_COLLISION_AVOIDANCE,
                 collision_pairs=None,
                 collision_buffer=DEFAULT_COLLISION_BUFFER,
                 collision_safe_distance=DEFAULT_COLLISION_SAFE_DISTANCE,
                 collision_barrier_gain=DEFAULT_COLLISION_BARRIER_GAIN,
                 collision_slack_weight=DEFAULT_COLLISION_SLACK_WEIGHT):
        self.model = model
        self.site_ids = {
            side: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
            for side, name in site_names.items()
        }
        self.arm_joint_names = {side: tuple(names) for side, names in arm_joint_names.items()}
        self.joint_names = (BASE_JOINTS + ("lift_joint",)
                            + self.arm_joint_names["r"] + self.arm_joint_names["l"])
        self.joint_ids = np.array([
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.joint_names
        ], dtype=int)
        if np.any(self.joint_ids < 0) or any(site_id < 0 for site_id in self.site_ids.values()):
            raise ValueError("whole-body IK references a joint or site absent from the model")
        self.qpos_adrs = np.array([model.jnt_qposadr[jid] for jid in self.joint_ids], dtype=int)
        self.dof_ids = np.array([model.jnt_dofadr[jid] for jid in self.joint_ids], dtype=int)
        self.index = {name: i for i, name in enumerate(self.joint_names)}
        self.side_indices = {
            side: np.array([self.index[name] for name in names], dtype=int)
            for side, names in self.arm_joint_names.items()
        }
        # MJCF topology는 한 번만 읽고 불변 tree를 양쪽 end-effector solver가 공유한다.
        self.kinematic_tree = KinematicTree(model)
        self.kinematics_solvers = {
            side: KinematicsSolver(
                model, site_names[side], self.joint_names, tree=self.kinematic_tree)
            for side in self.site_ids
        }

        self.position_weight = float(position_weight)
        self.orientation_weight = float(orientation_weight)
        self.position_gain = float(position_gain)
        self.orientation_gain = float(orientation_gain)
        self.linear_velocity_damping = float(linear_velocity_damping)
        self.angular_velocity_damping = float(angular_velocity_damping)
        self.posture_gain = float(posture_gain)
        self.joint_limit_margin = float(joint_limit_margin)
        self.joint_limit_gain = float(joint_limit_gain)
        self.rigid_grasp_weight = float(rigid_grasp_weight)
        self.collision_buffer = float(collision_buffer)
        self.collision_safe_distance = float(collision_safe_distance)
        self.collision_barrier_gain = float(collision_barrier_gain)
        self.collision_slack_weight = float(collision_slack_weight)
        if self.collision_safe_distance < 0.0:
            raise ValueError("collision_safe_distance must be non-negative")
        if self.collision_buffer <= self.collision_safe_distance:
            raise ValueError("collision_buffer must exceed collision_safe_distance")
        if self.collision_barrier_gain <= 0.0 or self.collision_slack_weight <= 0.0:
            raise ValueError("collision barrier gain/weight must be positive")
        if collision_avoidance:
            self.collision_pairs = tuple(
                collision.default_collision_pairs(model)
                if collision_pairs is None else collision_pairs)
        else:
            self.collision_pairs = ()
        self.max_task_linear_speed = SETTINGS.number(
            "whole_body_ik.max_task_linear_speed_m_s", positive=True)
        self.max_task_angular_speed = SETTINGS.number(
            "whole_body_ik.max_task_angular_speed_rad_s", positive=True)
        self.base_linear_acceleration_limit = SETTINGS.number(
            "whole_body_ik.base_linear_acceleration_limit_m_s2", positive=True)
        self.base_angular_acceleration_limit = SETTINGS.number(
            "whole_body_ik.base_angular_acceleration_limit_rad_s2", positive=True)
        self.base_position_fade_distance = SETTINGS.number(
            "whole_body_ik.base_position_fade_distance_m", positive=True)
        self.base_orientation_fade_angle = SETTINGS.number(
            "whole_body_ik.base_orientation_fade_angle_rad", positive=True)
        self.common_base_position_gain = SETTINGS.number(
            "whole_body_ik.common_base.position_gain", positive=True)
        self.common_base_yaw_gain = SETTINGS.number(
            "whole_body_ik.common_base.yaw_gain", positive=True)
        self.common_base_yaw_deadband = SETTINGS.number(
            "whole_body_ik.common_base.yaw_deadband_rad", minimum=0.0)
        self.common_base_yaw_speed_limit = SETTINGS.number(
            "whole_body_ik.common_base.yaw_speed_limit_rad_s", positive=True)
        self.common_base_weights = np.asarray(
            SETTINGS.get("whole_body_ik.common_base.task_weights"), dtype=float)
        if np.any(self.common_base_weights <= 0.0):
            raise ValueError("whole_body_ik.common_base.task_weights는 모두 양수여야 합니다.")
        self._previous_base_velocity_world = np.zeros(3)
        self._last_solve_time = None
        self._reference_base_yaw = None
        self._reference_base_xy = None
        self._reference_hand_positions = {}
        self._reference_hand_quaternions = {}
        self._rigid_grasp_reference = None

        # 양손의 공통 이동에 base/lift가 참여하도록 해당 DOF의 정규화 비용을 낮춘다.
        damping_base_lift = SETTINGS.get("whole_body_ik.damping_weights.base_lift")
        posture_base_lift = SETTINGS.get("whole_body_ik.posture_weights.base_lift")
        self.damping_weights = np.array(
            damping_base_lift
            + [SETTINGS.number("whole_body_ik.damping_weights.arm", minimum=0.0)] * 14,
            dtype=float)
        self.posture_weights = np.array(
            posture_base_lift
            + [SETTINGS.number("whole_body_ik.posture_weights.arm", minimum=0.0)] * 14,
            dtype=float)
        if np.any(self.damping_weights < 0.0) or np.any(self.posture_weights < 0.0):
            raise ValueError("전신 IK의 damping/posture 가중치는 음수일 수 없습니다.")

        self.velocity_limits = np.array([
            DEFAULT_VELOCITY_LIMITS.get(name, DEFAULT_ARM_VELOCITY_LIMIT)
            for name in self.joint_names
        ], dtype=float)
        self.position_limited = np.array([bool(model.jnt_limited[jid]) for jid in self.joint_ids])
        self.position_ranges = np.array([model.jnt_range[jid] for jid in self.joint_ids], dtype=float)

    def rebase(self, data, target_poses=None):
        """현재 base pose를 이후 양손 공통 이동의 기준점으로 재설정한다.

        수동 주행 중에는 target frame도 chassis와 함께 움직인다. 제어권을 넘길 때
        시작 시점의 reference를 그대로 쓰면 이 이동을 새 task로 해석해 base가 원래
        위치로 돌아가려 한다. 현재 target pose를 다시 기준으로 삼아 target delta가
        0일 때 현 위치를 유지하도록 한다.
        """
        current_q = np.asarray(data.qpos[self.qpos_adrs], dtype=float)
        self._reference_base_yaw = float(current_q[self.index["base_yaw"]])
        self._reference_base_xy = current_q[:2].copy()
        target_poses = target_poses or {}
        for side in self.site_ids:
            if side in target_poses:
                position, quaternion = target_poses[side]
                position = np.asarray(position, dtype=float).copy()
                quaternion = rotations.normalize_quaternion(quaternion)
            else:
                state = self.site_state(data, side, current_q)
                position = state.position
                quaternion = state.quaternion
            self._reference_hand_positions[side] = position
            self._reference_hand_quaternions[side] = quaternion
        self._previous_base_velocity_world[:] = 0.0
        self._last_solve_time = None

    def set_rigid_grasp(self, data, active):
        """오른손 frame에서 본 현재 왼손 pose를 캡처하거나 해제한다.

        Cyclo의 bimanual MoveL controller는 6차원 상대 pose equality를 QP에 넣는다.
        여기서는 같은 velocity-level geometry를 강한 least-squares task로 구성해
        OSQP나 ROS 의존성을 추가하지 않는다.
        """
        if not active:
            self._rigid_grasp_reference = None
            return
        current_q = np.asarray(data.qpos[self.qpos_adrs], dtype=float)
        right = self.site_state(data, "r", current_q)
        left = self.site_state(data, "l", current_q)
        self._rigid_grasp_reference = bimanual.capture_reference(
            right, left)

    def solve(self, data, target_poses, dt, *, active_sides=SIDES,
              arm_nominal=None, lift_nominal=None, rigid_grasp=False,
              whole_body_enabled=True):
        """한 control frame에 적용할 actuator-level 목표를 반환한다.

        ``target_poses``는 ``"r"``/``"l"``을 world ``(position, quaternion)``에
        대응시킨다. ``active_sides``에서 빠진 FK-mode 팔은 계산에 참여하지 않는다.
        ``whole_body_enabled=False``이면 base와 lift 속도를 0으로 고정하되 관절 한계와
        충돌 constraint는 그대로 유지한다.
        """
        dt = max(float(dt), 1e-5)
        active_sides = tuple(side for side in active_sides if side in self.site_ids)
        current_q = np.asarray(data.qpos[self.qpos_adrs], dtype=float).copy()
        if self._reference_base_yaw is None:
            self.rebase(data)
        rows, rhs = [], []
        dual_base_request = None
        position_errors, orientation_errors = {}, {}

        site_states = {}
        for side in active_sides:
            target_pos, target_quat = target_poses[side]
            state = self.site_state(data, side, current_q)
            site_states[side] = state
            jac = state.jacobian
            site_velocity = jac @ data.qvel[self.dof_ids]

            current_pos = state.position
            pos_error = np.asarray(target_pos, dtype=float) - current_pos
            ori_error_world = rotations.shortest_orientation_error(
                target_quat, state.quaternion)

            desired = np.concatenate((
                rotations.clip_norm(
                    self.position_gain * pos_error
                    - self.linear_velocity_damping * site_velocity[:3],
                    self.max_task_linear_speed),
                rotations.clip_norm(
                    self.orientation_gain * ori_error_world
                    - self.angular_velocity_damping * site_velocity[3:],
                    self.max_task_angular_speed),
            ))
            weights = np.sqrt(np.array(
                [self.position_weight] * 3 + [self.orientation_weight] * 3))
            rows.append(weights[:, None] * jac)
            rhs.append(weights * desired)
            position_errors[side] = float(np.linalg.norm(pos_error))
            orientation_errors[side] = float(np.linalg.norm(ori_error_world))

        if rigid_grasp and all(side in site_states for side in SIDES):
            if self._rigid_grasp_reference is None:
                self.set_rigid_grasp(data, True)
            grasp_jacobian, grasp_velocity = bimanual.rigid_grasp_task(
                self._rigid_grasp_reference,
                site_states,
                dt,
                self.max_task_linear_speed,
                self.max_task_angular_speed,
            )
            weight = math.sqrt(self.rigid_grasp_weight)
            rows.append(weight * grasp_jacobian)
            rhs.append(weight * grasp_velocity)

        # 양손 평균 오차로 base를 먼저 servo하고 나머지 개별 오차는 lift/팔이 푼다.
        # minimum norm에만 맡기면 swerve가 조향 중일 때 14개 팔 열이 공통 오차의
        # 부호를 바꾸어 chassis 방향이 반복 반전될 수 있다.
        if whole_body_enabled and all(side in position_errors for side in SIDES):
            reference_centroid = 0.5 * (
                self._reference_hand_positions["r"] + self._reference_hand_positions["l"])
            target_centroid = 0.5 * (
                np.asarray(target_poses["r"][0]) + np.asarray(target_poses["l"][0]))
            desired_base_xy = self._reference_base_xy + (target_centroid - reference_centroid)[:2]
            base_position_error = desired_base_xy - current_q[:2]
            target_yaw_deltas = []
            for side in SIDES:
                target_quaternion = np.asarray(target_poses[side][1], dtype=float)
                delta_world = rotations.shortest_orientation_error(
                    target_quaternion, self._reference_hand_quaternions[side])
                target_yaw_deltas.append(delta_world[2])
            desired_base_yaw = self._reference_base_yaw + float(np.mean(target_yaw_deltas))
            base_yaw_error = rotations.wrap_angle(
                desired_base_yaw - current_q[self.index["base_yaw"]])
            yaw_control_error = (
                0.0 if abs(base_yaw_error) <= self.common_base_yaw_deadband else
                math.copysign(
                    abs(base_yaw_error) - self.common_base_yaw_deadband,
                                               base_yaw_error))
            desired_base = np.array([
                self.common_base_position_gain * base_position_error[0],
                self.common_base_position_gain * base_position_error[1],
                np.clip(self.common_base_yaw_gain * yaw_control_error,
                        -self.common_base_yaw_speed_limit,
                        self.common_base_yaw_speed_limit),
            ])
            desired_base = np.clip(
                desired_base, -self.velocity_limits[:3], self.velocity_limits[:3])
            dual_base_request = desired_base.copy()
            base_selector = np.zeros((3, len(self.joint_names)))
            base_selector[:, :3] = np.eye(3)
            common_base_weights = np.sqrt(self.common_base_weights)
            rows.append(common_base_weights[:, None] * base_selector)
            rhs.append(common_base_weights * desired_base)

        # Tikhonov damping으로 특이점 부근에서도 least-squares 조건을 안정화한다.
        rows.append(np.diag(np.sqrt(self.damping_weights)))
        rhs.append(np.zeros(len(self.joint_names)))

        nominal = current_q.copy()
        if lift_nominal is not None:
            nominal[self.index["lift_joint"]] = float(lift_nominal)
        if arm_nominal is not None:
            for side in SIDES:
                if side in arm_nominal:
                    nominal[self.side_indices[side]] = np.asarray(arm_nominal[side], dtype=float)
        posture_velocity = self.posture_gain * (nominal - current_q)
        rows.append(np.diag(np.sqrt(self.posture_weights)))
        rhs.append(np.sqrt(self.posture_weights) * posture_velocity)

        matrix = np.vstack(rows)
        vector = np.concatenate(rhs)
        lower, upper = self._velocity_bounds(current_q, dt)

        # Whole-body OFF는 작은 weight가 아니라 hard participation gate다. body 4개
        # DOF를 고정해 수치 절충이나 posture/collision 항이 잔류 명령을 만들지 못한다.
        if not whole_body_enabled:
            lower[:4] = 0.0
            upper[:4] = 0.0

        # FK mode 팔은 기존 FK controller가 소유하므로 differential velocity를 0으로
        # 고정하고, 반대쪽 팔과 lift/base만 계속 협력하게 한다.
        for side in SIDES:
            if side not in active_sides:
                lower[self.side_indices[side]] = 0.0
                upper[self.side_indices[side]] = 0.0

        qdot = optimization.bounded_least_squares(
            matrix, vector, lower, upper)
        if dual_base_request is not None:
            # hierarchy를 정확히 적용한다. lift/팔은 위 weighted row로 잔차를 풀고,
            # base 3축을 복사해 작은 수치 절충이 큰 swerve heading 변화로 번지지 않게 한다.
            qdot[:3] = dual_base_request
        if whole_body_enabled:
            qdot = self._shape_base_velocity(
                qdot, position_errors, orientation_errors, dt, float(data.time))
        else:
            self._previous_base_velocity_world[:] = 0.0
            self._last_solve_time = None
        collision_constraints = self._collision_constraints(data, dt)
        if collision_constraints:
            barrier_matrix = np.vstack([
                constraint.gradient for constraint, _bound in collision_constraints])
            barrier_lower = np.array([
                bound for _constraint, bound in collision_constraints])
            # Cyclo collision CBF의 quadratic slack을 squared hinge loss로 줄여 작은
            # active set으로 푼다. base shaping 뒤에 적용해야 가속도 제한이 위험한
            # 접근 속도를 다시 만들지 않는다.
            qdot = optimization.bounded_least_squares_with_barriers(
                np.eye(len(qdot)), qdot, lower, upper,
                barrier_matrix, barrier_lower, self.collision_slack_weight)
            # 다음 가속 ramp는 collision safety override까지 반영된 실제 명령에서 시작한다.
            self._previous_base_velocity_world = qdot[:3].copy()
            collision_violation = float(np.max(np.maximum(
                barrier_lower - barrier_matrix @ qdot, 0.0)))
            collision_names = tuple(
                constraint.name for constraint, _bound in collision_constraints)
            minimum_collision_distance = min(
                constraint.distance for constraint, _bound in collision_constraints)
        else:
            collision_violation = 0.0
            collision_names = ()
            minimum_collision_distance = math.inf
        next_q = current_q + qdot * dt
        next_q = self._clip_positions(next_q)

        yaw = float(data.qpos[self.qpos_adrs[self.index["base_yaw"]]])
        vx_world, vy_world = qdot[self.index["base_x"]], qdot[self.index["base_y"]]
        cy, sy = math.cos(yaw), math.sin(yaw)
        base_twist = BodyTwist(
            float(cy * vx_world + sy * vy_world),
            float(-sy * vx_world + cy * vy_world),
            float(qdot[self.index["base_yaw"]]),
        )
        arm_positions = {
            side: next_q[self.side_indices[side]].copy() for side in active_sides
        }
        return WholeBodyCommand(
            base_twist=base_twist,
            arm_positions=arm_positions,
            lift_position=float(next_q[self.index["lift_joint"]]),
            position_errors=position_errors,
            orientation_errors=orientation_errors,
            generalized_velocity=qdot,
            minimum_collision_distance=minimum_collision_distance,
            active_collision_pairs=collision_names,
            collision_constraint_violation=collision_violation,
        )

    def _collision_constraints(self, data, dt):
        """활성화된 ``grad(distance) @ qdot >= lower`` CBF 행을 반환한다."""
        barrier_gain = min(
            self.collision_barrier_gain, 1.0 / max(float(dt), 1e-5))
        constraints = []
        for result in self.collision_distances(data):
            if np.linalg.norm(result.gradient) < 1e-10:
                continue
            lower = -barrier_gain * (result.distance - self.collision_safe_distance)
            constraints.append((result, float(lower)))
        return constraints

    def site_state(self, data, side, current_q=None):
        """custom tree FK/Jacobian으로 한 손의 pose와 Jacobian을 계산한다."""
        if side not in self.kinematics_solvers:
            raise ValueError(f"unknown hand side: {side!r}")
        if current_q is None:
            current_q = np.asarray(data.qpos[self.qpos_adrs], dtype=float)
        return self.kinematics_solvers[side].forward(
            current_q, context_qpos=data.qpos)

    def collision_distances(self, data, max_distance=None):
        """제어 진단과 시각화에 쓸 collision pair 거리를 반환한다.

        read-only query이며 CBF와 같은 geometry/gradient 구현을 사용한다. 따라서
        화면의 최근접점 선과 safety controller의 판단이 달라지지 않는다.
        """
        distance_limit = (self.collision_buffer if max_distance is None
                          else max(float(max_distance), 0.0))
        results = []
        frame_cache = {}
        for pair in self.collision_pairs:
            result = collision.collision_distance_gradient(
                self.model, data, pair, self.kinematic_tree,
                self.joint_ids, distance_limit, frame_cache)
            if result is not None:
                results.append(result)
        return tuple(results)

    def _shape_base_velocity(self, qdot, position_errors, orientation_errors, dt, data_time):
        """differential 해의 물리 base 성분에 fade와 가속도 제한을 적용한다.

        팔은 새 목표를 빠르게 추종하지만 swerve 조향과 무거운 chassis에는 지연이 있다.
        매 frame의 부호 변화를 그대로 보내면 목표 근처에서 방향이 반복 반전된다. 오차가
        클 때는 빠르게 움직이고, 0에 가까워지면 정밀 위치 보정을 lift/팔에 넘긴다.
        짧은 가속 ramp는 한 frame짜리 chassis 반전을 억제한다.
        """
        result = qdot.copy()
        if self._last_solve_time is None or data_time < self._last_solve_time:
            self._previous_base_velocity_world[:] = 0.0
        self._last_solve_time = data_time

        position_ratio = max(position_errors.values(), default=0.0) / self.base_position_fade_distance
        orientation_ratio = (max(orientation_errors.values(), default=0.0)
                             / self.base_orientation_fade_angle)
        task_scale = float(np.clip(max(position_ratio, orientation_ratio), 0.0, 1.0))
        requested = result[:3] * task_scale
        max_delta = np.array([
            self.base_linear_acceleration_limit * dt,
            self.base_linear_acceleration_limit * dt,
            self.base_angular_acceleration_limit * dt,
        ])
        shaped = self._previous_base_velocity_world + np.clip(
            requested - self._previous_base_velocity_world, -max_delta, max_delta)
        shaped = np.clip(shaped, -self.velocity_limits[:3], self.velocity_limits[:3])
        result[:3] = shaped
        self._previous_base_velocity_world = shaped.copy()
        return result

    def _velocity_bounds(self, current_q, dt):
        lower = -self.velocity_limits.copy()
        upper = self.velocity_limits.copy()
        barrier_gain = min(self.joint_limit_gain, 1.0 / max(float(dt), 1e-5))
        for i, limited in enumerate(self.position_limited):
            if not limited:
                continue
            lo, hi = self.position_ranges[i]
            margin = min(self.joint_limit_margin, max(0.0, 0.25 * (hi - lo)))
            safe_lo, safe_hi = lo + margin, hi - margin
            # joint-limit CBF는 한계까지의 거리에 따라 접근 속도를 줄인다. 한 frame
            # hard clamp와 달리 margin 전에 감속하고, 외력으로 벗어나도 부드럽게 복귀한다.
            lower[i] = max(lower[i], -barrier_gain * (current_q[i] - safe_lo))
            upper[i] = min(upper[i], barrier_gain * (safe_hi - current_q[i]))
            if lower[i] > upper[i]:
                # 너무 좁은 range나 범위를 크게 벗어난 상태는 infeasible box를 넘기지 않고
                # 제한된 복귀 방향 하나로 고정한다.
                recovery = (self.velocity_limits[i]
                            if current_q[i] < safe_lo else -self.velocity_limits[i])
                lower[i] = upper[i] = recovery
        return lower, upper

    def _clip_positions(self, q):
        result = q.copy()
        limited = self.position_limited
        result[limited] = np.clip(
            result[limited],
            self.position_ranges[limited, 0],
            self.position_ranges[limited, 1],
        )
        return result
