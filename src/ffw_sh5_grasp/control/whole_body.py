"""FFW-SH5 이동형 로봇의 MuJoCo/NumPy 기반 differential whole-body IK.

제어하는 generalized velocity 순서는 다음과 같다::

    [base_x, base_y, base_yaw, lift, right_arm(7), left_arm(7)]

이 파일은 target/reference 상태와 한 frame의 solve 순서만 담당한다. soft task 표현은
``kinematics.tasks``, hard bound/CBF는 ``kinematics.constraints``, 실제 수치 해법은
``kinematics.solver``에 분리되어 있다.
"""

from dataclasses import dataclass, field
import math

import mujoco
import numpy as np

from . import bimanual
from .base import BodyTwist
from ..config import SETTINGS
from ..kinematics import collision, constraints, rotations, tasks
from ..kinematics.solver import (
    DifferentialIKSolver,
    IKMethod,
)
from ..kinematics.tree import KinematicTree
# yaml 설정에서 읽는 기본값. 생성자에서 명시적으로 주입하면 덮어쓴다.
BASE_JOINTS = ("base_x", "base_y", "base_yaw")
DEFAULT_BASE_LINEAR_VELOCITY_LIMIT = SETTINGS.number(
    "whole_body_ik.velocity_limits.base_linear", positive=True)
DEFAULT_VELOCITY_LIMITS = {
    "base_x": DEFAULT_BASE_LINEAR_VELOCITY_LIMIT,
    "base_y": DEFAULT_BASE_LINEAR_VELOCITY_LIMIT,
    "base_yaw": SETTINGS.number("whole_body_ik.velocity_limits.base_yaw", positive=True),
    "lift_joint": SETTINGS.number("whole_body_ik.velocity_limits.lift", positive=True),
}
DEFAULT_ARM_VELOCITY_LIMIT = SETTINGS.number(
    "whole_body_ik.velocity_limits.arm", positive=True)
DEFAULT_POSITION_WEIGHT = SETTINGS.number(
    "whole_body_ik.position_weight", positive=True)
DEFAULT_ORIENTATION_WEIGHT = SETTINGS.number(
    "whole_body_ik.orientation_weight", positive=True)
DEFAULT_POSITION_GAIN = SETTINGS.number("whole_body_ik.position_gain", positive=True)
DEFAULT_ORIENTATION_GAIN = SETTINGS.number("whole_body_ik.orientation_gain", positive=True)
# 속도 감쇠는 기본 동작에서 사용하지 않는다. 고급 API 호출에서만 명시적으로 주입한다.
DEFAULT_LINEAR_VELOCITY_DAMPING = 0.0
DEFAULT_ANGULAR_VELOCITY_DAMPING = 0.0
DEFAULT_POSTURE_GAIN = SETTINGS.number("whole_body_ik.posture_gain", minimum=0.0)
DEFAULT_JOINT_LIMIT_MARGIN = SETTINGS.number(
    "whole_body_ik.joint_limit_margin_rad", minimum=0.0)
DEFAULT_JOINT_LIMIT_GAIN = SETTINGS.number("whole_body_ik.joint_limit_gain", positive=True)
DEFAULT_RIGID_GRASP_WEIGHTS = SETTINGS.get("whole_body_ik.rigid_grasp_weights")
DEFAULT_RIGID_GRASP_POSITION_WEIGHT = float(
    DEFAULT_RIGID_GRASP_WEIGHTS["position"])
DEFAULT_RIGID_GRASP_ORIENTATION_WEIGHT = float(
    DEFAULT_RIGID_GRASP_WEIGHTS["orientation"])
DEFAULT_BASE_PARTICIPATION_SCALE = SETTINGS.number(
    "whole_body_ik.base.participation_scale", minimum=0.0)
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
    """한 WBIK 주기에서 actuator 계층으로 전달할 명령과 진단값 묶음.

    ``base_twist``는 차체 좌표 속도, ``arm_positions``는 손별 관절 목표,
    ``lift_position``은 리프트 위치 목표다. 나머지 필드는 손 pose 오차, 전체 일반화
    속도와 충돌 회피 상태를 UI·테스트에서 확인할 수 있도록 보존한다.
    """

    base_twist: BodyTwist = BodyTwist()
    arm_positions: dict = field(default_factory=dict)
    lift_position: float = 0.0
    position_errors: dict = field(default_factory=dict)
    orientation_errors: dict = field(default_factory=dict)
    generalized_velocity: np.ndarray = field(default_factory=lambda: np.zeros(0))
    minimum_collision_distance: float = math.inf
    active_collision_pairs: tuple = ()
    collision_constraint_violation: float = 0.0


@dataclass(frozen=True)
class _CollisionSafetyResult:
    """Collision CBF 투영 뒤의 속도와 진단값."""

    generalized_velocity: np.ndarray
    minimum_distance: float = math.inf
    active_pairs: tuple = ()
    constraint_violation: float = 0.0


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
                 rigid_grasp_position_weight=DEFAULT_RIGID_GRASP_POSITION_WEIGHT,
                 rigid_grasp_orientation_weight=DEFAULT_RIGID_GRASP_ORIENTATION_WEIGHT,
                 base_participation_scale=DEFAULT_BASE_PARTICIPATION_SCALE,
                 collision_avoidance=DEFAULT_COLLISION_AVOIDANCE,
                 collision_pairs=None,
                 collision_buffer=DEFAULT_COLLISION_BUFFER,
                 collision_safe_distance=DEFAULT_COLLISION_SAFE_DISTANCE,
                 collision_barrier_gain=DEFAULT_COLLISION_BARRIER_GAIN,
                 collision_slack_weight=DEFAULT_COLLISION_SLACK_WEIGHT,
                 solver_method=None,
                 pseudoinverse_rcond=None,
                 dls_damping=None):
        """전신 자유도와 손 site를 연결하고 task·제약·충돌 회피 설정을 준비한다.

        ``site_names``와 ``arm_joint_names``는 ``'r'``/``'l'`` 키를 사용한다. 생성자는
        모델 주소, 기구학 트리, 속도·위치 한계와 충돌 쌍을 미리 계산하며 live qpos를
        변경하지 않는다.
        """
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
        # MJCF topology는 한 번만 읽고 양쪽 end-effector FK가 공유한다.
        self.kinematic_tree = KinematicTree(model)
        solver_settings = SETTINGS.get("whole_body_ik.solver")
        self.differential_solver = DifferentialIKSolver(
            method=(solver_settings["method"] if solver_method is None
                    else solver_method),
            pseudoinverse_rcond=(
                solver_settings["pseudoinverse_rcond"]
                if pseudoinverse_rcond is None else pseudoinverse_rcond),
            dls_damping=(solver_settings["dls_damping"]
                         if dls_damping is None else dls_damping),
        )

        self.position_weight = float(position_weight)
        self.orientation_weight = float(orientation_weight)
        self.position_gain = float(position_gain)
        self.orientation_gain = float(orientation_gain)
        self.linear_velocity_damping = float(linear_velocity_damping)
        self.angular_velocity_damping = float(angular_velocity_damping)
        self.posture_gain = float(posture_gain)
        self.joint_limit_margin = float(joint_limit_margin)
        self.joint_limit_gain = float(joint_limit_gain)
        self.rigid_grasp_position_weight = float(rigid_grasp_position_weight)
        self.rigid_grasp_orientation_weight = float(rigid_grasp_orientation_weight)
        self.base_participation_scale = float(base_participation_scale)
        if not 0.0 <= self.base_participation_scale <= 1.0:
            raise ValueError("base_participation_scale must be between 0 and 1")
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
        common_base_weights = SETTINGS.get("whole_body_ik.common_base.task_weights")
        self.common_base_weights = np.array([
            common_base_weights["translation"],
            common_base_weights["translation"],
            common_base_weights["yaw"],
        ], dtype=float)
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
        base_linear_damping = SETTINGS.number(
            "whole_body_ik.damping_weights.base_linear", minimum=0.0)
        self.damping_weights = np.array(
            [base_linear_damping, base_linear_damping,
             SETTINGS.number("whole_body_ik.damping_weights.base_yaw", minimum=0.0),
             SETTINGS.number("whole_body_ik.damping_weights.lift", minimum=0.0)]
            + [SETTINGS.number("whole_body_ik.damping_weights.arm", minimum=0.0)] * 14,
            dtype=float)
        self.posture_weights = np.array(
            [0.0, 0.0, 0.0,
             SETTINGS.number("whole_body_ik.posture_weights.lift", minimum=0.0)]
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
        positive_task_weights = (
            self.position_weight,
            self.orientation_weight,
            self.rigid_grasp_position_weight,
            self.rigid_grasp_orientation_weight,
        )
        if any(weight <= 0.0 for weight in positive_task_weights):
            raise ValueError("전신 IK의 task strength는 모두 양수여야 합니다.")

    @property
    def solver_method(self):
        """현재 differential IK 해법 이름을 반환한다."""
        return self.differential_solver.method.value

    def set_solver_method(self, method):
        """pseudoinverse, DLS, QP 중 다음 frame에 사용할 해법을 선택한다."""
        self.differential_solver.set_method(method)

    def set_dls_damping(self, value):
        """실행 중 DLS 감쇠 계수를 안전하게 갱신한다."""
        value = float(value)
        if value <= 0.0:
            raise ValueError("DLS damping must be positive")
        self.differential_solver.dls_damping = value

    def qp_weights(self):
        """UI와 외부 호출자가 수정할 수 있는 무차원 QP strength를 반환한다."""
        return {
            "position": self.position_weight,
            "orientation": self.orientation_weight,
            "rigid_grasp_position": self.rigid_grasp_position_weight,
            "rigid_grasp_orientation": self.rigid_grasp_orientation_weight,
            "damping_base_linear": float(self.damping_weights[0]),
            "damping_base_yaw": float(self.damping_weights[2]),
            "damping_lift": float(self.damping_weights[3]),
            "damping_arm": float(self.damping_weights[4]),
            "posture_lift": float(self.posture_weights[3]),
            "posture_arm": float(self.posture_weights[4]),
            "collision_slack": self.collision_slack_weight,
        }

    def set_qp_weight(self, name, value):
        """이름 기반으로 QP task·정규화·slack 가중치 하나를 갱신한다."""
        value = float(value)
        positive = {
            "position", "orientation", "rigid_grasp_position",
            "rigid_grasp_orientation", "collision_slack",
        }
        non_negative = {
            "damping_base_linear", "damping_base_yaw", "damping_lift",
            "damping_arm", "posture_lift", "posture_arm",
        }
        if name not in positive | non_negative:
            raise ValueError(f"unknown QP weight: {name!r}")
        if (name in positive and value <= 0.0) or (name in non_negative and value < 0.0):
            raise ValueError(f"invalid QP weight {name!r}: {value}")
        if name == "position":
            self.position_weight = value
        elif name == "orientation":
            self.orientation_weight = value
        elif name == "rigid_grasp_position":
            self.rigid_grasp_position_weight = value
        elif name == "rigid_grasp_orientation":
            self.rigid_grasp_orientation_weight = value
        elif name == "damping_base_linear":
            self.damping_weights[:2] = value
        elif name == "damping_base_yaw":
            self.damping_weights[2] = value
        elif name == "damping_lift":
            self.damping_weights[3] = value
        elif name == "damping_arm":
            self.damping_weights[4:] = value
        elif name == "posture_lift":
            self.posture_weights[3] = value
        elif name == "posture_arm":
            self.posture_weights[4:] = value
        elif name == "collision_slack":
            self.collision_slack_weight = value

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

    def _append_hand_tasks(self, data, target_poses, current_q, active_sides,
                           task_list):
        """활성 손의 pose 추종 task를 weighted least-squares 행에 추가한다."""
        position_errors, orientation_errors, site_states = {}, {}, {}
        for side in active_sides:
            target_pos, target_quat = target_poses[side]
            state = self.site_state(data, side, current_q)
            site_states[side] = state
            jac = state.jacobian
            site_velocity = jac @ data.qvel[self.dof_ids]

            error = tasks.pose_error(
                state.position, state.quaternion, target_pos, target_quat)
            desired = tasks.pose_velocity_command(
                error,
                position_gain=self.position_gain,
                orientation_gain=self.orientation_gain,
                current_twist=site_velocity,
                linear_velocity_damping=self.linear_velocity_damping,
                angular_velocity_damping=self.angular_velocity_damping,
                max_linear_speed=self.max_task_linear_speed,
                max_angular_speed=self.max_task_angular_speed,
            )
            strengths = np.array(
                [self.position_weight] * 3 + [self.orientation_weight] * 3)
            speed_scales = np.array(
                [self.max_task_linear_speed] * 3
                + [self.max_task_angular_speed] * 3)
            task_list.append(tasks.velocity_task(
                f"{side}_hand_pose", jac, desired, strengths, speed_scales))
            position_errors[side] = error.position_norm
            orientation_errors[side] = error.orientation_norm
        return position_errors, orientation_errors, site_states

    def _append_rigid_grasp_task(self, data, site_states, dt, task_list):
        """캡처한 양손 상대 pose를 보존하는 task를 추가한다."""
        if not all(side in site_states for side in SIDES):
            return
        if self._rigid_grasp_reference is None:
            self.set_rigid_grasp(data, True)
        grasp_jacobian, grasp_velocity = bimanual.rigid_grasp_task(
            self._rigid_grasp_reference,
            site_states,
            dt,
            self.max_task_linear_speed,
            self.max_task_angular_speed,
        )
        strengths = np.array(
            [self.rigid_grasp_position_weight] * 3
            + [self.rigid_grasp_orientation_weight] * 3)
        speed_scales = np.array(
            [self.max_task_linear_speed] * 3
            + [self.max_task_angular_speed] * 3)
        task_list.append(tasks.velocity_task(
            "rigid_grasp", grasp_jacobian, grasp_velocity,
            strengths, speed_scales))

    def _append_common_base_task(self, target_poses, current_q,
                                 position_errors, task_list):
        """양손의 공통 목표 이동을 base velocity task로 추가한다."""
        if not all(side in position_errors for side in SIDES):
            return
        # 양손 평균 오차로 base를 먼저 servo하고 나머지 개별 오차는 lift/팔이 푼다.
        # minimum norm에만 맡기면 swerve가 조향 중일 때 14개 팔 열이 공통 오차의
        # 부호를 바꾸어 chassis 방향이 반복 반전될 수 있다.
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
            self.common_base_yaw_gain * yaw_control_error,
        ])
        # 참여율은 명시적 base task와 물리 속도 상한 양쪽에 적용한다. 목표만
        # 줄이면 다른 hand task가 base 열을 다시 크게 사용할 수 있고, bound만
        # 줄이면 작은 참여율에서도 계속 포화되므로 두 곳을 같은 비율로 낮춘다.
        desired_base *= self.base_participation_scale
        base_velocity_limits = self._base_velocity_limits()
        desired_base = np.clip(
            desired_base, -base_velocity_limits, base_velocity_limits)
        base_selector = np.zeros((3, len(self.joint_names)))
        base_selector[:, :3] = np.eye(3)
        task_list.append(tasks.velocity_task(
            "common_base", base_selector, desired_base,
            self.common_base_weights, self.velocity_limits[:3]))

    def _apply_mode_velocity_bounds(self, lower, upper, active_sides,
                                    whole_body_enabled):
        """선택한 IK 모드에 맞게 base, lift, 비활성 팔의 속도 bounds를 고정한다."""
        # Whole-body OFF는 작은 weight가 아니라 hard participation gate다. body 4개
        # DOF를 고정해 수치 절충이나 posture/collision 항이 잔류 명령을 만들지 못한다.
        if not whole_body_enabled:
            lower[:4] = 0.0
            upper[:4] = 0.0
        else:
            # 베이스만 끄거나 참여율을 낮춰도 lift와 양팔 bound는 그대로 유지한다.
            base_limits = self._base_velocity_limits()
            lower[:3] = np.maximum(lower[:3], -base_limits)
            upper[:3] = np.minimum(upper[:3], base_limits)

        # FK mode 팔은 기존 FK controller가 소유하므로 differential velocity를 0으로
        # 고정하고, 반대쪽 팔과 lift/base만 계속 협력하게 한다.
        for side in SIDES:
            if side not in active_sides:
                lower[self.side_indices[side]] = 0.0
                upper[self.side_indices[side]] = 0.0

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
        task_list = []
        position_errors, orientation_errors, site_states = self._append_hand_tasks(
            data, target_poses, current_q, active_sides, task_list)

        if rigid_grasp:
            self._append_rigid_grasp_task(data, site_states, dt, task_list)

        if whole_body_enabled:
            self._append_common_base_task(
                target_poses, current_q, position_errors, task_list)

        # 자유도별 정규화는 QP 정책이다. DLS는 solver의 단일 damping을 사용하고,
        # pseudoinverse는 task 행의 Moore-Penrose 최소노름 해를 그대로 계산한다.
        if self.differential_solver.method is IKMethod.QP:
            task_list.append(tasks.regularization_task(
                "damping", np.zeros(len(self.joint_names)),
                self.damping_weights, self.velocity_limits))

        nominal = current_q.copy()
        if lift_nominal is not None:
            nominal[self.index["lift_joint"]] = float(lift_nominal)
        if arm_nominal is not None:
            for side in SIDES:
                if side in arm_nominal:
                    nominal[self.side_indices[side]] = np.asarray(
                        arm_nominal[side], dtype=float)
        posture_velocity = self.posture_gain * (nominal - current_q)
        task_list.append(tasks.regularization_task(
            "posture", posture_velocity,
            self.posture_weights, self.velocity_limits))

        matrix, vector = tasks.stack_velocity_tasks(
            task_list, len(self.joint_names))
        lower, upper = self._velocity_bounds(current_q, dt)
        self._apply_mode_velocity_bounds(
            lower, upper, active_sides, whole_body_enabled)

        qdot = self.differential_solver.solve(matrix, vector, lower, upper)
        if whole_body_enabled:
            qdot = self._shape_base_velocity(
                qdot, position_errors, orientation_errors, dt, float(data.time))
        else:
            self._previous_base_velocity_world[:] = 0.0
            self._last_solve_time = None
        safety = self._project_collision_safety(
            data, dt, qdot, lower, upper)
        return self._command_from_velocity(
            data, current_q, dt, active_sides,
            position_errors, orientation_errors, safety)

    def _project_collision_safety(self, data, dt, qdot, lower, upper):
        """Base shaping 이후 collision CBF를 투영하고 진단값을 함께 반환한다."""
        collision_barriers = self._collision_constraints(data, dt)
        if not collision_barriers:
            return _CollisionSafetyResult(qdot)

        barrier_matrix = np.vstack([
            barrier.gradient for barrier in collision_barriers])
        barrier_lower = np.array([
            barrier.lower for barrier in collision_barriers])
        # 가속도 제한 뒤에 투영해 shaping이 위험한 접근 속도를 다시 만들지 않게 한다.
        safe_qdot = self.differential_solver.enforce_constraints(
            qdot, lower, upper,
            barrier_matrix, barrier_lower, self.collision_slack_weight,
            variable_scale=self.velocity_limits,
            barrier_scale=self.max_task_linear_speed)
        # 다음 base 가속 ramp는 safety override가 반영된 실제 명령에서 시작한다.
        self._previous_base_velocity_world = safe_qdot[:3].copy()
        violation = float(np.max(np.maximum(
            barrier_lower - barrier_matrix @ safe_qdot, 0.0)))
        return _CollisionSafetyResult(
            generalized_velocity=safe_qdot,
            minimum_distance=min(
                barrier.distance for barrier in collision_barriers),
            active_pairs=tuple(
                barrier.name for barrier in collision_barriers),
            constraint_violation=violation,
        )

    def _command_from_velocity(self, data, current_q, dt, active_sides,
                               position_errors, orientation_errors, safety):
        """안전 투영된 generalized velocity를 actuator 계층용 명령으로 변환한다."""
        qdot = safety.generalized_velocity
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
            minimum_collision_distance=safety.minimum_distance,
            active_collision_pairs=safety.active_pairs,
            collision_constraint_violation=safety.constraint_violation,
        )

    def _collision_constraints(self, data, dt):
        """활성화된 ``grad(distance) @ qdot >= lower`` CBF 행을 반환한다."""
        return constraints.collision_velocity_barriers(
            self.collision_distances(data), dt,
            safe_distance=self.collision_safe_distance,
            gain=self.collision_barrier_gain)

    def site_state(self, data, side, current_q=None):
        """custom tree FK/Jacobian으로 한 손의 pose와 Jacobian을 계산한다."""
        if side not in self.site_ids:
            raise ValueError(f"unknown hand side: {side!r}")
        if current_q is None:
            current_q = np.asarray(data.qpos[self.qpos_adrs], dtype=float)
        current_q = np.asarray(current_q, dtype=float)
        if current_q.shape != (len(self.joint_names),):
            raise ValueError("whole-body joint position vector has an invalid shape")
        qpos = np.asarray(data.qpos, dtype=float).copy()
        qpos[self.qpos_adrs] = current_q
        return self.kinematic_tree.forward_site(
            qpos, self.site_ids[side], self.joint_ids)

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
        base_limits = self._base_velocity_limits()
        if not np.any(base_limits):
            result[:3] = 0.0
            self._previous_base_velocity_world[:] = 0.0
            self._last_solve_time = data_time
            return result
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
        shaped = np.clip(shaped, -base_limits, base_limits)
        result[:3] = shaped
        self._previous_base_velocity_world = shaped.copy()
        return result

    def _base_velocity_limits(self):
        """현재 참여율이 반영된 base 3축 속도 상한을 반환한다."""
        return self.base_participation_scale * self.velocity_limits[:3]

    def _velocity_bounds(self, current_q, dt):
        """속도 한계와 joint-limit barrier를 결합한 관절별 하한·상한을 반환한다."""
        return constraints.joint_velocity_bounds(
            current_q, self.velocity_limits,
            self.position_limited, self.position_ranges, dt,
            margin=self.joint_limit_margin, gain=self.joint_limit_gain)

    def _clip_positions(self, q):
        """위치 제한이 있는 자유도만 안전 범위로 자른 관절 벡터를 반환한다."""
        return constraints.clip_joint_positions(
            q, self.position_limited, self.position_ranges)
