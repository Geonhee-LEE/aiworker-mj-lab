"""한 Teleop 제어 frame의 수동 주행과 IK 명령 중재 단계.

렌더링과 입력 장치 처리는 다루지 않는다. 전달받은 ``app`` 상태에서 베이스 피드백을
읽고, 수동 명령 우선순위와 Whole-body 결과를 actuator 직전 상태로 정리한다.
"""

import math
from dataclasses import dataclass

import numpy as np

from ..control import base
from . import state, targets


@dataclass(frozen=True)
class BaseFeedback:
    """한 frame에서 읽은 스워브와 차체 피드백."""

    steering_positions: dict
    wheel_velocities: dict
    body_twist: base.BodyTwist
    pose: np.ndarray


@dataclass(frozen=True)
class ManualDriveState:
    """수동 주행 입력을 해석한 결과와 다음 중재에 필요한 상태."""

    feedback: BaseFeedback
    command: base.BodyTwist
    keys_active: bool
    measured_motion_active: bool
    carry_targets: bool


def read_base_feedback(app):
    """모델의 바퀴 상태와 world 속도를 차체 좌표 피드백으로 묶어 반환한다."""
    data = app.data
    bindings = app.bindings
    steering_positions = {
        wheel: float(data.qpos[address.steer_qpos])
        for wheel, address in bindings.wheels.items()
    }
    wheel_velocities = {
        wheel: float(data.qvel[address.drive_dof])
        for wheel, address in bindings.wheels.items()
    }
    base_bindings = bindings.base
    yaw = float(data.qpos[base_bindings.yaw_qpos])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    vx_world = float(data.qvel[base_bindings.x_dof])
    vy_world = float(data.qvel[base_bindings.y_dof])
    body_twist = base.BodyTwist(
        cosine * vx_world + sine * vy_world,
        -sine * vx_world + cosine * vy_world,
        float(data.qvel[base_bindings.yaw_dof]),
    )
    pose = np.array(
        [
            data.qpos[base_bindings.x_qpos],
            data.qpos[base_bindings.y_qpos],
            data.qpos[base_bindings.yaw_qpos],
        ],
        dtype=float,
    )
    return BaseFeedback(
        steering_positions=steering_positions,
        wheel_velocities=wheel_velocities,
        body_twist=body_twist,
        pose=pose,
    )


def update_manual_drive(app, drive_keys, *, stop_linear_speed, stop_angular_speed):
    """키 입력과 실제 차체 속도로 수동 우선권 및 target 운반 상태를 갱신한다."""
    feedback = read_base_feedback(app)
    command = app.base_drive.base.update_body(drive_keys, app.frame_dt)
    keys_active = any(drive_keys.values())
    measured_motion_active = (
        math.hypot(feedback.body_twist.vx, feedback.body_twist.vy) > stop_linear_speed
        or abs(feedback.body_twist.wz) > stop_angular_speed
    )

    if keys_active and not app._manual_override_active:
        # 수동 입력 시작 전 WBIK 이동을 수동 이동으로 계산하지 않도록 시작점을 잡는다.
        app._manual_reference_base_pose = feedback.pose.copy()
    carry_targets = keys_active or app._manual_override_active
    if carry_targets:
        # 키를 놓은 직후 물리 제동으로 움직이는 구간까지 world 목표를 함께 운반한다.
        targets.carry_world_targets_with_base(
            app, app._manual_reference_base_pose, feedback.pose
        )
    app._manual_reference_base_pose = feedback.pose.copy()

    return ManualDriveState(
        feedback=feedback,
        command=command,
        keys_active=keys_active,
        measured_motion_active=measured_motion_active,
        carry_targets=carry_targets,
    )


def apply_whole_body_solution(app, task_command, *, sides, arm_nominal):
    """Whole-body solver를 호출하고 물리 적용용 목표·진단 상태를 갱신한다."""
    active_sides = tuple(side for side in sides if app.arm_mode[side] == "ik")
    command = app.whole_body_solver.solve(
        app.data,
        task_command.hand_poses,
        app.frame_dt,
        active_sides=active_sides,
        arm_nominal=arm_nominal,
        lift_nominal=task_command.lift_position,
        rigid_grasp=(
            app.cyclo_controller == "bimanual_movel" and app.cyclo_grasp_captured
        ),
        whole_body_enabled=app.whole_body_enabled,
    )
    app.whole_body_base_twist = command.base_twist
    app.collision_active_pairs = command.active_collision_pairs
    app.collision_min_distance = command.minimum_collision_distance
    app.collision_constraint_violation = command.collision_constraint_violation
    app.lift_cmd = (
        command.lift_position if app.whole_body_enabled else task_command.lift_position
    )
    for side in sides:
        if side in command.arm_positions:
            app.q_des[side] = command.arm_positions[side]
            app.ik_err_mm[side] = command.position_errors[side] * 1000.0
        else:
            app.q_des[side] = np.radians(app.fk_q_deg[side])
    return command


def select_base_command(app, manual_state):
    """manual > braking stop > WBIK 순으로 차체 명령을 고르고 wheel 명령을 만든다."""
    if manual_state.keys_active:
        app.commanded_base_twist = manual_state.command
    elif app._manual_override_active:
        app.commanded_base_twist = base.BodyTwist()
    elif app.whole_body_enabled:
        app.commanded_base_twist = app.whole_body_base_twist
    else:
        app.commanded_base_twist = base.BodyTwist()

    previous_override = app._manual_override_active
    feedback = manual_state.feedback
    wheel_commands = app.base_drive.update_twist(
        app.commanded_base_twist,
        app.frame_dt,
        feedback.steering_positions,
        feedback.wheel_velocities,
    )
    app._manual_override_active = bool(
        manual_state.keys_active
        or (previous_override and manual_state.measured_motion_active)
    )
    if previous_override and not app._manual_override_active:
        app.base_drive.base.reset_motion()
    return wheel_commands


def build_control_command(app, task_command, wheel_commands):
    """현재 controller 결과를 물리 적용용 불변 명령 스냅샷으로 묶는다."""
    return state.ControlCommand.create(
        arm_positions=app.q_des,
        lift_position=app.lift_cmd,
        base_twist=app.commanded_base_twist,
        wheel_commands=wheel_commands,
        grasp=task_command.grasp,
        thumb=task_command.thumb,
    )


__all__ = [
    "BaseFeedback",
    "ManualDriveState",
    "apply_whole_body_solution",
    "build_control_command",
    "read_base_feedback",
    "select_base_command",
    "update_manual_drive",
]
