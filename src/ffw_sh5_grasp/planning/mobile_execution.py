"""P7.1: 목표 베이스 자세로 실제 주행하는 얇은 실행 헬퍼.

``control.whole_body.WholeBodyIK``는 손 목표를 줄이는 반응형 솔버라 베이스
이동은 그 부산물일 뿐이다 — "베이스를 (x, y, yaw)로 보내라"는 명시적
지점-대-지점 주행에는 맞지 않는다. 그래서 이 모듈은 새 저수준 제어를
만들지 않고, 매 스텝 현재 오차로 간단한 비례 제어 차체 twist를 계산해
``control.base.SwerveDrive.update_twist()``에 그대로 넘긴다 — 반환된 바퀴
조향·구동 명령을 ``data.ctrl``에 기록하고 ``mujoco.mj_step``으로 물리를
진행할 뿐이다(``base_x``/``base_y``/``base_yaw``는 수동 관절이라 바퀴-지면
마찰로만 실제로 움직인다 — qpos에 직접 쓰지 않는다).
"""

import math
from dataclasses import dataclass

import mujoco
import numpy as np

from ..control.base import BodyTwist

_WHEEL_NAMES = ("left_wheel", "right_wheel", "rear_wheel")
_BASE_JOINT_NAMES = ("base_x", "base_y", "base_yaw")


@dataclass(frozen=True)
class BaseTransitReport:
    """``drive_base_to_pose`` 한 번의 결과."""

    success: bool
    final_base_pose: object  # (x, y, yaw) np.ndarray
    steps: int
    final_position_error_m: float
    final_yaw_error_rad: float


def _wrap_angle(angle):
    """각도를 (-pi, pi]로 감는다 — base_yaw 관절은 범위 제한이 없다."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def drive_base_to_pose(
    model,
    data,
    target_pose,
    swerve_drive,
    *,
    tolerance_m=0.03,
    tolerance_rad=0.02,
    kp_linear=1.5,
    kp_angular=1.5,
    max_speed=0.6,
    max_steps=20000,
):
    """``swerve_drive``로 베이스를 ``target_pose``(x, y, yaw)까지 주행시킨다.

    매 스텝: 월드 프레임 위치 오차에 비례 게인을 곱해 목표 차체 속도를
    구하고, 현재 yaw로 회전시켜 차체(body) 프레임 twist로 바꾼 뒤
    ``swerve_drive.update_twist``에 넘긴다. 반환된 바퀴별 (조향각, 구동
    속도)를 대응하는 액추에이터에 그대로 쓴다 — 새 컨트롤러가 아니라
    기존 ``SwerveDrive``를 목표-오차 루프로 감싼 것뿐이다.
    """
    dt = float(model.opt.timestep)
    target_pose = np.asarray(target_pose, dtype=float)

    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in _BASE_JOINT_NAMES
    ]
    if any(joint_id < 0 for joint_id in joint_ids):
        raise ValueError("모델에 base_x/base_y/base_yaw 관절이 없습니다")
    qpos_adrs = np.array([model.jnt_qposadr[joint_id] for joint_id in joint_ids])

    wheel_actuators = {}
    for name in _WHEEL_NAMES:
        steer_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_steer")
        drive_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_drive")
        if steer_id < 0 or drive_id < 0:
            raise ValueError(f"모델에 {name}_steer/{name}_drive 액추에이터가 없습니다")
        wheel_actuators[name] = (steer_id, drive_id)

    steps = 0
    position_error_m = float("inf")
    yaw_error_rad = float("inf")
    for steps in range(1, max_steps + 1):
        current = data.qpos[qpos_adrs]
        dx = target_pose[0] - current[0]
        dy = target_pose[1] - current[1]
        position_error_m = float(math.hypot(dx, dy))
        yaw_error_rad = _wrap_angle(target_pose[2] - current[2])
        if position_error_m < tolerance_m and abs(yaw_error_rad) < tolerance_rad:
            break

        cos_t, sin_t = math.cos(-current[2]), math.sin(-current[2])
        vx_world = kp_linear * dx
        vy_world = kp_linear * dy
        vx = cos_t * vx_world - sin_t * vy_world
        vy = sin_t * vx_world + cos_t * vy_world
        speed = math.hypot(vx, vy)
        if speed > max_speed:
            scale = max_speed / speed
            vx *= scale
            vy *= scale
        wz = float(np.clip(kp_angular * yaw_error_rad, -max_speed, max_speed))

        module_results = swerve_drive.update_twist(BodyTwist(vx, vy, wz), dt)
        for name, (steer_actuator, drive_actuator) in wheel_actuators.items():
            steer_angle, wheel_speed = module_results[name]
            data.ctrl[steer_actuator] = steer_angle
            data.ctrl[drive_actuator] = wheel_speed
        mujoco.mj_step(model, data)

    final = data.qpos[qpos_adrs].copy()
    success = position_error_m < tolerance_m and abs(yaw_error_rad) < tolerance_rad
    return BaseTransitReport(
        success=success,
        final_base_pose=final,
        steps=steps,
        final_position_error_m=position_error_m,
        final_yaw_error_rad=abs(yaw_error_rad),
    )


__all__ = ["BaseTransitReport", "drive_base_to_pose"]
