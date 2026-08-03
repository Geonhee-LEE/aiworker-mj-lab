"""ROS 없이 동작하는 AI Worker 방식 모바일 베이스 제어.

이 모듈은 의도적으로 다음 세 계층을 분리한다.

``BaseTeleop``
    키보드 입력을 부드러운 차체 좌표계 속도 명령으로 바꾼다.
``SwerveKinematics``
    순수 기하만으로 차체 twist와 실행 가능한 조향·구동 상태를 양방향 변환한다.
    키, MuJoCo, ROS 또는 액추에이터는 알지 못한다.
``SwerveDrive``
    피드백 기반 조향 변화율 제한, 정렬 게이트와 ROBOTIS의
    ``감속 -> 조향 -> 가속`` 구동 방향 반전 상태 머신을 적용한다.

기하와 제어 흐름은 ROBOTIS AI Worker의 공식 ``ffw_swerve_drive_controller``를
따른다. 알고리즘만 이식했으므로 공개 입출력은 일반 Python 수와 사전이며, ROS
의존성이나 qpos 직접 쓰기가 없다.
"""

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np

from ..config import SETTINGS


# 키보드 명령 평활화 값이다. 하드웨어 상수가 아니라 시뮬레이션과 조작감 기준이다.
K_SPEED = SETTINGS.number("base.teleop.cruise_speed_m_s", minimum=0.0)
K_MAX = SETTINGS.number("base.teleop.max_speed_m_s", positive=True)
K_ACCEL = SETTINGS.number("base.teleop.acceleration_gain", positive=True)
K_BRAKE = SETTINGS.number("base.teleop.brake_gain", positive=True)
K_YAW = SETTINGS.number("base.teleop.yaw_speed_rad_s", minimum=0.0)
YAW_FOLLOW = SETTINGS.number("base.teleop.yaw_follow_gain", positive=True)
YAW_DECAY = SETTINGS.number("base.teleop.yaw_decay_gain", positive=True)
# 수치적인 0 판정값은 사용자 조절 대상이 아닌 구현 상수다.
VEL_ZERO_EPS = 1e-3
if K_SPEED > K_MAX:
    raise ValueError("base.teleop.cruise_speed_m_s는 max_speed_m_s 이하여야 합니다.")

# 공식 AI Worker 설정에서 가져온 FFW-SH5 3모듈 기하 정보다.
WHEEL_POS = {
    name: tuple(float(value) for value in position)
    for name, position in SETTINGS.get("base.geometry.wheel_positions_m").items()
}
WHEELS = tuple(WHEEL_POS)

# 실제 공식 바퀴 반지름은 0.0865 m지만 이 MuJoCo 장면은 의도적으로 0.09 m 충돌
# 원통을 사용하므로 기구학 반지름도 접촉 형상과 일치시킨다.
WHEEL_RADIUS = SETTINGS.number("base.geometry.wheel_radius_m", positive=True)
# 공식 AI Worker 런타임 설정은 약 ±2π 조향을 허용한다. 이 범위를 유지하면 ±90도
# 부근의 작은 차체 twist 변화가 거의 180도에 이르는 모듈 반전을 요구하는 현상도
# 막을 수 있다. 좁은 범위 동작은 주입 가능한 범위 인자로 계속 검증한다.
STEER_RANGE = tuple(float(value) for value in SETTINGS.get(
    "base.geometry.steering_range_rad"))
MODULE_ANGLE_OFFSETS = {
    name: float(value) for name, value in SETTINGS.get(
        "base.geometry.module_angle_offsets_rad").items()
}
WHEEL_SPEED_LIMIT = tuple(float(value) for value in SETTINGS.get(
    "base.geometry.wheel_speed_limit_rad_s"))
if set(MODULE_ANGLE_OFFSETS) != set(WHEEL_POS):
    raise ValueError("바퀴 위치와 module_angle_offsets_rad의 모듈 이름이 같아야 합니다.")
if STEER_RANGE[0] >= STEER_RANGE[1] or WHEEL_SPEED_LIMIT[0] >= WHEEL_SPEED_LIMIT[1]:
    raise ValueError("조향 및 바퀴 속도 범위는 [최솟값, 최댓값] 순서여야 합니다.")

LINEAR_VEL_DEADBAND = SETTINGS.number(
    "base.control.linear_velocity_deadband_m_s", minimum=0.0)
ANGULAR_VEL_DEADBAND = SETTINGS.number(
    "base.control.angular_velocity_deadband_rad_s", minimum=0.0)
MODULE_SPEED_EPS = 1e-5
STEERING_ANGULAR_VELOCITY_LIMIT = SETTINGS.number(
    "base.control.steering_velocity_limit_rad_s", positive=True)
STEERING_ALIGNMENT_ANGLE_ERROR_THRESHOLD = SETTINGS.number(
    "base.control.alignment_angle_threshold_rad", minimum=0.0)
STEERING_ALIGNMENT_START_SPEED_ERROR_THRESHOLD = SETTINGS.number(
    "base.control.alignment_start_speed_threshold_rad_s", minimum=0.0)
STEERING_TOLERANCE = SETTINGS.number(
    "base.control.steering_tolerance_rad", minimum=0.0)
DIRECTION_SWITCH_STEERING_HYSTERESIS = SETTINGS.number(
    "base.control.direction_switch_hysteresis_rad", minimum=0.0)
REVERSAL_DECEL_RATE = SETTINGS.number(
    "base.control.reversal_deceleration_rate", positive=True)
REVERSAL_ACCEL_RATE = SETTINGS.number(
    "base.control.reversal_acceleration_rate", positive=True)
REVERSAL_THRESHOLD = SETTINGS.number(
    "base.control.reversal_threshold", minimum=0.0)
DRIVE_COMMAND_ACCEL_LIMIT = SETTINGS.number(
    "base.control.drive_acceleration_limit_rad_s2", positive=True)
DRIVE_COMMAND_BRAKE_LIMIT = SETTINGS.number(
    "base.control.drive_brake_limit_rad_s2", positive=True)
DRIVE_COMMAND_CREEP_THRESHOLD = SETTINGS.number(
    "base.control.drive_creep_threshold_rad_s", minimum=0.0)
DRIVE_COMMAND_CREEP_BRAKE_LIMIT = SETTINGS.number(
    "base.control.drive_creep_brake_limit_rad_s2", positive=True)


@dataclass(frozen=True)
class BodyTwist:
    """로봇 차체 좌표계의 평면 속도."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def is_zero(self):
        """세 속도 성분이 YAML deadband보다 작으면 정지 명령으로 판정한다."""
        return (
            abs(self.vx) < LINEAR_VEL_DEADBAND
            and abs(self.vy) < LINEAR_VEL_DEADBAND
            and abs(self.wz) < ANGULAR_VEL_DEADBAND
        )


class BaseTeleop:
    """키보드 입력 의도를 평활화된 차체 좌표계 속도 명령으로 변환한다."""

    def __init__(self):
        """병진과 회전 명령의 내부 평활화 상태를 정지값으로 초기화한다."""
        self.v_local = np.zeros(2)
        self.w = 0.0

    def update_body(self, keys, dt, measured_twist=None):
        """월드 좌표계나 바퀴 정보 없이 차체 좌표계 명령을 반환한다."""
        fwd = float(bool(keys.get("w"))) - float(bool(keys.get("s")))
        left = float(bool(keys.get("a"))) - float(bool(keys.get("d")))
        turn = float(bool(keys.get("left"))) - float(bool(keys.get("right")))

        target_local = np.array([fwd, left], dtype=float)
        norm = float(np.linalg.norm(target_local))
        if norm > 1e-9:
            target_local *= K_SPEED / norm
        target_w = turn * K_YAW

        # 홀로노믹 차체 twist에서 병진과 yaw는 독립 성분이다. yaw 입력 중 병진을
        # 억제하면 곡선 주행을 할 수 없으므로 두 성분을 함께 허용한다.
        if fwd != 0.0 or left != 0.0:
            self.v_local += (target_local - self.v_local) * (1.0 - math.exp(-K_ACCEL * dt))
        else:
            self.v_local *= math.exp(-K_BRAKE * dt)

        speed = float(np.linalg.norm(self.v_local))
        if speed > K_MAX:
            self.v_local *= K_MAX / speed
        if float(np.linalg.norm(self.v_local)) < VEL_ZERO_EPS:
            self.v_local[:] = 0.0

        if target_w != 0.0:
            self.w += (target_w - self.w) * (1.0 - math.exp(-YAW_FOLLOW * dt))
        else:
            self.w *= math.exp(-YAW_DECAY * dt)
        if abs(self.w) < VEL_ZERO_EPS:
            self.w = 0.0

        self.v_local[np.abs(self.v_local) < LINEAR_VEL_DEADBAND] = 0.0
        if abs(self.w) < ANGULAR_VEL_DEADBAND:
            self.w = 0.0
        del measured_twist  # 오도메트리를 전달하는 기존 호출부와의 호환 인자다.
        return BodyTwist(float(self.v_local[0]), float(self.v_local[1]), float(self.w))

    def update(self, keys, dt, yaw=0.0):
        """기존 월드 좌표계 ``vx, vy, wz`` 튜플을 반환하는 호환 함수."""
        cmd = self.update_body(keys, dt)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return cy * cmd.vx - sy * cmd.vy, sy * cmd.vx + cy * cmd.vy, cmd.wz

    def reset_motion(self):
        """피드백으로 물리 정지를 확인한 뒤 남은 입력 평활화 상태를 지운다."""
        self.v_local[:] = 0.0
        self.w = 0.0


class SwerveKinematics:
    """독립 조향 바퀴 모듈의 순수 역기구학과 정기구학."""

    def __init__(self, wheel_positions=WHEEL_POS, wheel_radius=WHEEL_RADIUS,
                 steer_range=STEER_RANGE, angle_offsets=MODULE_ANGLE_OFFSETS,
                 wheel_speed_limit=WHEEL_SPEED_LIMIT):
        """모듈 위치·반지름·조향 보정값과 actuator 한계를 복사해 보관한다.

        기본값은 FFW-SH5 YAML 설정이며, 테스트나 다른 플랫폼은 같은 형식의 기하를
        주입해 역기구학과 정기구학을 재사용할 수 있다.
        """
        self.wheel_positions = dict(wheel_positions)
        self.wheel_radius = float(wheel_radius)
        self.steer_range = tuple(float(v) for v in steer_range)
        self.angle_offsets = dict(angle_offsets)
        self.wheel_speed_limit = tuple(float(v) for v in wheel_speed_limit)

    def inverse(self, twist, steering_positions=None, preferred_directions=None):
        """차체 twist를 실행 가능한 ``(steer_angle, wheel_rad_s)`` 상태로 변환한다.

        단순히 ``atan2`` 결과를 잘라내지 않고, 같은 구름 방향을 만드는 모든 각도
        상태를 탐색한다. 이 상태는 ``angle + k*pi``와 교대하는 구동 부호로 표현된다.
        런타임의 약 ±2π 조향 범위와 주입된 좁은 범위 모델에 모두 적용할 수 있다.
        이미 고른 각도를 잘라내지 않고 실행 가능성을 먼저 판정한 뒤 최단 명령을 고른다.
        """
        steering_positions = steering_positions or {}
        preferred_directions = preferred_directions or {}
        states = {}
        for name, (module_x, module_y) in self.wheel_positions.items():
            wheel_vx = twist.vx - twist.wz * module_y
            wheel_vy = twist.vy + twist.wz * module_x
            linear_speed = math.hypot(wheel_vx, wheel_vy)
            current = float(steering_positions.get(name, 0.0))
            if linear_speed < MODULE_SPEED_EPS:
                states[name] = (current, 0.0)
                continue

            robot_angle = math.atan2(wheel_vy, wheel_vx)
            joint_angle = _normalize_angle(robot_angle - self.angle_offsets[name])
            steer, direction = self._nearest_feasible_state(
                current, joint_angle, preferred_directions.get(name, 1.0))
            states[name] = (steer, direction * linear_speed / self.wheel_radius)

        # 포화 중에도 요청한 차체 twist 방향을 보존한다. 공통 배율은 모든 모듈의 속도
        # 비율을 유지하지만 모듈별 제한은 병진과 회전의 비율을 왜곡한다.
        max_requested = max((abs(speed) for _angle, speed in states.values()), default=0.0)
        max_allowed = max(abs(self.wheel_speed_limit[0]), abs(self.wheel_speed_limit[1]))
        scale = 1.0 if max_requested <= max_allowed else max_allowed / max_requested
        if scale < 1.0:
            states = {name: (angle, speed * scale) for name, (angle, speed) in states.items()}
        return states, scale

    def forward(self, steering_positions, wheel_velocities):
        """바퀴 피드백으로 차체 ``BodyTwist``를 최소제곱 추정한다."""
        rows, rhs = [], []
        for name, (module_x, module_y) in self.wheel_positions.items():
            joint_angle = float(steering_positions[name])
            robot_angle = joint_angle + self.angle_offsets[name]
            c, s = math.cos(robot_angle), math.sin(robot_angle)
            rows.append([c, s, -c * module_y + s * module_x])
            rhs.append(float(wheel_velocities[name]) * self.wheel_radius)
        # 평행한 모듈은 구름 축에 수직인 속도를 관측할 수 없다. 작은 서보 각도 잡음은
        # 원래 계수 부족인 행렬을 가역처럼 보이게 하고 약 1e-7 rad 잡음을 수 m/s의
        # 가짜 횡속도로 증폭한다. 절단 SVD로 물리적 의미가 있는 최소 노름 관측 twist를
        # 반환한다.
        solution, *_ = np.linalg.lstsq(
            np.asarray(rows), np.asarray(rhs), rcond=1e-6)
        return BodyTwist(*(float(v) for v in solution))

    def _nearest_feasible_state(self, current, target_angle, preferred_direction=1.0):
        """동일한 구름 방향 후보 중 조향 범위 안에서 이동 비용이 가장 작은 상태를 고른다.

        반환값은 ``(joint_angle, drive_direction)``이다. 목표에 π를 더할 때마다 같은
        바퀴 속도 벡터를 만들도록 구동 부호를 번갈아 반전한다.
        """
        lo, hi = self.steer_range
        candidates = []
        for k in range(-3, 4):
            angle = target_angle + k * math.pi
            if lo - 1e-12 <= angle <= hi + 1e-12:
                direction = 1.0 if k % 2 == 0 else -1.0
                travel = abs(angle - current)
                switch_cost = (DIRECTION_SWITCH_STEERING_HYSTERESIS
                               if direction != preferred_direction else 0.0)
                candidates.append((travel + switch_cost, travel, angle, direction))
        if not candidates:
            # π보다 좁은 조향 구간은 모든 방향을 표현할 수 없다. 이런 모델에서도
            # 결과가 안전하고 결정적이도록 목표 각도를 범위 안으로 제한한다.
            clipped = _clamp(target_angle, lo, hi)
            return clipped, 1.0
        _cost, _travel, angle, direction = min(candidates, key=lambda item: item[0])
        return float(angle), float(direction)


class ReversalPhase(Enum):
    """구동 부호 반전 중 감속·조향·재가속의 진행 단계를 나타낸다."""

    NORMAL = 0
    DECELERATING = 1
    STEERING = 2
    ACCELERATING = 3


class SwerveDrive:
    """차체 twist를 액추에이터용 모듈 명령으로 바꾸는 피드백 제어기."""

    def __init__(self, kinematics=None):
        """모듈별 조향 명령, 구동 방향과 반전 FSM 상태를 정지값으로 초기화한다."""
        self.base = BaseTeleop()
        self.kinematics = kinematics or SwerveKinematics()
        self.wheels = tuple(self.kinematics.wheel_positions)
        self.steer_angle = dict.fromkeys(self.wheels, 0.0)
        self.previous_wheel_rotation_direction = dict.fromkeys(self.wheels, 1.0)
        self.wheel_speed_scale = dict.fromkeys(self.wheels, 1.0)
        self.reversal_phase = dict.fromkeys(self.wheels, ReversalPhase.NORMAL)
        self.reversal_target_steering_angle = dict.fromkeys(self.wheels, 0.0)
        self.reversal_target_direction = dict.fromkeys(self.wheels, 1.0)
        self.previous_commands = dict.fromkeys(self.wheels, 0.0)
        self.previous_drive_commands = dict.fromkeys(self.wheels, 0.0)
        self.wheel_saturation_scale = 1.0
        self.last_body_twist = BodyTwist()

    def update(self, keys, dt, yaw=0.0, steering_positions=None, wheel_velocities=None):
        """기존 테스트와 호출부가 사용하는 키보드 입력 호환 경로."""
        del yaw  # 바퀴 역기구학은 차체 좌표계 명령을 사용한다.
        measured_twist = None
        if (steering_positions is not None and wheel_velocities is not None
                and all(name in steering_positions and name in wheel_velocities
                        for name in self.wheels)):
            measured_twist = self.kinematics.forward(steering_positions, wheel_velocities)
        twist = self.base.update_body(keys, dt, measured_twist)
        return self.update_twist(twist, dt, steering_positions, wheel_velocities)

    def update_twist(self, twist, dt, steering_positions=None, wheel_velocities=None):
        """임의의 차체 twist를 제어해 전신 IK가 베이스를 구동하게 한다."""
        if not isinstance(twist, BodyTwist):
            twist = BodyTwist(*(float(v) for v in twist))
        twist = BodyTwist(
            0.0 if abs(twist.vx) < LINEAR_VEL_DEADBAND else twist.vx,
            0.0 if abs(twist.vy) < LINEAR_VEL_DEADBAND else twist.vy,
            0.0 if abs(twist.wz) < ANGULAR_VEL_DEADBAND else twist.wz,
        )
        self.last_body_twist = twist
        if twist.is_zero():
            self.wheel_saturation_scale = 1.0
            self.previous_drive_commands = dict.fromkeys(self.wheels, 0.0)
            return self._hold_zero(steering_positions)

        desired, self.wheel_saturation_scale = self.kinematics.inverse(
            twist, steering_positions or self.previous_commands,
            self.previous_wheel_rotation_direction)
        module_results = {}
        all_aligned = True
        for name, (target_angle, target_wheel_speed) in desired.items():
            steering_cmd, wheel_cmd, aligned = self._control_module(
                name, target_angle, target_wheel_speed, dt,
                steering_positions, wheel_velocities)
            module_results[name] = (steering_cmd, wheel_cmd)
            all_aligned = all_aligned and aligned

        # 모든 모듈이 정렬되기 전에는 추진력을 적용하지 않는 AI Worker의 안전 규칙을
        # 따른다. 조향 도중 차체가 의도하지 않은 방향으로 순간 이동하는 것을 막는다.
        if not all_aligned:
            module_results = {
                name: (angle, 0.0)
                for name, (angle, _speed) in module_results.items()
            }
            self.previous_drive_commands = dict.fromkeys(self.wheels, 0.0)
        else:
            module_results = self._rate_limit_drive_commands(module_results, dt)

        for name, (angle, _speed) in module_results.items():
            self.steer_angle[name] = angle
            self.previous_commands[name] = angle
        return module_results

    def _rate_limit_drive_commands(self, commands, dt):
        """0이 아닌 구동 전환의 변화율을 제한하고 주차 명령은 즉시 0으로 만든다."""
        result = {}
        for name, (steering, target_speed) in commands.items():
            previous = self.previous_drive_commands[name]
            braking = (previous * target_speed >= 0.0
                       and abs(target_speed) < abs(previous))
            if braking and abs(previous) < DRIVE_COMMAND_CREEP_THRESHOLD:
                rate = DRIVE_COMMAND_CREEP_BRAKE_LIMIT
            else:
                rate = DRIVE_COMMAND_BRAKE_LIMIT if braking else DRIVE_COMMAND_ACCEL_LIMIT
            max_change = rate * dt
            speed = previous + float(np.clip(target_speed - previous, -max_change, max_change))
            if abs(speed) < 1e-6:
                speed = 0.0
            self.previous_drive_commands[name] = speed
            result[name] = (steering, speed)
        return result

    def _hold_zero(self, steering_positions):
        """정지 명령에서 반전 상태를 지우고 현재 조향각을 유지한 영속도 명령을 만든다."""
        for name in self.wheels:
            self.reversal_phase[name] = ReversalPhase.NORMAL
            self.wheel_speed_scale[name] = 1.0
            cur = _feedback_value(
                steering_positions, name, self.previous_commands[name])
            self.steer_angle[name] = cur
            self.previous_commands[name] = cur
        return {name: (self.steer_angle[name], 0.0) for name in self.wheels}

    def _control_module(self, name, target_angle, target_wheel_speed, dt,
                        steering_positions, wheel_velocities):
        """모듈 하나에 반전 FSM·조향 변화율·정렬 게이트를 적용한다.

        반환값은 ``(조향 명령, 구동 각속도 명령, 정렬 여부)``다. 실제 조향이 허용
        오차 밖이면 잘못된 방향으로 밀지 않도록 구동 명령을 0으로 만든다.
        """
        current_steering = _feedback_value(
            steering_positions, name, self.previous_commands[name])
        current_wheel_velocity = _feedback_value(wheel_velocities, name, 0.0)
        direction = -1.0 if target_wheel_speed < 0.0 else 1.0
        steering_target = self._update_reversal_phase(
            name, direction, target_angle, current_steering, current_wheel_velocity, dt)
        # 지연된 피드백 위치가 아니라 명령 궤적의 변화율을 제한한다. 실시간 피드백을
        # 기준으로 쓰면 위치 서보 오차와 토크가 일정하게 남아 정지 마찰 때문에 조향이
        # 약 20도 부근에 갇힐 수 있다.
        steering_cmd = _limit_steering_rate(
            self.previous_commands[name], steering_target, dt,
            self.kinematics.steer_range,
        )

        effective_direction = (
            self.previous_wheel_rotation_direction[name]
            if self.reversal_phase[name] == ReversalPhase.DECELERATING else direction
        )
        wheel_cmd = effective_direction * abs(target_wheel_speed) * self.wheel_speed_scale[name]
        align_err = abs(target_angle - current_steering)
        aligned = align_err < STEERING_ALIGNMENT_ANGLE_ERROR_THRESHOLD
        if not aligned:
            wheel_cmd = 0.0
        return steering_cmd, wheel_cmd, aligned

    def _update_reversal_phase(self, name, direction, target, current, wheel_velocity, dt):
        """바퀴 방향 변경을 감속→조향→가속 순서로 진행하고 현재 조향 목표를 반환한다."""
        previous_direction = self.previous_wheel_rotation_direction[name]
        phase = self.reversal_phase[name]

        # 반전이 끝나기 전에 요청 방향이 기존 방향으로 돌아오면 더는 유효하지 않은 상태
        # 순서를 끝까지 실행하지 않고 즉시 취소한다.
        if phase in (ReversalPhase.DECELERATING, ReversalPhase.STEERING):
            if direction == previous_direction:
                self.reversal_phase[name] = ReversalPhase.NORMAL
                self.wheel_speed_scale[name] = 1.0
                return target

        if direction != previous_direction and phase in (
                ReversalPhase.NORMAL, ReversalPhase.ACCELERATING):
            self.reversal_target_steering_angle[name] = target
            self.reversal_target_direction[name] = direction
            if abs(wheel_velocity) < STEERING_ALIGNMENT_START_SPEED_ERROR_THRESHOLD:
                # 정지한 바퀴는 감속할 회전 에너지가 없으므로 부호를 즉시 바꿔 불필요한
                # 제어 지연을 없앤다.
                self.previous_wheel_rotation_direction[name] = direction
                self.reversal_phase[name] = ReversalPhase.NORMAL
                self.wheel_speed_scale[name] = 1.0
                return target
            self.reversal_phase[name] = ReversalPhase.DECELERATING

        phase = self.reversal_phase[name]
        if phase == ReversalPhase.DECELERATING:
            self.reversal_target_direction[name] = direction
            self.reversal_target_steering_angle[name] = target
            self.wheel_speed_scale[name] = max(
                0.0, self.wheel_speed_scale[name] - REVERSAL_DECEL_RATE * dt)
            if self.wheel_speed_scale[name] <= REVERSAL_THRESHOLD:
                self.wheel_speed_scale[name] = 0.0
                self.reversal_phase[name] = ReversalPhase.STEERING
            return current

        if phase == ReversalPhase.STEERING:
            self.reversal_target_steering_angle[name] = target
            self.reversal_target_direction[name] = direction
            self.wheel_speed_scale[name] = 0.0
            if abs(target - current) < STEERING_TOLERANCE:
                self.previous_wheel_rotation_direction[name] = self.reversal_target_direction[name]
                self.reversal_phase[name] = ReversalPhase.ACCELERATING
            return target

        if phase == ReversalPhase.ACCELERATING:
            self.wheel_speed_scale[name] = min(
                1.0, self.wheel_speed_scale[name] + REVERSAL_ACCEL_RATE * dt)
            if self.wheel_speed_scale[name] >= 1.0:
                self.reversal_phase[name] = ReversalPhase.NORMAL
            return target

        self.wheel_speed_scale[name] = 1.0
        return target


def _limit_steering_rate(current, target, dt, steer_range=STEER_RANGE):
    """한 제어 주기 조향 변화량을 속도 한계로 제한하고 허용 각도 범위에 맞춘다."""
    max_change = STEERING_ANGULAR_VELOCITY_LIMIT * dt
    desired = target - current
    if abs(desired) <= max_change:
        return target
    return _clamp(current + math.copysign(max_change, desired), *steer_range)


def _feedback_value(feedback, name, fallback):
    """선택적 바퀴 피드백을 읽고 없으면 명시한 상태값을 사용한다."""
    if feedback is not None and name in feedback:
        return float(feedback[name])
    return fallback


def _normalize_angle(angle):
    """라디안 각도를 ``[-π, π)`` 범위의 동치각으로 정규화한다."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(value, lo, hi):
    """스칼라 값을 닫힌 구간 ``[lo, hi]`` 안으로 제한한다."""
    return min(hi, max(lo, value))
