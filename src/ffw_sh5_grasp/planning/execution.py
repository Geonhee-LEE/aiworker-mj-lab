"""P3: 계획한 ``Trajectory``를 기존 ``ArmTorqueController``로 실제 재생한다.

새 액추에이터 코드는 추가하지 않는다 — ``control.arm.ArmTorqueController``의
토크를 통해서만 팔이 움직이는 진짜 폐루프 재생이다(``data.qpos``를 직접 쓰지
않는다). 매 표본마다 실제로 재생된 configuration을 계획 때와 같은
``ArmCollisionChecker.is_valid`` 계약으로 다시 확인해 "재생 중 침투 없음"을
직접 검증하고, 재생이 끝나면 목표에서 잠시 더 버텨(``ArmTorqueController``
모듈 docstring이 설명하는 수 초 시정수 정착) 최종 site 오차를 잰다.
"""

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass(frozen=True)
class ExecutionReport:
    """``follow_trajectory`` 한 번의 재생 안전성·정확도 보고서."""

    final_site_error_m: float
    max_joint_error_rad: float
    invalid_sample_count: int
    invalid_sample_indices: object  # tuple[int, ...] — 디버깅용
    total_samples: int


def follow_trajectory(model, data, space, checker, controller, trajectory, site_id, *, settle_time_s=1.0):
    """``trajectory``를 물리로 재생하고 ``ExecutionReport``를 반환한다.

    호출자는 이 함수를 부르기 전에 ``data.qpos``를 이미
    ``trajectory.positions[0]``에 맞춰 두어야 한다(``space.write`` +
    ``mujoco.mj_forward``) — 이 함수는 시작 상태를 대신 맞춰 주지 않는다.
    """
    dt = float(model.opt.timestep)

    invalid_indices = []
    for index, q_des in enumerate(trajectory.positions[1:], start=1):
        controller.apply(data, q_des)
        mujoco.mj_step(model, data)
        actual_q = data.qpos[space.qpos_adrs]
        if not checker.is_valid(actual_q):
            invalid_indices.append(index)

    target_q = trajectory.positions[-1]
    settle_steps = max(1, int(round(settle_time_s / dt)))
    for _ in range(settle_steps):
        controller.apply(data, target_q)
        mujoco.mj_step(model, data)

    final_q = data.qpos[space.qpos_adrs]
    background = checker.snapshot_qpos
    actual_qpos = space.write(background.copy(), final_q)
    target_qpos = space.write(background.copy(), target_q)
    actual_site = checker.tree.forward_site(actual_qpos, site_id, space.joint_ids).position
    target_site = checker.tree.forward_site(target_qpos, site_id, space.joint_ids).position

    return ExecutionReport(
        final_site_error_m=float(np.linalg.norm(actual_site - target_site)),
        max_joint_error_rad=float(np.max(np.abs(final_q - target_q))),
        invalid_sample_count=len(invalid_indices),
        invalid_sample_indices=tuple(invalid_indices),
        total_samples=len(trajectory.positions),
    )


__all__ = ["ExecutionReport", "follow_trajectory"]
