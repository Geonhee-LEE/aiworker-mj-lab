"""Task Space 숫자 입력을 IK 목표 상태로 변환한다.

이 모듈은 ImGui를 알지 않는다. UI 편집 버퍼 초기화, world pose 읽기와 목표 적용만
담당하므로 headless 테스트와 다른 입력 장치에서도 같은 경로를 재사용할 수 있다.
"""

import numpy as np

from ..application import targets
from ..kinematics import rotations


SIDES = (("r", "Right hand"), ("l", "Left hand"))
SIDE_LABELS = dict(SIDES)


def load_pose(app, side, source="target"):
    """손의 world pose를 Task Space 편집 버퍼에 복사한다.

    ``source="target"``은 현재 명령 목표를, ``source="current"``는 실제 손끝
    site를 읽는다. 편집 중인 값은 이 함수가 호출될 때만 갱신된다.
    """
    if side not in SIDE_LABELS:
        raise ValueError(f"Unknown hand side: {side!r}")
    if source == "target":
        position, quaternion = targets.target_world_pose(app, side)
    elif source == "current":
        state = app.whole_body_solver.site_state(app.data, side)
        position, quaternion = state.position, state.quaternion
    else:
        raise ValueError(f"Unknown task-space pose source: {source!r}")
    app.task_space_position[side] = np.asarray(position, dtype=float).tolist()
    app.task_space_rpy[side] = rotations.quat_to_rpy_deg(quaternion)


def ensure_state(app):
    """Task Space 편집 버퍼와 선택 상태를 필요할 때 한 번 초기화한다."""
    if not hasattr(app, "task_space_position"):
        app.task_space_position = {}
        app.task_space_rpy = {}
        for side, _label in SIDES:
            load_pose(app, side)
    if not hasattr(app, "task_space_side") or app.task_space_side not in SIDE_LABELS:
        app.task_space_side = "r"
    if not hasattr(app, "task_space_status"):
        app.task_space_status = "Load or edit a world pose, then press Apply Target."


def apply_target(app, side, world_position, world_rpy):
    """유한한 world XYZ/RPY를 기존 IK 목표 경로에 적용한다.

    성공 여부와 사용자 메시지를 반환한다. 실제 이동은 수행하지 않으며 target
    smoothing, IK와 actuator 적용은 메인 제어 루프가 담당한다.
    """
    position = np.asarray(world_position, dtype=float)
    rpy = np.asarray(world_rpy, dtype=float)
    if position.shape != (3,) or rpy.shape != (3,) or not np.all(
            np.isfinite(np.concatenate((position, rpy)))):
        return False, "Rejected: XYZ and RPY must contain three finite numbers."
    if side not in SIDE_LABELS:
        return False, f"Rejected: unknown hand {side!r}."

    target_position = np.asarray(
        targets.world_to_target_pos(app, side, position), dtype=float)

    # 캡처된 가상 물체가 다음 프레임에 양손 목표를 덮어쓰지 않게 독립 MoveL로
    # 전환한다. FK 팔은 현재 pose에서 IK로 전환한 다음 새 목표를 기록한다.
    if getattr(app, "cyclo_grasp_captured", False):
        app.release_grasp()
        app.cyclo_controller = "movel"
    if app.arm_mode[side] != "ik":
        app.set_arm_mode(side, "ik")

    quaternion = rotations.rpy_deg_to_quat(rpy)
    app.targets[f"pos_{side}"] = target_position.tolist()
    target_rpy = np.asarray(
        targets.world_quat_to_target_rpy(app, side, quaternion), dtype=float)
    # 같은 회전을 나타내는 표현 중 평활화 상태와 가장 가까운 각도를 고른다.
    smoothed_rpy = np.asarray(app.smoothed_rpy[side], dtype=float)
    target_rpy += 360.0 * np.round((smoothed_rpy - target_rpy) / 360.0)
    app.targets[f"rpy_{side}"] = target_rpy.tolist()
    targets.sync_ik_mocaps_from_targets(app)
    return True, f"Applied {SIDE_LABELS[side]} world pose; IK is tracking it."


__all__ = ["SIDES", "apply_target", "ensure_state", "load_pose"]
