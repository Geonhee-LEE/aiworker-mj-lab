"""텔레옵 앱의 목표 자세와 Cyclo 방식 양손 제어 보조 함수.

이 모듈은 같은 명령을 나타내는 세 가지 표현을 연결하는 수학을 담당한다.

- UI 값: ``app.targets``에 저장된 홈 기준 XYZ/RPY 목표
- 렌더링 값: 월드 좌표계의 마커 및 기즈모 자세
- IK 값: ``control.whole_body.WholeBodyIK``에 전달되는 월드 좌표계 site 목표

메인 앱만 조립 지점으로 유지하기 위해 ``teleop_app``을 import하지 않고 전달받은
``app`` 객체에 필요한 속성이 있다고 가정하는 덕 타이핑 방식을 사용한다.
"""

import math

import mujoco
import numpy as np

from ..kinematics import rotations

SIDES = ("r", "l")


def set_home_references(app):
    """현재 양손 pose와 베이스 pose를 UI 목표의 홈·월드 기준으로 저장한다.

    손별 홈 위치·쿼터니언과 전신 제어용 시작 anchor를 ``app``에 기록한다. 이후
    XYZ/RPY 슬라이더의 0은 이 시점의 손 pose를 뜻한다.
    """
    site_states = {
        side: app.whole_body_solver.site_state(app.data, side)
        for side in SIDES
    }
    for side, state in site_states.items():
        setattr(app, f"home_quat_{side}", state.quaternion.copy())
    app.home_pos_local = {
        side: world_to_base_pos(app, state.position)
        for side, state in site_states.items()
    }
    base_x, base_y, base_yaw, _cy, _sy, base_quat = base_pose(app)
    app.target_anchor_xy = np.array([base_x, base_y], dtype=float)
    app.target_anchor_yaw = float(base_yaw)
    app.target_anchor_quat = base_quat.copy()
    app.home_pos_world = {
        side: state.position.copy() for side, state in site_states.items()
    }
    app.home_quat_world = {
        "r": app.home_quat_r.copy(),
        "l": app.home_quat_l.copy(),
    }


def carry_world_targets_with_base(app, previous_base_pose, current_base_pose):
    """측정된 수동 베이스의 SE(2) 이동을 모든 월드 목표 기준에 적용한다.

    전신 목표는 보통 월드 좌표계에 고정되어야 IK가 베이스를 움직여 오차를 줄일 수
    있다. 반면 수동 주행은 로봇 전체의 위치를 옮기는 동작이므로 목표도 차체를 따라
    이동해야 한다. UI 오프셋 대신 기준 좌표를 변환하면 독립 손 목표와 캡처된 양손
    명령도 그대로 보존된다.
    """
    old_x, old_y, old_yaw = (float(v) for v in previous_base_pose)
    new_x, new_y, new_yaw = (float(v) for v in current_base_pose)
    delta_yaw = (new_yaw - old_yaw + math.pi) % (2.0 * math.pi) - math.pi
    c, s = math.cos(delta_yaw), math.sin(delta_yaw)
    old_xy = np.array([old_x, old_y])
    new_xy = np.array([new_x, new_y])

    def transform_position(position):
        """이전 베이스 기준 월드 점을 현재 베이스의 동일한 상대 위치로 옮긴다."""
        result = np.asarray(position, dtype=float).copy()
        relative = result[:2] - old_xy
        result[:2] = new_xy + np.array([
            c * relative[0] - s * relative[1],
            s * relative[0] + c * relative[1],
        ])
        return result

    delta_quat = np.array([math.cos(delta_yaw / 2.0), 0.0, 0.0,
                           math.sin(delta_yaw / 2.0)])
    app.target_anchor_xy = transform_position(app.target_anchor_xy)[:2]
    app.target_anchor_yaw += delta_yaw
    anchor_quat = np.zeros(4)
    mujoco.mju_mulQuat(anchor_quat, delta_quat, app.target_anchor_quat)
    app.target_anchor_quat = anchor_quat
    for side in SIDES:
        app.home_pos_world[side] = transform_position(app.home_pos_world[side])
        app.home_quat_world[side] = rotations.multiply_quaternions(
            delta_quat, app.home_quat_world[side]
        )


def base_pose(app):
    """MuJoCo 베이스 qpos에서 x·y·yaw, 삼각함수와 yaw 쿼터니언을 함께 반환한다."""
    bindings = app.bindings.base
    base_x = app.data.qpos[bindings.x_qpos]
    base_y = app.data.qpos[bindings.y_qpos]
    base_yaw = app.data.qpos[bindings.yaw_qpos]
    cy, sy = math.cos(base_yaw), math.sin(base_yaw)
    base_quat = np.array([math.cos(base_yaw / 2), 0.0, 0.0, math.sin(base_yaw / 2)])
    return base_x, base_y, base_yaw, cy, sy, base_quat


def local_to_world_pos(app, p_local):
    """현재 베이스 좌표의 3차원 점을 MuJoCo 월드 좌표로 변환한다."""
    base_x, base_y, _base_yaw, cy, sy, _base_quat = base_pose(app)
    x, y, z = p_local
    return np.array([base_x + cy * x - sy * y, base_y + sy * x + cy * y, z])


def world_to_base_pos(app, p_world):
    """MuJoCo 월드 좌표의 3차원 점을 현재 베이스 좌표로 역변환한다."""
    base_x, base_y, _base_yaw, cy, sy, _base_quat = base_pose(app)
    dx, dy = p_world[0] - base_x, p_world[1] - base_y
    return np.array([cy * dx + sy * dy, -sy * dx + cy * dy, p_world[2]])


def anchor_local_to_world_pos(app, p_local):
    """움직이는 현재 베이스가 아니라 시작 시점의 베이스 자세를 통해 변환한다."""
    cy, sy = math.cos(app.target_anchor_yaw), math.sin(app.target_anchor_yaw)
    x, y, z = p_local
    return np.array([
        app.target_anchor_xy[0] + cy * x - sy * y,
        app.target_anchor_xy[1] + sy * x + cy * y,
        z,
    ])


def world_to_anchor_local_pos(app, p_world):
    """월드 점을 전신 제어 시작 시 저장한 anchor의 로컬 좌표로 변환한다."""
    cy, sy = math.cos(app.target_anchor_yaw), math.sin(app.target_anchor_yaw)
    dx = p_world[0] - app.target_anchor_xy[0]
    dy = p_world[1] - app.target_anchor_xy[1]
    return np.array([cy * dx + sy * dy, -sy * dx + cy * dy, p_world[2]])


def target_pos_to_world_pos(app, side, pos_target):
    """손별 XYZ 목표값을 현재 제어 모드에 맞는 월드 위치로 변환한다.

    Whole-body 모드에서는 시작 anchor에 고정된 홈을, 팔 전용 모드에서는 움직이는
    현재 베이스에 고정된 홈을 기준으로 사용한다.
    """
    if getattr(app, "whole_body_enabled", False):
        offset = np.asarray(pos_target, dtype=float)
        cy, sy = math.cos(app.target_anchor_yaw), math.sin(app.target_anchor_yaw)
        rotated = np.array([
            cy * offset[0] - sy * offset[1],
            sy * offset[0] + cy * offset[1],
            offset[2],
        ])
        return app.home_pos_world[side] + rotated
    return local_to_world_pos(
        app, app.home_pos_local[side] + np.asarray(pos_target, dtype=float))


def world_to_target_pos(app, side, world_pos):
    """손의 월드 위치를 현재 제어 모드의 XYZ 슬라이더 값으로 역변환한다."""
    if getattr(app, "whole_body_enabled", False):
        delta = np.asarray(world_pos, dtype=float) - app.home_pos_world[side]
        cy, sy = math.cos(app.target_anchor_yaw), math.sin(app.target_anchor_yaw)
        return [cy * delta[0] + sy * delta[1],
                -sy * delta[0] + cy * delta[1], float(delta[2])]
    return (world_to_base_pos(app, world_pos) - app.home_pos_local[side]).tolist()


def target_rpy_to_world_quat(app, side, rpy_deg):
    """활성 목표 좌표계를 사용해 손의 홈 기준 RPY 값을 월드 자세로 변환한다."""
    delta_quat = rotations.rpy_deg_to_quat(rpy_deg)
    if getattr(app, "whole_body_enabled", False):
        return rotations.multiply_quaternions(app.home_quat_world[side], delta_quat)
    *_unused, base_quat = base_pose(app)
    home_quat = getattr(app, f"home_quat_{side}")
    return rotations.multiply_quaternions(base_quat, home_quat, delta_quat)


def target_world_quat(app, side):
    """손별 RPY 슬라이더 목표를 월드 좌표계 쿼터니언으로 반환한다."""
    return target_rpy_to_world_quat(app, side, app.targets[f"rpy_{side}"])


def world_quat_to_target_rpy(app, side, world_quat):
    """월드 쿼터니언을 현재 제어 모드의 손별 홈 기준 RPY 각도로 역변환한다."""
    if getattr(app, "whole_body_enabled", False):
        delta = rotations.multiply_quaternions(
            rotations.inverse_quaternion(app.home_quat_world[side]), world_quat
        )
        return rotations.quat_to_rpy_deg(delta)
    home_quat = getattr(app, f"home_quat_{side}")
    *_unused, base_quat = base_pose(app)
    rpy_delta_quat = rotations.multiply_quaternions(
        rotations.inverse_quaternion(home_quat),
        rotations.inverse_quaternion(base_quat),
        world_quat,
    )
    return rotations.quat_to_rpy_deg(rpy_delta_quat)


def world_quat_to_virtual_rpy(app, world_quat):
    """가상 물체의 월드 쿼터니언을 활성 베이스/anchor 기준 RPY 각도로 바꾼다."""
    if getattr(app, "whole_body_enabled", False):
        base_quat = app.target_anchor_quat
    else:
        *_unused, base_quat = base_pose(app)
    rpy_delta_quat = rotations.multiply_quaternions(
        rotations.inverse_quaternion(base_quat), world_quat
    )
    return rotations.quat_to_rpy_deg(rpy_delta_quat)


def target_world_pose(app, side):
    """손별 UI 목표를 WBIK와 마커가 사용하는 ``(월드 위치, 쿼터니언)``으로 반환한다."""
    return (
        target_pos_to_world_pos(app, side, app.targets[f"pos_{side}"]),
        target_world_quat(app, side),
    )


def virtual_object_world_pose(app):
    """가상 양손 물체의 UI 위치·RPY를 월드 pose로 변환해 반환한다."""
    if getattr(app, "whole_body_enabled", False):
        pos = anchor_local_to_world_pos(app, app.targets["virtual_object_pos"])
        base_quat = app.target_anchor_quat
    else:
        pos = local_to_world_pos(app, app.targets["virtual_object_pos"])
        *_unused, base_quat = base_pose(app)
    quat = rotations.multiply_quaternions(
        base_quat, rotations.rpy_deg_to_quat(app.targets["virtual_object_rpy"])
    )
    return pos, quat


def sync_virtual_object_to_hand_targets(app):
    """가상 물체 목표를 현재 두 손 목표의 중점으로 옮기고 회전 오프셋을 초기화한다."""
    pos_r, _quat_r = target_world_pose(app, "r")
    pos_l, _quat_l = target_world_pose(app, "l")
    midpoint = 0.5 * (pos_r + pos_l)
    if getattr(app, "whole_body_enabled", False):
        app.targets["virtual_object_pos"] = world_to_anchor_local_pos(app, midpoint).tolist()
    else:
        app.targets["virtual_object_pos"] = world_to_base_pos(app, midpoint).tolist()
    app.targets["virtual_object_rpy"] = [0.0, 0.0, 0.0]


def capture_grasp(app):
    """가상 물체 마커를 기준으로 양손 목표 자세를 기록한다."""
    sync_virtual_object_to_hand_targets(app)
    obj_pos, obj_quat = virtual_object_world_pose(app)
    obj_R = rotations.rotation_from_quaternion(obj_quat)
    offsets = {}
    for side in SIDES:
        hand_pos, hand_quat = target_world_pose(app, side)
        offsets[side] = {
            "pos": obj_R.T @ (hand_pos - obj_pos),
            "mat": obj_R.T @ rotations.rotation_from_quaternion(hand_quat),
        }
    app.cyclo_capture_offsets = offsets
    app.cyclo_grasp_captured = True
    app.cyclo_controller = "bimanual_movel"
    app.cyclo_status = "captured virtual object"


def release_grasp(app):
    """캡처한 가상 물체-양손 상대 변환을 지우고 독립 손 목표 모드로 돌아간다."""
    app.cyclo_grasp_captured = False
    app.cyclo_capture_offsets = None
    app.cyclo_status = "released"


def apply_virtual_object_target(app):
    """캡처 당시 상대 변환을 보존하며 가상 물체 pose를 양손 목표 pose로 전개한다."""
    if not app.cyclo_grasp_captured or app.cyclo_capture_offsets is None:
        return
    obj_pos, obj_quat = virtual_object_world_pose(app)
    obj_R = rotations.rotation_from_quaternion(obj_quat)
    for side, offset in app.cyclo_capture_offsets.items():
        hand_pos = obj_pos + obj_R @ offset["pos"]
        hand_quat = rotations.quaternion_from_rotation(obj_R @ offset["mat"])
        app.targets[f"pos_{side}"] = world_to_target_pos(app, side, hand_pos)
        app.targets[f"rpy_{side}"] = world_quat_to_target_rpy(app, side, hand_quat)


def sync_marker_visibility(app):
    """양손 MoveL 캡처 상태에 맞춰 가상 물체 geom/site 마커의 투명도를 갱신한다."""
    bindings = getattr(app, "bindings", None)
    if bindings is None:
        return
    markers = bindings.markers
    visible = (getattr(app, "cyclo_controller", "movel") == "bimanual_movel"
               and bool(getattr(app, "cyclo_grasp_captured", False)))
    alpha_scale = float(visible)
    geom_rgba = markers.virtual_geom_rgba.copy()
    site_rgba = markers.virtual_site_rgba.copy()
    geom_rgba[3] *= alpha_scale
    site_rgba[3] *= alpha_scale
    app.model.geom_rgba[markers.virtual_geom_id] = geom_rgba
    app.model.site_rgba[markers.virtual_site_id] = site_rgba


def active_gizmo_target(app):
    """현재 3D 기즈모가 편집할 오른손·왼손·가상 물체 식별자를 반환한다."""
    if app.cyclo_controller == "bimanual_movel" and app.cyclo_grasp_captured:
        return "virtual"
    side = getattr(app, "jog_side", "r")
    return side if side in ("l", "r") else "r"


def gizmo_target_world_pose(app, target):
    """기즈모 대상 식별자에 대응하는 현재 월드 pose를 반환한다."""
    if target == "virtual":
        return virtual_object_world_pose(app)
    return target_world_pose(app, target)


def set_gizmo_target_world_pose(app, target, world_pos, world_quat):
    """기즈모에서 편집한 월드 pose를 대상의 UI 좌표로 역변환해 저장한다.

    가상 물체를 편집한 경우 캡처된 상대 변환을 이용해 두 손 목표도 즉시 동기화한다.
    """
    if target == "virtual":
        if getattr(app, "whole_body_enabled", False):
            app.targets["virtual_object_pos"] = world_to_anchor_local_pos(app, world_pos).tolist()
        else:
            app.targets["virtual_object_pos"] = world_to_base_pos(app, world_pos).tolist()
        app.targets["virtual_object_rpy"] = world_quat_to_virtual_rpy(app, world_quat)
        apply_virtual_object_target(app)
    else:
        app.targets[f"pos_{target}"] = world_to_target_pos(app, target, world_pos)
        app.targets[f"rpy_{target}"] = world_quat_to_target_rpy(app, target, world_quat)


def sync_ik_mocaps_from_targets(app):
    """수치 UI 목표를 MuJoCo 손·가상 물체 mocap 마커 pose와 가시성에 반영한다."""
    bindings = getattr(app, "bindings", None)
    if bindings is None:
        return
    markers = bindings.markers
    for side, mocap_id in markers.hand_mocap_ids.items():
        pos, quat = target_world_pose(app, side)
        app.data.mocap_pos[mocap_id] = pos
        app.data.mocap_quat[mocap_id] = quat
    pos, quat = virtual_object_world_pose(app)
    app.data.mocap_pos[markers.virtual_mocap_id] = pos
    app.data.mocap_quat[markers.virtual_mocap_id] = quat
    sync_marker_visibility(app)
