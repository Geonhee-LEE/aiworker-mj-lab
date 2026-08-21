"""TeleopApp을 위한 간결한 ImGui 제어·진단 작업 공간.

``application/teleop.py``에서 실제로 독립적인 부분이므로 별도 모듈로 분리했다. 이
모듈은 ``app.targets``, ``app.contact_viz``처럼 이미 공개된 앱 상태만 읽고 쓰며,
물리나 3D 렌더링에는 접근하지 않는다. IK, 파지 시너지와 ``mj_step``의 동작을 알
필요 없이 슬라이더의 현재 값만 다룬다. 순환 import를 피하기 위해 ``teleop_app``을
import하지 않고 ``draw_panel``에 전달된 ``app`` 객체를 덕 타이핑한다. 관련 제어는
하나의 Control Center 안에서 탭으로 묶고 관절·트리 검사는 하나의 Diagnostics 창을
공유한다. 기능마다 별도 운영체제 창을 만들지 않으면서 기본 다중 뷰포트를 유지한다.
"""

import math
import time

from imgui_bundle import imgui
import numpy as np

from ..application import targets
from ..config import SETTINGS
from ..kinematics import rotations
from . import diagnostics, task_space

JOG_POS_STEP_DEFAULT = SETTINGS.number("ui.jog_position_step_m", positive=True)
JOG_RPY_STEP_DEFAULT = SETTINGS.number("ui.jog_rotation_step_deg", positive=True)
# 다음 범위는 제어 이득이 아니라 위젯의 고정 표시 범위다. YAML에는 사용자가 자주
# 바꾸는 조그 기본값만 노출하고 UI 안전 범위는 구현과 함께 관리한다.
HAND_POS_OFFSET_RANGE = (-0.35, 0.35)
HAND_RPY_RANGE = (-90.0, 90.0)
VIRTUAL_POS_RANGE = (-0.2, 1.2)
JOG_POS_STEP_RANGE = (0.001, 0.050)
JOG_RPY_STEP_RANGE = (0.5, 15.0)
PSEUDOINVERSE_RCOND_RANGE = (1e-8, 1e-3)
DLS_DAMPING_RANGE = (0.001, 0.5)
POS_AXES = ("X", "Y", "Z")
RPY_AXES = ("Roll", "Pitch", "Yaw")
TASK_SPACE_SIDES = task_space.SIDES
UI_WINDOW_SPECS = SETTINGS.get("ui.windows")
for _spec in UI_WINDOW_SPECS.values():
    _spec["position"] = tuple(_spec["position"])
    _spec["size"] = tuple(_spec["size"])


def _begin_expanded(title, flags=0):
    """바인딩 버전에 따라 달라지는 ``imgui.begin`` 반환형을 bool로 정규화한다.

    반환값은 단일 bool 또는 ``(expanded, opened)`` 튜플일 수 있다.
    """
    result = imgui.begin(title, None, flags) if flags else imgui.begin(title)
    if isinstance(result, tuple):
        return result[0]
    return result


def _ensure_window_state(app):
    """렌더링 초기화 없이 지속되는 창 표시·필터 상태를 초기화한다."""
    if not hasattr(app, "ui_windows") or set(app.ui_windows) != set(UI_WINDOW_SPECS):
        previous = getattr(app, "ui_windows", {})
        app.ui_windows = {
            "control": any(previous.get(key, False)
                           for key in ("control", "marker", "right_arm",
                                       "left_arm", "robot")),
            "diagnostics": any(previous.get(key, False)
                               for key in ("diagnostics", "joints", "tree")),
        }
        if not previous:
            app.ui_windows = {
                key: spec["visible"] for key, spec in UI_WINDOW_SPECS.items()
            }
    if not hasattr(app, "kinematic_tree_scope"):
        app.kinematic_tree_scope = "both"
    if not hasattr(app, "kinematic_tree_show_full"):
        app.kinematic_tree_show_full = False
    if not hasattr(app, "ui_layout_request"):
        # 상태 창은 MuJoCo 뷰포트에 남기고 모든 도구는 오른쪽 바깥의 운영체제 창으로
        # 시작한다. 이 일회성 요청은 도구를 주 창 안에 가두던 이전 imgui.ini 배치도
        # 덮어쓴다.
        app.ui_layout_request = "detach"
    return app.ui_windows


def _begin_tool_window(app, key):
    """이동·크기 조절 가능한 도구 창을 열고 닫기 버튼 상태를 보존한다."""
    spec = UI_WINDOW_SPECS[key]
    layout_request = app.ui_layout_request
    main_viewport = imgui.get_main_viewport()
    if layout_request == "detach":
        index = tuple(UI_WINDOW_SPECS).index(key)
        # 분리된 각 운영체제 창의 제목 표시줄에 접근할 수 있도록 계단식으로 배치한다.
        position = (
            main_viewport.pos.x + main_viewport.size.x + 24.0 + 36.0 * index,
            main_viewport.pos.y + 24.0 + 36.0 * index,
        )
        imgui.set_next_window_pos(position, imgui.Cond_.always)
    elif layout_request == "main":
        imgui.set_next_window_pos(
            (main_viewport.pos.x + spec["position"][0],
             main_viewport.pos.y + spec["position"][1]),
            imgui.Cond_.always)
    else:
        imgui.set_next_window_pos(
            (main_viewport.pos.x + spec["position"][0],
             main_viewport.pos.y + spec["position"][1]),
            imgui.Cond_.first_use_ever)
    imgui.set_next_window_size(spec["size"], imgui.Cond_.first_use_ever)
    expanded, opened = imgui.begin(spec["title"], app.ui_windows[key])
    app.ui_windows[key] = bool(opened)
    return expanded


def _ik_err_text(app, side):
    """FK 모드인 손은 IK를 아예 안 풀므로 mm 오차 자체가 의미 없다 -- 지난 IK
    모드 시절의 값을 그대로 보여주는 대신 "FK"라고 명시한다."""
    if app.arm_mode[side] == "ik":
        return f"{app.ik_err_mm[side]:.2f}mm"
    return "FK"


def _clamp(value, lo, hi):
    """스칼라 UI 값을 닫힌 구간 ``[lo, hi]`` 안으로 제한한다."""
    return min(hi, max(lo, value))


def _slider_float_clamped(label, value, lo, hi, fmt, flags=0):
    """실수 슬라이더를 그리고 변경값을 범위 안으로 재확인해 ``(변경, 값)``으로 반환한다."""
    changed, value = imgui.slider_float(label, value, lo, hi, fmt, flags)
    if changed:
        value = _clamp(value, lo, hi)
    return changed, value


def _item_tooltip(message):
    """방금 그린 위젯 위에 마우스를 올리면 폭을 제한한 설명을 표시한다."""
    if not imgui.is_item_hovered():
        return
    imgui.begin_tooltip()
    imgui.push_text_wrap_pos(imgui.get_font_size() * 32.0)
    imgui.text_wrapped(message)
    imgui.pop_text_wrap_pos()
    imgui.end_tooltip()


def _draw_vector_sliders(prefix, values, axes, lo, hi, fmt, on_change=None):
    """벡터 각 축의 실수 슬라이더를 그리고 하나라도 바뀌었는지 반환한다."""
    changed_any = False
    for i, axis in enumerate(axes):
        changed, values[i] = _slider_float_clamped(f"{axis}##{prefix}_{axis}", values[i], lo, hi, fmt)
        if changed:
            changed_any = True
            if on_change is not None:
                on_change()
    return changed_any


def _ensure_jog_state(app):
    """이전 저장 상태에 없는 jog 대상과 위치·회전 step 기본값을 지연 초기화한다."""
    if not hasattr(app, "jog_side"):
        app.jog_side = "virtual" if getattr(app, "cyclo_grasp_captured", False) else "r"
    if not hasattr(app, "jog_pos_step_m"):
        app.jog_pos_step_m = JOG_POS_STEP_DEFAULT
    if not hasattr(app, "jog_rpy_step_deg"):
        app.jog_rpy_step_deg = JOG_RPY_STEP_DEFAULT


def _clamp_pose_targets(targets, side):
    """지정 손의 XYZ 오프셋과 RPY 목표를 UI 허용 범위 안으로 제한한다."""
    pos = targets[f"pos_{side}"]
    rpy = targets[f"rpy_{side}"]
    for i in range(3):
        pos[i] = _clamp(pos[i], HAND_POS_OFFSET_RANGE[0], HAND_POS_OFFSET_RANGE[1])
        rpy[i] = _clamp(rpy[i], *HAND_RPY_RANGE)


def _apply_cartesian_jog(app, side, pos_delta=(0.0, 0.0, 0.0), rpy_delta=(0.0, 0.0, 0.0)):
    """선택한 손 또는 가상 물체 목표에 Cartesian 위치·RPY 증분을 적용한다."""
    if side == "virtual":
        pos = app.targets["virtual_object_pos"]
        rpy = app.targets["virtual_object_rpy"]
        for i in range(3):
            pos[i] = _clamp(pos[i] + pos_delta[i], *VIRTUAL_POS_RANGE)
            rpy[i] = _clamp(rpy[i] + rpy_delta[i], *HAND_RPY_RANGE)
        app.apply_virtual_object_target()
        return

    sides = ("l", "r") if side == "both" else (side,)
    for s in sides:
        if app.arm_mode[s] != "ik":
            continue
        pos = app.targets[f"pos_{s}"]
        rpy = app.targets[f"rpy_{s}"]
        for i in range(3):
            pos[i] += pos_delta[i]
            rpy[i] += rpy_delta[i]
        _clamp_pose_targets(app.targets, s)


def _repeat_button(label):
    """버튼을 처음 누른 프레임과 누르고 있는 프레임 모두에서 참을 반환한다."""
    pressed = imgui.button(label)
    active = imgui.is_item_active()
    return pressed or active


def _draw_jog_row(app, title, axis_labels, step, is_rotation=False):
    """XYZ 또는 RPY 각 축의 음·양 방향 반복 jog 버튼 한 줄을 그린다."""
    imgui.text(f"{title} jog")
    for i, axis in enumerate(axis_labels):
        if i:
            imgui.same_line()
        neg = f"{axis}-##jog_{title}_{axis}_neg"
        pos = f"{axis}+##jog_{title}_{axis}_pos"
        if _repeat_button(neg):
            delta = [0.0, 0.0, 0.0]
            delta[i] = -step
            _apply_cartesian_jog(
                app, app.jog_side,
                **({"rpy_delta": delta} if is_rotation else {"pos_delta": delta}))
        imgui.same_line()
        if _repeat_button(pos):
            delta = [0.0, 0.0, 0.0]
            delta[i] = step
            _apply_cartesian_jog(
                app, app.jog_side,
                **({"rpy_delta": delta} if is_rotation else {"pos_delta": delta}))


def _active_marker_choices(app):
    """현재 독립 손/양손 캡처 상태에서 사용자가 선택할 수 있는 마커 목록을 반환한다."""
    if app.cyclo_controller == "bimanual_movel" and app.cyclo_grasp_captured:
        return (("virtual", "Virtual object"),)
    return (("r", "Right goal"), ("l", "Left goal"))


def _selected_marker_label(app):
    """현재 jog 대상에 대응하는 사용자 표시용 마커 이름을 반환한다."""
    choices = dict(_active_marker_choices(app))
    jog_side = getattr(app, "jog_side", None)
    if jog_side in choices:
        return choices[jog_side]
    return next(iter(choices.values()))


def _draw_cyclo_control_panel(app):
    """MoveL 모드, 파지 캡처, 활성 마커와 Cartesian jog 제어 UI를 그린다."""
    _ensure_jog_state(app)
    imgui.text("Controller")
    for controller, label in (("movel", "MoveL"), ("bimanual_movel", "Bimanual MoveL")):
        if controller != "movel":
            imgui.same_line()
        if imgui.radio_button(f"{label}##cyclo{controller}", app.cyclo_controller == controller):
            if controller == "movel" and app.cyclo_grasp_captured:
                app.release_grasp()
            app.cyclo_controller = controller

    if app.cyclo_controller == "bimanual_movel":
        if imgui.button("Release Grasp" if app.cyclo_grasp_captured else "Capture Grasp"):
            if app.cyclo_grasp_captured:
                app.release_grasp()
                app.jog_side = "r"
            else:
                app.capture_grasp()
                app.jog_side = "virtual"
        imgui.text(f"Grasp: {'captured' if app.cyclo_grasp_captured else 'free'}")
        imgui.text(f"Status: {app.cyclo_status}")

    imgui.text("Active marker")
    choices = _active_marker_choices(app)
    if app.jog_side not in {choice[0] for choice in choices}:
        app.jog_side = choices[0][0]
    for i, (side, label) in enumerate(choices):
        if i:
            imgui.same_line()
        if imgui.radio_button(f"{label}##jogside{side}", app.jog_side == side):
            app.jog_side = side
    imgui.text("3D gizmo: arrows = XYZ, rings = Roll/Pitch/Yaw")

    _, app.jog_pos_step_m = _slider_float_clamped(
        "Position step", app.jog_pos_step_m, *JOG_POS_STEP_RANGE, "%.3f m")
    _, app.jog_rpy_step_deg = _slider_float_clamped(
        "RPY step", app.jog_rpy_step_deg, *JOG_RPY_STEP_RANGE, "%.1f deg")

    _draw_jog_row(app, "Position", POS_AXES, app.jog_pos_step_m, is_rotation=False)
    _draw_jog_row(app, "RPY", RPY_AXES, app.jog_rpy_step_deg, is_rotation=True)
    if imgui.button("Reset selected RPY##jog_reset_rpy"):
        if app.jog_side == "virtual":
            app.targets["virtual_object_rpy"] = [0.0, 0.0, 0.0]
            app.apply_virtual_object_target()
        else:
            for side in (("l", "r") if app.jog_side == "both" else (app.jog_side,)):
                if app.arm_mode[side] == "ik":
                    app.targets[f"rpy_{side}"][:] = [0.0, 0.0, 0.0]

    if app.cyclo_controller == "bimanual_movel" and app.cyclo_grasp_captured:
        imgui.separator()
        imgui.text("Virtual object target")
        pos = app.targets["virtual_object_pos"]
        rpy = app.targets["virtual_object_rpy"]
        def apply_virtual_edit():
            """가상 물체 슬라이더 변경을 캡처된 양손 목표에 즉시 반영한다."""
            app.apply_virtual_object_target()
        _draw_vector_sliders(
            "virtual_object_pos", pos, POS_AXES, *VIRTUAL_POS_RANGE,
            "%.3f m", apply_virtual_edit)
        _draw_vector_sliders(
            "virtual_object_rpy", rpy, RPY_AXES, *HAND_RPY_RANGE,
            "%.1f deg", apply_virtual_edit)


def _draw_task_space_panel(app):
    """절대 world-frame 손 pose를 숫자로 입력해 IK 목표로 적용하는 UI를 그린다."""
    task_space.ensure_state(app)
    imgui.text_wrapped(
        "Enter an absolute end-effector pose in the MuJoCo world frame. "
        "Apply Target sends it through target smoothing, whole-body IK, and actuators.")

    imgui.text("End effector")
    for index, (side, label) in enumerate(TASK_SPACE_SIDES):
        if index:
            imgui.same_line()
        if imgui.radio_button(
                f"{label}##task_space_{side}", app.task_space_side == side):
            app.task_space_side = side

    side = app.task_space_side
    position = app.task_space_position[side]
    rpy = app.task_space_rpy[side]
    imgui.separator_text("World position (m)")
    for index, axis in enumerate(POS_AXES):
        changed, value = imgui.input_float(
            f"{axis}##task_space_pos_{side}_{axis}", position[index],
            0.001, 0.010, "%.4f")
        if changed:
            position[index] = value

    imgui.separator_text("World orientation RPY (deg)")
    for index, axis in enumerate(RPY_AXES):
        changed, value = imgui.input_float(
            f"{axis}##task_space_rpy_{side}_{axis}", rpy[index],
            1.0, 10.0, "%.2f")
        if changed:
            rpy[index] = value

    if imgui.button("Load Current Pose##task_space_current"):
        task_space.load_pose(app, side, "current")
        app.task_space_status = "Loaded the measured end-effector pose."
    imgui.same_line()
    if imgui.button("Load Target Pose##task_space_target"):
        task_space.load_pose(app, side, "target")
        app.task_space_status = "Loaded the active IK target pose."

    if imgui.button("Apply Target##task_space_apply"):
        _ok, app.task_space_status = task_space.apply_target(
            app, side, position, rpy)
    _item_tooltip(
        "Applies this hand independently. If a bimanual grasp is captured, it is released first.")
    imgui.text_wrapped(app.task_space_status)

    state = app.whole_body_solver.site_state(app.data, side)
    current_rpy = rotations.quat_to_rpy_deg(state.quaternion)
    imgui.separator_text("Measured end-effector pose")
    imgui.text(
        f"XYZ  {state.position[0]:+.4f}  {state.position[1]:+.4f}  "
        f"{state.position[2]:+.4f} m")
    imgui.text(
        f"RPY  {current_rpy[0]:+.2f}  {current_rpy[1]:+.2f}  "
        f"{current_rpy[2]:+.2f} deg")


def _draw_status_panel(app, data):
    """시간·IK 오차·베이스 명령·충돌 CBF와 키 도움말을 상태 패널에 표시한다."""
    imgui.text(f"CAN  |  {app.cyclo_controller}  |  marker: {_selected_marker_label(app)}")
    imgui.text(f"sim {data.time:6.1f}s  wall {time.perf_counter()-app.wall_start:6.1f}s  "
               f"{app.freq_ema:4.1f} Hz")
    method = app.whole_body_solver.solver_method
    imgui.text(
        f"IK [{method}]  L: {_ik_err_text(app, 'l')}   R: {_ik_err_text(app, 'r')}")
    base_bindings = app.bindings.base
    imgui.text(
        f"Base x={data.qpos[base_bindings.x_qpos]:+.2f}m "
        f"y={data.qpos[base_bindings.y_qpos]:+.2f}m "
        f"yaw={math.degrees(data.qpos[base_bindings.yaw_qpos]):+.1f}deg")
    body_cmd = getattr(app, "commanded_base_twist", None)
    if body_cmd is not None:
        whole_body_state = "ON" if getattr(app, "whole_body_enabled", True) else "OFF (arm-only)"
        imgui.text(f"Whole-body IK {whole_body_state}  |  body cmd vx={body_cmd.vx:+.2f} "
                   f"vy={body_cmd.vy:+.2f} wz={body_cmd.wz:+.2f}")
    if getattr(app, "collision_viz", False):
        active = len(getattr(app, "collision_active_pairs", ()))
        distance = getattr(app, "collision_min_distance", math.inf)
        buffer_mm = 1000.0 * app.whole_body_solver.collision_buffer
        distance_text = (f"min {distance*1000:.1f}mm" if math.isfinite(distance)
                         else f"clear >{buffer_mm:.0f}mm")
        violation = getattr(app, "collision_constraint_violation", 0.0)
        imgui.text(f"Collision CBF viz ON  |  active {active}  |  {distance_text}  |  "
                   f"slack {violation:.4f}m/s")
    imgui.text("Keys: arrows drive/yaw, [/] strafe, Q/E lift, R reset, G contacts, V collision, C camera")


def _draw_ik_pose_controls(app, targets, side):
    """지정 손의 홈 기준 위치 오프셋과 RPY IK 목표 슬라이더를 그린다."""
    pos = targets[f"pos_{side}"]
    rpy = targets[f"rpy_{side}"]
    imgui.text("Position offset from home (startup/world anchor)")
    _draw_vector_sliders(f"{side}_pos", pos, POS_AXES,
                         HAND_POS_OFFSET_RANGE[0], HAND_POS_OFFSET_RANGE[1], "%.3f m")
    imgui.text("Orientation RPY (home-relative)")
    _draw_vector_sliders(
        f"{side}_rpy", rpy, RPY_AXES, *HAND_RPY_RANGE, "%.1f deg")
    if imgui.button(f"Reset RPY##{side}"):
        rpy[0], rpy[1], rpy[2] = 0.0, 0.0, 0.0


def _draw_fk_joint_controls(app, side):
    """지정 팔의 각 관절 목표를 도 단위 FK 슬라이더로 그린다."""
    imgui.text("Joint angles (deg)")
    fk_deg = app.fk_q_deg[side]
    for i, (lo, hi) in enumerate(app.arm_joint_ranges_deg[side]):
        _, fk_deg[i] = _slider_float_clamped(f"J{i+1}##{side}fk", fk_deg[i], lo, hi, "%.1f deg")


def _draw_arm_panel(app, targets, side):
    """한 팔의 IK/FK 모드 전환 버튼과 선택된 모드의 목표 제어 UI를 그린다."""
    mode = app.arm_mode[side]
    imgui.text(f"Mode: {'IK pose' if mode == 'ik' else 'FK joints'}")
    imgui.same_line()
    if imgui.button(f"Switch to {'FK' if mode == 'ik' else 'IK'}##{side}mode"):
        app.set_arm_mode(side, "fk" if mode == "ik" else "ik")

    if mode == "ik":
        _draw_ik_pose_controls(app, targets, side)
    else:
        _draw_fk_joint_controls(app, side)


def _draw_can_grasp_panel(app, targets):
    """양손의 원터치 파지 버튼과 grasp·thumb 연속 명령 슬라이더를 그린다."""
    for side, label in (("r", "Right"), ("l", "Left")):
        if side == "l":
            imgui.separator()
        if imgui.button(f"{'Release' if app.grab_state[side] else 'Grab'} {label}##grab{side}"):
            app.grab_state[side] = not bool(app.grab_state[side])
        changed, targets[f"grasp_{side}"] = imgui.slider_float(
            f"{label} grasp##{side}", targets[f"grasp_{side}"], 0.0, 1.0)
        if changed:
            app.grab_state[side] = None
        changed, targets[f"thumb_{side}"] = imgui.slider_float(
            f"{label} thumb##{side}", targets[f"thumb_{side}"], 0.0, 1.0)
        if changed:
            app.grab_state[side] = None


def _draw_lift_utils_panel(app, targets):
    """전신 제어 전환, 리프트 목표, reset·진단·카메라 유틸리티를 그린다."""
    whole_body_enabled = getattr(app, "whole_body_enabled", True)
    button_label = ("Whole-body Control: ON##wholebody"
                    if whole_body_enabled else "Whole-body Control: OFF (arm-only)##wholebody")
    if imgui.button(button_label):
        app.toggle_whole_body_control()
    imgui.same_line()
    imgui.text("base + lift join IK" if whole_body_enabled else "base + lift excluded from IK")
    _, targets["lift"] = _slider_float_clamped(
        "Lift target", targets["lift"], app.lift_range[0], app.lift_range[1], "%.3f m")
    if imgui.button("Reset Can (R)"):
        app.reset_active_object()
    imgui.same_line()
    if imgui.button("Contact Viz (G)"):
        app.contact_viz = not app.contact_viz
    imgui.same_line()
    changed, collision_viz = imgui.checkbox(
        "Collision CBF Viz (V)", getattr(app, "collision_viz", False))
    if changed:
        app.collision_viz = collision_viz
    imgui.same_line()
    if imgui.button("Camera (C)"):
        app.cycle_camera()


def _draw_ik_solver_panel(app):
    """Differential IK 해법과 실행 중 조절 가능한 QP 가중치를 그린다."""
    solver = app.whole_body_solver
    imgui.text("Nominal differential IK method")
    choices = (
        ("pseudoinverse", "Pseudoinverse"),
        ("dls", "DLS"),
        ("qp", "QP"),
    )
    for index, (method, label) in enumerate(choices):
        if index:
            imgui.same_line()
        if imgui.radio_button(
                f"{label}##ik_method_{method}", solver.solver_method == method):
            solver.set_solver_method(method)

    if solver.solver_method == "pseudoinverse":
        changed, value = _slider_float_clamped(
            "SVD rcond", solver.differential_solver.pseudoinverse_rcond,
            *PSEUDOINVERSE_RCOND_RANGE, "%.1e")
        if changed:
            solver.differential_solver.pseudoinverse_rcond = value
        imgui.text("Box/CBF safety constraints are projected after the minimum-norm solve.")
        return

    if solver.solver_method == "dls":
        changed, value = _slider_float_clamped(
            "DLS damping", solver.differential_solver.dls_damping,
            *DLS_DAMPING_RANGE, "%.3f")
        if changed:
            solver.set_dls_damping(value)
        imgui.text("Box/CBF safety constraints are projected after the damped solve.")
        return

    imgui.text("QP dimensionless strengths")
    imgui.text_wrapped(
        "Cost = strength * (residual / matching speed limit)^2. "
        "Hover a slider for its meaning.")
    imgui.text_wrapped(
        "Hard constraints: velocity bounds, joint-limit CBF, fixed mode DOFs. "
        "All sliders below tune soft costs; collision uses a penalized CBF slack.")
    logarithmic = imgui.SliderFlags_.logarithmic
    weight_specs = (
        ("position", "Position", 1e-3, 1e4, "%.4g", logarithmic,
         "Tracks commanded hand linear velocity. Residual is divided by the "
         "maximum task linear speed (m/s). Higher means smaller position error."),
        ("orientation", "Orientation", 1e-3, 1e4, "%.4g", logarithmic,
         "Tracks commanded hand angular velocity. Residual is divided by the "
         "maximum task angular speed (rad/s). Higher means smaller rotation error."),
        ("rigid_grasp_position", "Rigid grasp position", 1e-3, 1e4,
         "%.4g", logarithmic,
         "Preserves the captured relative translation between both hands. "
         "Its residual is divided by the maximum linear speed (m/s)."),
        ("rigid_grasp_orientation", "Rigid grasp orientation", 1e-3, 1e4,
         "%.4g", logarithmic,
         "Preserves the captured relative rotation between both hands. "
         "Its residual is divided by the maximum angular speed (rad/s)."),
        ("damping_base_linear", "Damping base XY", 1e-4, 1e3,
         "%.4g", logarithmic,
         "Penalizes base X/Y speed divided by its velocity limit. Higher values "
         "make translation participate less."),
        ("damping_base_yaw", "Damping base yaw", 1e-4, 1e3,
         "%.4g", logarithmic,
         "Penalizes base yaw speed divided by its velocity limit. Higher values "
         "make base rotation participate less."),
        ("damping_lift", "Damping lift", 1e-4, 1e3, "%.4g", logarithmic,
         "Penalizes lift speed divided by its velocity limit. Higher values make "
         "the lift participate less."),
        ("damping_arm", "Damping arms", 1e-4, 1e3, "%.4g", logarithmic,
         "Penalizes each arm joint speed divided by its velocity limit. Higher "
         "values make both arms participate less."),
        ("posture_lift", "Posture lift", 1e-4, 1e3, "%.4g", logarithmic,
         "Tracks the lift's nominal recovery velocity after normalization. Higher "
         "values pull the lift toward its nominal position more strongly."),
        ("posture_arm", "Posture arms", 1e-4, 1e3, "%.4g", logarithmic,
         "Tracks each arm's nominal recovery velocity after normalization. Higher "
         "values pull the arms toward their nominal posture more strongly."),
        ("collision_slack", "Collision CBF slack", 1e-2, 1e5,
         "%.4g", logarithmic,
         "Penalizes violation of the collision distance-rate CBF after dividing "
         "it by the maximum task linear speed. Higher is safer but can be stiffer."),
    )
    weights = solver.qp_weights()
    for name, label, lower, upper, fmt, flags, help_text in weight_specs:
        changed, value = _slider_float_clamped(
            f"{label}##qp_weight_{name}", weights[name], lower, upper, fmt, flags)
        _item_tooltip(help_text)
        if changed:
            solver.set_qp_weight(name, value)


def _draw_act_policy_panel(app):
    """Select and launch a trained ACT checkpoint under ``outputs/act``."""
    imgui.text_wrapped(
        "Select a training run and checkpoint. ACT will use this teleop "
        "window and the current MuJoCo model directly.")
    if imgui.button("Refresh models##act_policy"):
        app.refresh_act_policies()

    runs = app.act_policy_runs
    if not runs:
        imgui.text_wrapped(app.act_policy_status)
        return

    run_names = [run.name for run in runs]
    changed, run_index = imgui.combo(
        "Training run", app.act_policy_run_index, run_names)
    if changed:
        app.act_policy_run_index = run_index
        app.act_policy_checkpoint_index = 0

    selected_run = runs[app.act_policy_run_index]
    checkpoint_names = [path.name for path in selected_run.checkpoints]
    changed, checkpoint_index = imgui.combo(
        "Checkpoint", app.act_policy_checkpoint_index, checkpoint_names)
    if changed:
        app.act_policy_checkpoint_index = checkpoint_index

    changed, max_steps = imgui.input_int(
        "Max steps", app.act_policy_max_steps, 10, 100)
    if changed:
        app.act_policy_max_steps = max(1, max_steps)
        if (app.act_policy_env is not None
                and app.act_policy_frame >= app.act_policy_max_steps):
            app.request_finish_act_policy("Max steps reached")

    checkpoint = selected_run.checkpoints[app.act_policy_checkpoint_index]
    imgui.text_wrapped(f"Selected: {checkpoint.relative_to(selected_run.path)}")
    if imgui.button("Load + Run ACT Policy"):
        app.request_act_policy_launch(checkpoint, app.act_policy_max_steps)
    imgui.text_wrapped(app.act_policy_status)

    if app.act_policy_env is None:
        return
    imgui.separator_text("Active rollout")
    imgui.text_wrapped(f"Model: {app.act_policy_checkpoint.name}")
    imgui.text(
        f"Frame: {app.act_policy_frame} / {app.act_policy_max_steps}")
    placed = app.act_policy_observation["task"]["success"]
    imgui.text(f"Can settled in box: {'YES' if placed else 'NO'}")
    if imgui.button("Pause" if app.act_policy_running else "Run policy"):
        app.toggle_act_policy()
    imgui.same_line()
    if imgui.button("Step [N]##act_policy"):
        app.request_act_policy_step()
    imgui.same_line()
    if imgui.button("Reset can [R]##act_policy"):
        app.reset_act_policy()
    if imgui.button("Stop + Return to IK"):
        app.request_finish_act_policy("Stopped by operator")
    imgui.text("SPACE: run/pause | N: one step | R: reset can")
    imgui.text_wrapped(
        "Placing the can does not stop the rollout; the learned home-return "
        "sequence continues until Max steps.")


def _draw_window_visibility(app):
    """도구 창 분리·복귀와 작업 공간별 표시 여부를 제어하는 UI를 그린다."""
    imgui.separator_text("Workspaces")
    if imgui.button("Detach tools outside"):
        app.ui_layout_request = "detach"
    imgui.same_line()
    if imgui.button("Return tools to main"):
        app.ui_layout_request = "main"

    if imgui.button("Show all"):
        for key in app.ui_windows:
            app.ui_windows[key] = True
    imgui.same_line()
    if imgui.button("Control only"):
        for key in app.ui_windows:
            app.ui_windows[key] = key == "control"
    imgui.same_line()
    if imgui.button("Hide all"):
        for key in app.ui_windows:
            app.ui_windows[key] = False

    for index, (key, spec) in enumerate(UI_WINDOW_SPECS.items()):
        if index % 2:
            imgui.same_line()
        changed, visible = imgui.checkbox(
            f"{spec['title']}##window_{key}", app.ui_windows[key])
        if changed:
            app.ui_windows[key] = visible


def _draw_status_window(app, data):
    """주 viewport 좌상단에 항상 고정되는 상태·창 관리 도구 창을 그린다."""
    main_viewport = imgui.get_main_viewport()
    imgui.set_next_window_pos(
        (main_viewport.pos.x + 10.0, main_viewport.pos.y + 10.0),
        imgui.Cond_.always)
    imgui.set_next_window_size((550, 275), imgui.Cond_.first_use_ever)
    if _begin_expanded("FFW-SH5 Status & Windows"):
        _draw_status_panel(app, data)
        _draw_window_visibility(app)
    imgui.end()


def _draw_if_visible(app, key, draw_contents):
    """지정 작업 창이 활성일 때만 창을 열고 전달받은 내용 그리기 함수를 호출한다."""
    if not app.ui_windows[key]:
        return
    expanded = _begin_tool_window(app, key)
    if expanded:
        draw_contents()
    imgui.end()


def _draw_tab(label, draw_contents):
    """바인딩별 튜플 처리를 이 함수에 한정하고 선택된 탭 하나를 그린다."""
    selected, _ = imgui.begin_tab_item(label)
    if selected:
        draw_contents()
        imgui.end_tab_item()


def _draw_control_center(app, targets):
    """일반 작업자의 제어 흐름을 하나의 기본 탭 창으로 묶는다."""
    if not imgui.begin_tab_bar("control_center_tabs"):
        return
    _draw_tab("Target", lambda: _draw_cyclo_control_panel(app))
    _draw_tab("Task Space", lambda: _draw_task_space_panel(app))
    _draw_tab(
        f"Right Arm ({app.arm_mode['r'].upper()})###right_arm_tab",
        lambda: _draw_arm_panel(app, targets, "r"))
    _draw_tab(
        f"Left Arm ({app.arm_mode['l'].upper()})###left_arm_tab",
        lambda: _draw_arm_panel(app, targets, "l"))
    _draw_tab("Pose Graph", lambda: diagnostics.draw_pose_graph_panel(app))
    _draw_tab("IK Solver", lambda: _draw_ik_solver_panel(app))
    _draw_tab("ACT Policy", lambda: _draw_act_policy_panel(app))

    def draw_robot_controls():
        """리프트·유틸리티와 캔 파지 제어를 Robot/Grasp 탭에 묶어 그린다."""
        imgui.separator_text("Lift / Utilities")
        _draw_lift_utils_panel(app, targets)
        imgui.separator_text("Can Grasp")
        _draw_can_grasp_panel(app, targets)

    _draw_tab("Robot / Grasp", draw_robot_controls)
    imgui.end_tab_bar()


def _draw_diagnostics(app, data):
    """사용 빈도가 낮고 스크롤이 긴 검사 도구를 제어 흐름과 분리한다."""
    if not imgui.begin_tab_bar("diagnostics_tabs"):
        return
    _draw_tab("Kinematic Tree", lambda: diagnostics.draw_kinematic_tree(app))
    _draw_tab("Joint Monitor", lambda: diagnostics.draw_joint_monitor(app, data))
    imgui.end_tab_bar()


def draw_panel(app):
    """탭 작업 공간 두 개를 그리고 UI 변경을 앱 상태에 기록한다."""
    targets = app.targets
    data = app.data
    _ensure_window_state(app)
    _draw_status_window(app, data)
    _draw_if_visible(app, "control", lambda: _draw_control_center(app, targets))
    _draw_if_visible(app, "diagnostics", lambda: _draw_diagnostics(app, data))
    app.ui_layout_request = None
