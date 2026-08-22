"""TeleopApp의 pose, joint와 kinematic tree 진단 패널."""

import math

import numpy as np
from imgui_bundle import imgui

from ..application import targets

POSE_GRAPH_HISTORY_SECONDS = 10.0
POSE_GRAPH_SERIES_HEIGHT = 55.0
POS_AXES = ("X", "Y", "Z")
RPY_AXES = ("Roll", "Pitch", "Yaw")


def _new_pose_graph():
    """한 손에 대한 빈 target/current pose 시계열을 만든다."""
    return {
        "target_pos": [[], [], []],
        "current_pos": [[], [], []],
        "target_rpy": [[], [], []],
        "current_rpy": [[], [], []],
        "position_error_mm": [],
        "orientation_error_deg": [],
    }


def _ensure_pose_graph_state(app):
    """Pose Graph 탭의 손별 시계열 버퍼를 지연 초기화한다."""
    if hasattr(app, "pose_graph_state"):
        return
    app.pose_graph_side = "r"
    app.pose_graph_state = {side: _new_pose_graph() for side in ("r", "l")}


def _trim_series(series, max_samples):
    """리스트 기반 시계열을 지정 길이 이하로 유지한다."""
    overflow = len(series) - max_samples
    if overflow > 0:
        del series[:overflow]


def _append_pose_graph_sample(app, side):
    """선택한 손의 현재/목표 pose를 같은 target 좌표계로 기록한다."""
    graph = app.pose_graph_state[side]
    target_pos = np.asarray(app.targets[f"pos_{side}"], dtype=float)
    target_rpy = np.asarray(app.targets[f"rpy_{side}"], dtype=float)

    state = app.whole_body_solver.site_state(app.data, side)
    current_pos = np.asarray(
        targets.world_to_target_pos(app, side, state.position), dtype=float)
    current_rpy = np.asarray(
        targets.world_quat_to_target_rpy(app, side, state.quaternion), dtype=float)

    rate_hz = 1.0 / max(float(getattr(app, "frame_dt", 1.0 / 60.0)), 1e-6)
    max_samples = max(32, int(rate_hz * POSE_GRAPH_HISTORY_SECONDS))
    for axis in range(3):
        for key, values in (
            ("target_pos", target_pos),
            ("current_pos", current_pos),
            ("target_rpy", target_rpy),
            ("current_rpy", current_rpy),
        ):
            graph[key][axis].append(float(values[axis]))
            _trim_series(graph[key][axis], max_samples)

    graph["position_error_mm"].append(
        float(np.linalg.norm(target_pos - current_pos) * 1000.0))
    graph["orientation_error_deg"].append(
        float(np.linalg.norm(target_rpy - current_rpy)))
    _trim_series(graph["position_error_mm"], max_samples)
    _trim_series(graph["orientation_error_deg"], max_samples)


def _plot_series_or_wait(label, values, unit):
    """샘플이 충분하면 선 그래프를, 아니면 대기 문구를 표시한다."""
    if len(values) < 2:
        imgui.text(f"{label}: collecting {unit} samples...")
        return
    imgui.plot_lines(
        label,
        np.asarray(values, dtype=np.float32),
        graph_size=imgui.ImVec2(0.0, POSE_GRAPH_SERIES_HEIGHT),
    )


def _draw_pose_axis_group(title, axis_names, target_series, current_series,
                          value_fmt, unit, error_scale=1.0):
    """축별 target/current 시계열과 최신 오차를 묶어서 그린다."""
    imgui.separator_text(title)
    for axis, axis_name in enumerate(axis_names):
        if target_series[axis]:
            current_value = current_series[axis][-1]
            target_value = target_series[axis][-1]
            axis_error = (target_value - current_value) * error_scale
            error_unit = " mm" if error_scale == 1000.0 else " deg"
            imgui.text(
                f"{axis_name}: cur {value_fmt.format(current_value)}{unit}  "
                f"tgt {value_fmt.format(target_value)}{unit}  "
                f"err {value_fmt.format(axis_error)}{error_unit}"
            )
        _plot_series_or_wait(
            f"target##{title}_target_{axis_name}", target_series[axis], unit)
        _plot_series_or_wait(
            f"current##{title}_current_{axis_name}", current_series[axis], unit)


def draw_pose_graph_panel(app):
    """Target pose와 현재 pose의 시계열 비교 그래프를 그린다."""
    _ensure_pose_graph_state(app)
    imgui.text("Pose graph (home-relative target frame)")
    if imgui.radio_button("Right##pose_graph_side_r", app.pose_graph_side == "r"):
        app.pose_graph_side = "r"
    imgui.same_line()
    if imgui.radio_button("Left##pose_graph_side_l", app.pose_graph_side == "l"):
        app.pose_graph_side = "l"
    imgui.same_line()
    if imgui.button("Clear history##pose_graph_clear"):
        app.pose_graph_state[app.pose_graph_side] = _new_pose_graph()

    side = app.pose_graph_side
    _append_pose_graph_sample(app, side)
    graph = app.pose_graph_state[side]
    _draw_pose_axis_group(
        "Position (m)", POS_AXES,
        graph["target_pos"], graph["current_pos"],
        value_fmt="{:+.3f}", unit=" m", error_scale=1000.0)
    _draw_pose_axis_group(
        "Orientation (deg)", RPY_AXES,
        graph["target_rpy"], graph["current_rpy"],
        value_fmt="{:+.1f}", unit=" deg", error_scale=1.0)

    imgui.separator_text("Pose error norm")
    if graph["position_error_mm"]:
        imgui.text(
            f"Position error: {graph['position_error_mm'][-1]:.1f} mm   "
            f"Orientation error: {graph['orientation_error_deg'][-1]:.2f} deg")
    _plot_series_or_wait(
        "position error##pose_graph_pos_norm", graph["position_error_mm"], "mm")
    _plot_series_or_wait(
        "orientation error##pose_graph_ori_norm",
        graph["orientation_error_deg"], "deg")


def draw_joint_monitor(app, data):
    """감시 대상 관절의 현재 위치를 제한 범위 대비 진행 막대로 표시한다."""
    imgui.begin_child("joint_monitor", (0, 0), True)
    for name, qpos_address in app.bindings.monitor_qpos.items():
        value = float(data.qpos[qpos_address])
        lower, upper = app.bindings.monitor_ranges[name]
        fraction = ((value - lower) / (upper - lower)
                    if upper > lower else 0.0)
        fraction = min(1.0, max(0.0, fraction))
        imgui.progress_bar(
            fraction, (200, 0), f"{name} {math.degrees(value):+.1f}deg")
    imgui.end_child()


def _ensure_tree_state(app):
    """트리 필터 상태를 UI 전체 초기화와 독립적으로 준비한다."""
    if not hasattr(app, "kinematic_tree_scope"):
        app.kinematic_tree_scope = "both"
    if not hasattr(app, "kinematic_tree_show_full"):
        app.kinematic_tree_show_full = False


def kinematic_tree_body_ids(app, scope=None, show_full=None):
    """손 선택과 전체 트리 설정에 따라 표시할 body ID를 반환한다."""
    _ensure_tree_state(app)
    tree = app.whole_body_solver.kinematic_tree
    scope = app.kinematic_tree_scope if scope is None else scope
    show_full = app.kinematic_tree_show_full if show_full is None else show_full
    if scope not in {"both", "r", "l"}:
        raise ValueError(f"invalid kinematic tree scope: {scope!r}")
    if show_full:
        return frozenset(range(len(tree.bodies)))

    visible = {0}
    sides = ("r", "l") if scope == "both" else (scope,)
    for side in sides:
        site_id = app.whole_body_solver.site_ids[side]
        visible.update(tree.site_paths[site_id])
    return frozenset(visible)


def _joint_state_text(app, joint):
    """관절 종류에 맞춰 현재 qpos를 사용자 표시 문자열로 만든다."""
    value = float(app.data.qpos[joint.qpos_adr])
    if joint.kind_name == "hinge":
        return f"{math.degrees(value):+.1f} deg"
    if joint.kind_name == "slide":
        return f"{value:+.3f} m"
    return "multi-DOF state"


def _draw_kinematic_body(app, body_id, visible_body_ids,
                         controlled_joint_ids, target_site_ids):
    """Body와 소속 joint/site, 표시 대상 자식 body를 재귀적으로 그린다."""
    tree = app.whole_body_solver.kinematic_tree
    body = tree.bodies[body_id]
    body_name = body.name or "world"
    flags = (
        imgui.TreeNodeFlags_.span_avail_width
        | imgui.TreeNodeFlags_.draw_lines_to_nodes
    )
    if not app.kinematic_tree_show_full or body_id == 0:
        flags |= imgui.TreeNodeFlags_.default_open
    expanded = imgui.tree_node_ex(
        f"{body_name}  [body {body_id}]##kinbody{body_id}", flags)
    if not expanded:
        return

    for joint_id in body.joint_ids:
        joint = tree.joints[joint_id]
        marker = "[controlled] " if joint_id in controlled_joint_ids else ""
        name = joint.name or f"joint {joint_id}"
        imgui.bullet_text(
            f"{marker}{name} <{joint.kind_name}>  {_joint_state_text(app, joint)}")
    for site_id in tree.sites_by_body[body_id]:
        site = tree.sites[site_id]
        marker = "[IK target] " if site_id in target_site_ids else ""
        name = site.name or f"site {site_id}"
        imgui.bullet_text(f"{marker}{name} <site>")
    for child_id in tree.children_by_body[body_id]:
        if child_id in visible_body_ids:
            _draw_kinematic_body(
                app, child_id, visible_body_ids,
                controlled_joint_ids, target_site_ids)
    imgui.tree_pop()


def draw_kinematic_tree(app):
    """손 범위·전체 트리 선택 UI와 필터링된 MJCF 계층을 그린다."""
    _ensure_tree_state(app)
    tree = app.whole_body_solver.kinematic_tree
    imgui.text("Scope")
    for index, (scope, label) in enumerate(
            (("both", "Both arms"), ("r", "Right"), ("l", "Left"))):
        if index:
            imgui.same_line()
        if imgui.radio_button(
                f"{label}##tree_scope_{scope}",
                app.kinematic_tree_scope == scope):
            app.kinematic_tree_scope = scope
    changed, show_full = imgui.checkbox(
        "Show full MJCF tree", app.kinematic_tree_show_full)
    if changed:
        app.kinematic_tree_show_full = show_full

    visible = kinematic_tree_body_ids(app)
    controlled_joint_ids = set(map(int, app.whole_body_solver.joint_ids))
    target_site_ids = set(app.whole_body_solver.site_ids.values())
    imgui.text(
        f"Showing {len(visible)}/{len(tree.bodies)} bodies  |  "
        f"{len(controlled_joint_ids)} controlled joints")
    imgui.text("[controlled] solver column   [IK target] grasp site")
    imgui.separator()
    imgui.begin_child("kinematic_tree_scroll", (0, 0), True)
    _draw_kinematic_body(
        app, 0, visible, controlled_joint_ids, target_site_ids)
    imgui.end_child()


__all__ = [
    "draw_joint_monitor",
    "draw_kinematic_tree",
    "draw_pose_graph_panel",
    "kinematic_tree_body_ids",
]
