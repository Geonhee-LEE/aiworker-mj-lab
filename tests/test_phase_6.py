"""Phase 6 캔 전용 Cyclo 마커와 UI 통합 검사.

실시간 텔레옵 앱은 캔 파지와 Cyclo 방식 손 목표 제어라는 하나의 물체 흐름을 가진다.
숫자 X/Y/Z와 Roll/Pitch/Yaw 목표가 제어 입력인지, 보이는 마커가 이 목표에서
동기화되는지, 양손 MoveL이 양손 목표를 캡처해 가상 물체 마커로 이동할 수 있는지
검사한다.

Headless 실행: ``python3 tests/test_phase_6.py``
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "full_scene.xml"

import teleop_app  # noqa: E402
from ffw_sh5_grasp.application import targets as teleop_targets  # noqa: E402
from ffw_sh5_grasp.control import base  # noqa: E402
from ffw_sh5_grasp.kinematics import rotations, tasks as pose_tasks  # noqa: E402
from ffw_sh5_grasp.visualization import render as teleop_render  # noqa: E402
from ffw_sh5_grasp.visualization import ui as teleop_ui  # noqa: E402

ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
HOME_Q_R = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
HOME_Q_L = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])


def _make_sim_only_app():
    """창 없이 모델·제어 상태만 초기화한 테스트용 ``TeleopApp``을 반환한다."""
    app = teleop_app.TeleopApp.__new__(teleop_app.TeleopApp)
    app._setup_sim()
    return app


def _set_hand_base_target(app, side, base_pos):
    """베이스 좌표의 손 목표를 현재 UI target 표현으로 변환해 저장한다."""
    app.targets[f"pos_{side}"] = (np.array(base_pos) - app.home_pos_local[side]).tolist()


def run_model_gate(model):
    """Phase 6에 필요한 자유 캔과 mocap 가상 마커가 모델에 존재하는지 검사한다."""
    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    virtual_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "virtual_object_marker")
    ok = can_jid != -1 and virtual_body != -1 and model.nq == len(model.key_qpos[0])
    print(f"Model gate: can_free={can_jid} nq={model.nq} key_qpos={len(model.key_qpos[0])} "
          f"virtual_marker={virtual_body}: {'OK' if ok else 'FAIL'}")
    return ok


def run_cyclo_marker_jog_gate():
    """Cyclo jog가 손·가상 물체 목표를 올바르게 변경하고 범위를 지키는지 검사한다."""
    class FakeApp:
        """UI jog 함수에 필요한 최소 target 상태만 제공하는 테스트 대역이다."""

        arm_mode = {"l": "ik", "r": "ik"}
        cyclo_controller = "movel"
        cyclo_grasp_captured = False
        targets = {
            "pos_l": [0.0, 0.0, 0.0],
            "pos_r": [0.0, 0.0, 0.0],
            "rpy_l": [0.0, 0.0, 0.0],
            "rpy_r": [0.0, 0.0, 0.0],
            "virtual_object_pos": [0.30, 0.0, 0.85],
            "virtual_object_rpy": [0.0, 0.0, 0.0],
        }

        def apply_virtual_object_target(self):
            """가상 물체 적용 callback을 부작용 없이 대체한다."""
            return None

    app = FakeApp()
    teleop_ui._apply_cartesian_jog(
        app, "both", pos_delta=(0.005, -0.010, 0.015), rpy_delta=(1.0, -2.0, 3.0))
    both_ok = (
        np.allclose(app.targets["pos_l"], [0.005, -0.010, 0.015])
        and np.allclose(app.targets["pos_r"], [0.005, -0.010, 0.015])
        and np.allclose(app.targets["rpy_l"], [1.0, -2.0, 3.0])
        and np.allclose(app.targets["rpy_r"], [1.0, -2.0, 3.0])
    )

    app.arm_mode["l"] = "fk"
    teleop_ui._apply_cartesian_jog(app, "both", pos_delta=(1.0, 1.0, 1.0),
                                   rpy_delta=(100.0, 100.0, 100.0))
    fk_skip_and_clamp_ok = (
        np.allclose(app.targets["pos_l"], [0.005, -0.010, 0.015])
        and np.allclose(app.targets["rpy_l"], [1.0, -2.0, 3.0])
        and np.allclose(app.targets["pos_r"], [0.35, 0.35, 0.35])
        and np.allclose(app.targets["rpy_r"], [90.0, 90.0, 90.0])
    )
    move_marker_ok = (
        teleop_ui._active_marker_choices(app) == (("r", "Right goal"), ("l", "Left goal"))
        and teleop_ui._selected_marker_label(app) == "Right goal"
    )
    app.cyclo_controller = "bimanual_movel"
    app.cyclo_grasp_captured = True
    app.jog_side = "r"
    virtual_marker_ok = (
        teleop_ui._active_marker_choices(app) == (("virtual", "Virtual object"),)
        and teleop_ui._selected_marker_label(app) == "Virtual object"
    )
    ok = both_ok and fk_skip_and_clamp_ok and move_marker_ok and virtual_marker_ok
    print(f"Cyclo marker jog gate: both_updates={both_ok} "
          f"fk_skip_and_clamp={fk_skip_and_clamp_ok} marker_choices={move_marker_ok and virtual_marker_ok}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_initial_ik_target_origin_gate():
    """앱 시작 시 양손 IK 목표가 실제 손 pose와 정확히 겹치는지 검사한다."""
    app = _make_sim_only_app()
    offsets_zero = (
        np.allclose(app.targets["pos_r"], [0.0, 0.0, 0.0])
        and np.allclose(app.targets["pos_l"], [0.0, 0.0, 0.0])
    )
    pose_matches = True
    reports = []
    for side in ("r", "l"):
        pos, quat = teleop_targets.target_world_pose(app, side)
        state = app.whole_body_solver.site_state(app.data, side)
        pos_err = float(np.linalg.norm(pos - state.position))
        quat_dot = abs(float(np.dot(quat, state.quaternion)))
        case_ok = pos_err < 1e-12 and (1.0 - quat_dot) < 1e-12
        pose_matches = pose_matches and case_ok
        reports.append(f"{side}: pos_err={pos_err*1000:.6f}mm quat_dot={quat_dot:.12f}")
    ok = offsets_zero and pose_matches
    print(f"Initial IK target origin gate: offsets_zero={offsets_zero} "
          f"{'; '.join(reports)}: {'OK' if ok else 'FAIL'}")
    return ok


def run_cyclo_bimanual_virtual_object_gate():
    """양손 캡처·가상 물체 이동·release 상태 전환의 pose 보존을 검사한다."""
    app = _make_sim_only_app()
    _set_hand_base_target(app, "r", [0.34, -0.08, 0.88])
    _set_hand_base_target(app, "l", [0.34, 0.08, 0.88])
    app.targets["rpy_r"] = [0.0, 0.0, 0.0]
    app.targets["rpy_l"] = [0.0, 0.0, 0.0]
    r0 = teleop_targets.target_world_pose(app, "r")[0]
    l0 = teleop_targets.target_world_pose(app, "l")[0]

    app.capture_grasp()
    capture_ok = app.cyclo_grasp_captured and app.cyclo_controller == "bimanual_movel"
    rel0 = l0 - r0
    app.targets["virtual_object_pos"][0] += 0.025
    app.targets["virtual_object_pos"][2] += 0.060
    app.targets["virtual_object_rpy"][2] += 12.0
    app.apply_virtual_object_target()
    r1 = teleop_targets.target_world_pose(app, "r")[0]
    l1 = teleop_targets.target_world_pose(app, "l")[0]
    rel1 = l1 - r1
    rel_len_ok = abs(np.linalg.norm(rel1) - np.linalg.norm(rel0)) < 1e-9
    moved_ok = np.linalg.norm(0.5 * (r1 + l1) - 0.5 * (r0 + l0)) > 0.05

    teleop_targets.sync_ik_mocaps_from_targets(app)
    vo_pos = teleop_targets.local_to_world_pos(
        app, app.targets["virtual_object_pos"])
    marker_err = float(np.linalg.norm(app.data.mocap_pos[app.virtual_object_mocap_id] - vo_pos))

    app.release_grasp()
    release_ok = not app.cyclo_grasp_captured and app.cyclo_capture_offsets is None
    ok = capture_ok and rel_len_ok and moved_ok and marker_err < 1e-9 and release_ok
    print(f"Cyclo bimanual virtual object gate: capture={capture_ok} "
          f"rel_len={rel_len_ok} moved={moved_ok} marker_err={marker_err*1000:.6f}mm "
          f"release={release_ok}: {'OK' if ok else 'FAIL'}")
    return ok


def run_cyclo_3d_gizmo_pose_gate():
    """3D 기즈모 행렬 왕복과 손·가상 물체 목표 pose 반영을 검사한다."""
    app = _make_sim_only_app()
    world_pos = np.array([0.42, -0.11, 0.94])
    world_quat = rotations.rpy_deg_to_quat([13.0, -8.0, 21.0])
    matrix = teleop_render.pose_to_imguizmo_matrix(world_pos, world_quat)
    round_pos, round_quat = teleop_render.imguizmo_matrix_to_pose(matrix)
    roundtrip_ok = (
        np.linalg.norm(round_pos - world_pos) < 1e-7
        and abs(abs(float(np.dot(round_quat, world_quat))) - 1.0) < 1e-7
    )

    teleop_targets.set_gizmo_target_world_pose(
        app, "r", world_pos, world_quat)
    hand_pos = teleop_targets.target_world_pose(app, "r")[0]
    hand_quat = teleop_targets.target_world_quat(app, "r")
    hand_ok = (
        np.linalg.norm(hand_pos - world_pos) < 1e-9
        and abs(abs(float(np.dot(hand_quat, world_quat))) - 1.0) < 1e-9
    )

    app.capture_grasp()
    vo_pos = np.array([0.43, 0.02, 0.98])
    vo_quat = rotations.rpy_deg_to_quat([0.0, 0.0, 16.0])
    teleop_targets.set_gizmo_target_world_pose(
        app, "virtual", vo_pos, vo_quat)
    new_vo_pos, new_vo_quat = teleop_targets.virtual_object_world_pose(app)
    virtual_ok = (
        np.linalg.norm(new_vo_pos - vo_pos) < 1e-9
        and abs(abs(float(np.dot(new_vo_quat, vo_quat))) - 1.0) < 1e-9
        and app.cyclo_grasp_captured
    )
    ok = roundtrip_ok and hand_ok and virtual_ok
    print(f"Cyclo 3D gizmo pose gate: roundtrip={roundtrip_ok} "
          f"hand_target={hand_ok} virtual_target={virtual_ok}: {'OK' if ok else 'FAIL'}")
    return ok


def run_bimanual_marker_visibility_gate():
    """가상 물체 마커가 양손 캡처 중에만 보이고 release 뒤 숨는지 검사한다."""
    app = _make_sim_only_app()
    geom_id = app.virtual_object_marker_geom_id
    site_id = app.virtual_object_marker_site_id
    geom_alpha0 = float(app.model.geom_rgba[geom_id][3])
    site_alpha0 = float(app.model.site_rgba[site_id][3])

    _set_hand_base_target(app, "r", [0.34, -0.08, 0.88])
    _set_hand_base_target(app, "l", [0.34, 0.08, 0.88])
    app.capture_grasp()
    teleop_targets.sync_ik_mocaps_from_targets(app)
    geom_alpha_capture = float(app.model.geom_rgba[geom_id][3])
    site_alpha_capture = float(app.model.site_rgba[site_id][3])

    app.release_grasp()
    teleop_targets.sync_ik_mocaps_from_targets(app)
    geom_alpha_release = float(app.model.geom_rgba[geom_id][3])
    site_alpha_release = float(app.model.site_rgba[site_id][3])

    hidden_initial = geom_alpha0 == 0.0 and site_alpha0 == 0.0
    visible_capture = (
        abs(geom_alpha_capture - app.virtual_object_marker_rgba["geom"][3]) < 1e-12
        and abs(site_alpha_capture - app.virtual_object_marker_rgba["site"][3]) < 1e-12
    )
    hidden_release = geom_alpha_release == 0.0 and site_alpha_release == 0.0
    ok = hidden_initial and visible_capture and hidden_release
    print(f"Bimanual marker visibility gate: initial_hidden={hidden_initial} "
          f"capture_visible={visible_capture} release_hidden={hidden_release}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_numeric_target_marker_sync_gate():
    """수치 XYZ/RPY 목표가 MuJoCo mocap 마커 pose에 정확히 동기화되는지 검사한다."""
    app = _make_sim_only_app()
    app.data.qpos[app.base_x_qadr] = 0.12
    app.data.qpos[app.base_y_qadr] = -0.04
    app.data.qpos[app.base_yaw_qadr] = np.radians(17.0)
    mujoco.mj_forward(app.model, app.data)

    app.targets["pos_r"] = [0.04, -0.03, 0.05]
    app.targets["rpy_r"] = [11.0, -7.0, 5.0]
    app.targets["pos_l"] = [-0.02, 0.04, 0.06]
    app.targets["rpy_l"] = [-9.0, -6.0, -4.0]

    for side, mocap_id in app.ik_target_mocap_ids.items():
        app.data.mocap_pos[mocap_id] = [9.0, 9.0, 9.0]
        app.data.mocap_quat[mocap_id] = [0.0, 1.0, 0.0, 0.0]

    teleop_targets.sync_ik_mocaps_from_targets(app)

    ok = True
    reports = []
    for side, mocap_id in app.ik_target_mocap_ids.items():
        expected_pos = teleop_targets.target_world_pose(app, side)[0]
        expected_quat = teleop_targets.target_world_quat(app, side)
        pos_err = float(np.linalg.norm(app.data.mocap_pos[mocap_id] - expected_pos))
        quat_dot = abs(float(np.dot(app.data.mocap_quat[mocap_id], expected_quat)))
        case_ok = pos_err < 1e-9 and (1.0 - quat_dot) < 1e-9
        ok = ok and case_ok
        reports.append(f"{side}: pos_err={pos_err*1000:.6f}mm quat_dot={quat_dot:.12f}")

    print(f"Numeric target -> marker sync gate: {'; '.join(reports)}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_whole_body_toggle_gate():
    """모드 전환이 월드 목표를 보존하고 오래된 차체 명령을 지우는지 검사한다."""
    app = _make_sim_only_app()
    app.targets["pos_r"] = [0.025, -0.018, 0.012]
    app.targets["rpy_r"] = [7.0, -4.0, 5.0]
    app.targets["pos_l"] = [-0.020, 0.014, 0.018]
    app.targets["rpy_l"] = [-6.0, 3.0, -4.0]
    app.targets["virtual_object_pos"] = [0.31, -0.02, 0.87]
    app.targets["virtual_object_rpy"] = [2.0, -3.0, 8.0]
    hand_before = {
        side: teleop_targets.target_world_pose(app, side) for side in ("r", "l")}
    virtual_before = teleop_targets.virtual_object_world_pose(app)
    app.whole_body_base_twist = base.BodyTwist(0.2, -0.1, 0.3)
    app.commanded_base_twist = base.BodyTwist(0.2, -0.1, 0.3)

    app.toggle_whole_body_control()
    off_mode = not app.whole_body_enabled
    hand_off = {
        side: teleop_targets.target_world_pose(app, side) for side in ("r", "l")}
    virtual_off = teleop_targets.virtual_object_world_pose(app)
    off_pose_preserved = all(
        np.allclose(hand_before[side][0], hand_off[side][0], atol=1e-10)
        and abs(np.dot(hand_before[side][1], hand_off[side][1])) > 1.0 - 1e-10
        for side in ("r", "l"))
    off_pose_preserved &= (
        np.allclose(virtual_before[0], virtual_off[0], atol=1e-10)
        and abs(np.dot(virtual_before[1], virtual_off[1])) > 1.0 - 1e-10)
    stale_command_cleared = (
        app.commanded_base_twist == base.BodyTwist()
        and app.whole_body_base_twist == base.BodyTwist())
    smoothed_synced = all(
        np.allclose(app.smoothed_pos[side], app.targets[f"pos_{side}"])
        and np.allclose(app.smoothed_rpy[side], app.targets[f"rpy_{side}"])
        for side in ("r", "l"))

    app.toggle_whole_body_control()
    hand_on = {
        side: teleop_targets.target_world_pose(app, side) for side in ("r", "l")}
    virtual_on = teleop_targets.virtual_object_world_pose(app)
    round_trip = app.whole_body_enabled and all(
        np.allclose(hand_before[side][0], hand_on[side][0], atol=1e-10)
        and abs(np.dot(hand_before[side][1], hand_on[side][1])) > 1.0 - 1e-10
        for side in ("r", "l"))
    round_trip &= (
        np.allclose(virtual_before[0], virtual_on[0], atol=1e-10)
        and abs(np.dot(virtual_before[1], virtual_on[1])) > 1.0 - 1e-10)

    captured_app = _make_sim_only_app()
    captured_app.capture_grasp()
    captured_app.targets["virtual_object_pos"][0] += 0.035
    captured_app.targets["virtual_object_rpy"][2] = 6.0
    captured_app.apply_virtual_object_target()
    captured_before = {
        side: teleop_targets.target_world_pose(captured_app, side)
        for side in ("r", "l")}
    captured_virtual_before = teleop_targets.virtual_object_world_pose(captured_app)
    captured_app.toggle_whole_body_control()
    captured_app.toggle_whole_body_control()
    captured_round_trip = all(
        np.allclose(
            captured_before[side][0],
            teleop_targets.target_world_pose(captured_app, side)[0],
                    atol=1e-10)
        and abs(np.dot(captured_before[side][1],
                       teleop_targets.target_world_pose(
                           captured_app, side)[1])) > 1.0 - 1e-10
        for side in ("r", "l"))
    captured_virtual_after = teleop_targets.virtual_object_world_pose(captured_app)
    captured_round_trip &= (
        np.allclose(captured_virtual_before[0], captured_virtual_after[0], atol=1e-10)
        and abs(np.dot(captured_virtual_before[1], captured_virtual_after[1])) > 1.0 - 1e-10)

    integration_app = _make_sim_only_app()
    integration_app.q_des = {
        "r": teleop_app.HOME_Q_R.copy(),
        "l": teleop_app.HOME_Q_L.copy(),
    }
    integration_app.arm_mode = {"r": "ik", "l": "ik"}
    integration_app.fk_q_deg = {
        side: np.degrees(q_des).tolist()
        for side, q_des in integration_app.q_des.items()
    }
    integration_app.frame_dt = 1.0 / teleop_app.LOOP_HZ
    integration_app.steps_per_frame = max(
        1, round(integration_app.frame_dt / integration_app.model.opt.timestep))
    integration_app.ik_err_mm = {"r": 0.0, "l": 0.0}
    integration_app.toggle_whole_body_control()
    integration_app.targets["lift"] += 0.02
    integration_app._step_physics(
        {key: False for key in ("w", "a", "s", "d", "left", "right")})
    off_integration = (
        integration_app.lift_cmd == integration_app.targets["lift"]
        and integration_app.commanded_base_twist == base.BodyTwist())

    ok = (off_mode and off_pose_preserved and stale_command_cleared and smoothed_synced
          and round_trip and captured_round_trip and off_integration)
    print(f"Whole-body toggle gate: off={off_mode} off_pose={off_pose_preserved} "
          f"stale_zero={stale_command_cleared} smoothing={smoothed_synced} "
          f"round_trip={round_trip} captured={captured_round_trip} "
          f"integration={off_integration}: {'OK' if ok else 'FAIL'}")
    return ok


def run_collision_visualization_gate():
    """GL 창 없이도 V 토글이 활성 CBF 점을 제공하는지 검사한다."""
    app = _make_sim_only_app()
    initially_off = (
        not app.collision_viz
        and teleop_render.collision_visualization_data(app) == ())
    app.toggle_collision_visualization()

    lift_index = app.whole_body_solver.index["lift_joint"]
    lift_qadr = app.whole_body_solver.qpos_adrs[lift_index]
    app.data.qpos[lift_qadr] -= 0.035
    mujoco.mj_forward(app.model, app.data)
    constraints = teleop_render.collision_visualization_data(app)
    data_ok = (
        len(constraints) >= 2
        and all(constraint.distance <= app.whole_body_solver.collision_buffer + 1e-12
                for constraint in constraints)
        and all(np.isfinite(constraint.point_a).all()
                and np.isfinite(constraint.point_b).all()
                for constraint in constraints)
    )

    app.scene = mujoco.MjvScene(app.model, maxgeom=100)
    teleop_render._append_collision_overlay(app, constraints)
    overlay_ok = (
        app.scene.ngeom == 3 * len(constraints)
        and sum(int(app.scene.geoms[i].type) == int(mujoco.mjtGeom.mjGEOM_LINE)
                for i in range(app.scene.ngeom)) == len(constraints)
        and sum(int(app.scene.geoms[i].type) == int(mujoco.mjtGeom.mjGEOM_SPHERE)
                for i in range(app.scene.ngeom)) == 2 * len(constraints)
    )
    safe_distance = app.whole_body_solver.collision_safe_distance
    buffer_distance = app.whole_body_solver.collision_buffer
    colors_distinct = (
        not np.array_equal(teleop_render._collision_color(-0.001, safe_distance),
                           teleop_render._collision_color(0.5 * safe_distance, safe_distance))
        and not np.array_equal(
            teleop_render._collision_color(0.5 * safe_distance, safe_distance),
            teleop_render._collision_color(
                0.5 * (safe_distance + buffer_distance), safe_distance))
    )
    app.toggle_collision_visualization()
    toggles_off = not app.collision_viz and not teleop_render.collision_visualization_data(app)
    ok = initially_off and data_ok and overlay_ok and colors_distinct and toggles_off
    print(f"Collision visualization gate: initial_off={initially_off} "
          f"active={len(constraints)} overlay_geoms={app.scene.ngeom} "
          f"colors={colors_distinct} toggles_off={toggles_off}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_manual_xyz_rpy_ik_gate():
    """수동 XYZ/RPY 목표에서 arm-only differential IK 수렴을 검사한다."""
    cases = (
        ("r", np.array([-0.035, -0.015, 0.025]), np.array([8.0, -4.0, 6.0])),
        ("l", np.array([-0.035, 0.015, 0.025]), np.array([-8.0, -4.0, -6.0])),
    )
    ok = True
    reports = []
    for side, pos_delta, rpy_delta in cases:
        app = _make_sim_only_app()
        solver, data = app.whole_body_solver, app.data
        initial = solver.site_state(data, side)
        target_pos = initial.position + pos_delta
        target_quat = np.zeros(4)
        mujoco.mju_mulQuat(
            target_quat, initial.quaternion, rotations.rpy_deg_to_quat(rpy_delta)
        )
        for _ in range(80):
            command = solver.solve(
                data, {side: (target_pos, target_quat)}, 0.04,
                active_sides=(side,), whole_body_enabled=False)
            data.qpos[solver.qpos_adrs[solver.side_indices[side]]] = (
                command.arm_positions[side])
            mujoco.mj_forward(app.model, data)
        final = solver.site_state(data, side)
        error = pose_tasks.pose_error(
            final.position, final.quaternion, target_pos, target_quat)
        pos_err, ori_err = error.position_norm, error.orientation_norm
        reached_delta = final.position - initial.position
        pos_delta_ok = np.linalg.norm(reached_delta - pos_delta) < 0.006
        case_ok = pos_delta_ok and pos_err < 0.005 and ori_err < np.radians(5.0)
        ok = ok and case_ok
        reports.append(
            f"{side}: pos_err={pos_err*1000:.2f}mm "
            f"ori_err={np.degrees(ori_err):.2f}deg delta_ok={pos_delta_ok}")
    print(f"Manual XYZ/RPY IK gate: {'; '.join(reports)}: {'OK' if ok else 'FAIL'}")
    return ok


def run_task_space_input_gate():
    """world XYZ/RPY 입력이 UI 변환·평활화·IK를 거쳐 손을 움직이는지 검사한다."""
    app = _make_sim_only_app()
    app.arm_mode = {"r": "ik", "l": "ik"}
    side = "r"
    initial = app.whole_body_solver.site_state(app.data, side)
    desired_position = initial.position + np.array([-0.035, -0.018, 0.028])
    desired_quaternion = rotations.multiply_quaternions(
        initial.quaternion, rotations.rpy_deg_to_quat([7.0, -4.0, 6.0]))
    desired_rpy = rotations.quat_to_rpy_deg(desired_quaternion)

    teleop_ui._ensure_task_space_state(app)
    applied, message = teleop_ui._apply_task_space_target(
        app, side, desired_position, desired_rpy)
    converted_position, converted_quaternion = teleop_targets.target_world_pose(
        app, side)
    conversion_ok = (
        applied
        and "tracking" in message
        and np.allclose(converted_position, desired_position, atol=1e-10)
        and abs(float(np.dot(converted_quaternion, desired_quaternion)))
        > 1.0 - 1e-10
    )

    solver = app.whole_body_solver
    for _ in range(80):
        app._smooth_hand_targets()
        target_pose = app._smoothed_target_poses()[side]
        command = solver.solve(
            app.data, {side: target_pose}, 0.04,
            active_sides=(side,), whole_body_enabled=False)
        app.data.qpos[solver.qpos_adrs[solver.side_indices[side]]] = (
            command.arm_positions[side])
        mujoco.mj_forward(app.model, app.data)
    final = solver.site_state(app.data, side)
    error = pose_tasks.pose_error(
        final.position, final.quaternion, desired_position, desired_quaternion)
    moved = np.linalg.norm(final.position - initial.position) > 0.02
    converged = error.position_norm < 0.005 and error.orientation_norm < np.radians(5.0)

    previous_target = np.asarray(app.targets["pos_r"], dtype=float).copy()
    finite_ok, _ = teleop_ui._apply_task_space_target(
        app, side, [np.nan, 0.0, 0.0], [0.0, 0.0, 0.0])
    side_ok, _ = teleop_ui._apply_task_space_target(
        app, "invalid", initial.position, desired_rpy)
    rejects_invalid = (
        not finite_ok and not side_ok
        and np.allclose(app.targets["pos_r"], previous_target))

    ok = conversion_ok and moved and converged and rejects_invalid
    print(
        "Task-space input gate: "
        f"conversion={conversion_ok} moved={moved} "
        f"pos_err={error.position_norm*1000:.2f}mm "
        f"ori_err={np.degrees(error.orientation_norm):.2f}deg "
        f"rejects_invalid={rejects_invalid}: {'OK' if ok else 'FAIL'}")
    return ok


def run_split_ui_and_tree_gate():
    """탭 작업 공간이 간결하고 트리 필터가 손의 체인을 따르는지 검사한다."""
    app = _make_sim_only_app()
    windows = teleop_ui._ensure_window_state(app)
    titles = [spec["title"] for spec in teleop_ui.UI_WINDOW_SPECS.values()]
    windows_ok = (
        set(windows) == {"control", "diagnostics"}
        and len(titles) == len(set(titles))
        and windows["control"]
        and windows["diagnostics"]
    )

    tree = app.whole_body_solver.kinematic_tree
    right = teleop_ui.kinematic_tree_body_ids(app, "r", False)
    left = teleop_ui.kinematic_tree_body_ids(app, "l", False)
    both = teleop_ui.kinematic_tree_body_ids(app, "both", False)
    full = teleop_ui.kinematic_tree_body_ids(app, "both", True)
    right_site = app.whole_body_solver.site_ids["r"]
    left_site = app.whole_body_solver.site_ids["l"]
    tree_ok = (
        both == right | left
        and right != left
        and tree.sites[right_site].body_id in right
        and tree.sites[left_site].body_id in left
        and len(full) == len(tree.bodies)
        and all(tree.bodies[child].parent_id == parent
                for parent, children in enumerate(tree.children_by_body)
                for child in children)
        and all(joint.kind_name in {"free", "ball", "slide", "hinge"}
                for joint in tree.joints)
    )
    ok = windows_ok and tree_ok
    print(f"Tabbed UI/tree gate: workspaces={len(windows)} compact={windows_ok} "
          f"chain_bodies=R{len(right)}/L{len(left)}/both{len(both)} "
          f"full={len(full)}: {'OK' if ok else 'FAIL'}")
    return ok


def main():
    """마커·기즈모·양손 제어·모드 전환·UI 트리 Phase 6 gate를 실행한다."""
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    ok = (run_model_gate(model)
          and run_split_ui_and_tree_gate()
          and run_initial_ik_target_origin_gate()
          and run_cyclo_marker_jog_gate()
          and run_cyclo_bimanual_virtual_object_gate()
          and run_cyclo_3d_gizmo_pose_gate()
          and run_bimanual_marker_visibility_gate()
          and run_numeric_target_marker_sync_gate()
          and run_whole_body_toggle_gate()
          and run_collision_visualization_gate()
          and run_task_space_input_gate()
          and run_manual_xyz_rpy_ik_gate())
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
