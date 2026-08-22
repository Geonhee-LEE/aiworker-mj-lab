"""Interactive arm-only Gizmo demonstration recorder."""

import time

import glfw
import mujoco
import numpy as np
from imgui_bundle import imgui, imguizmo

from ...visualization import render
from ..data.recording import EpisodeRecorder
from ..simulation.environment import AIWorkerMujocoEnv
from ..visualization.rerun_live import LiveRecordingRerunLogger
from .base import KeyEdge, render_operator_frame
from .leader import GizmoLeader


class RecordEpisodesApp:
    """GLFW/ImGui app whose only motion source is an arm-only GizmoLeader."""

    def __init__(self, dataset_dir, *, task_name="can_to_box", seed=None,
                 width=1440, height=900, live_rerun=True, rerun_port=9876,
                 object_variants=None):
        self.object_variants = (
            None if object_variants is None else tuple(object_variants))
        self.env = AIWorkerMujocoEnv(
            seed=seed, task_name=task_name,
            object_variants=self.object_variants,
            randomize_bin_colors=task_name == "can_color_sort")
        self.model, self.data = self.env.model, self.env.data
        self.leader = GizmoLeader(self.env)
        self.recorder = EpisodeRecorder(dataset_dir, self.env, task_name=task_name)
        self.observation = self.env.get_observation()
        self.selected_side = "r"
        self.keys = KeyEdge()
        self.frame_dt = 1.0 / self.env.actual_control_hz
        self.freq_ema = self.env.actual_control_hz
        self.contact_viz = False
        self.collision_viz = False
        self.gizmo_mouse_active = False
        self.last_mouse = [0.0, 0.0]
        render.setup_render(self, width, height)
        self.last_mouse = list(glfw.get_cursor_pos(self.window))
        self.live_rerun = LiveRecordingRerunLogger(
            self.env.camera_names, enabled=live_rerun, port=rerun_port)
        self.live_rerun.start()

    def reset(self):
        """R semantics: discard partial data, home robot, randomize the can."""
        if self.recorder.recording:
            self.recorder.discard()
        self.observation = self.env.reset()
        self.leader.reset()

    def toggle_recording(self):
        if self.recorder.recording:
            if self.recorder.frame:
                self.recorder.finish()
            else:
                self.recorder.discard()
        else:
            self.recorder.start()

    def _handle_keys(self, io):
        if io.want_capture_keyboard:
            return
        if self.keys.pressed(self.window, glfw.KEY_ESCAPE):
            glfw.set_window_should_close(self.window, True)
        if self.keys.pressed(self.window, glfw.KEY_R):
            self.reset()
        if self.keys.pressed(self.window, glfw.KEY_SPACE):
            self.toggle_recording()
        if self.keys.pressed(self.window, glfw.KEY_BACKSPACE):
            self.recorder.discard()
        if (not self.env.left_arm_fixed
                and self.keys.pressed(self.window, glfw.KEY_TAB)):
            self.selected_side = "l" if self.selected_side == "r" else "r"
        if self.keys.pressed(self.window, glfw.KEY_Q):
            self.leader.toggle_grasp(self.selected_side)
        if self.keys.pressed(self.window, glfw.KEY_E):
            self.leader.return_home(self.selected_side)

    def _draw_panel(self):
        imgui.set_next_window_pos((10, 10), imgui.Cond_.first_use_ever)
        imgui.set_next_window_size((355, 465), imgui.Cond_.first_use_ever)
        imgui.begin("ALOHA Dataset Recorder")
        imgui.text(f"Task: {self.env.task.description}")
        if self.object_variants is not None:
            imgui.text(
                "Collection filter: "
                + ", ".join(name.upper() for name in self.object_variants))
        imgui.text(
            f"Object: {self.env.task.object_variant.upper()} can -> "
            f"{self.env.task.target_label.upper()} box")
        imgui.text("Control: arm-only (whole-body disabled)")
        imgui.separator()
        imgui.text(f"Recording: {'YES' if self.recorder.recording else 'NO'}")
        imgui.text(f"Frame: {self.recorder.frame}")
        imgui.text(f"Control Hz: {self.env.actual_control_hz:.1f}")
        imgui.text(
            f"IK max: {self.leader.linear_speed:.1f} m/s, "
            f"{self.leader.angular_speed:.1f} rad/s")
        imgui.text(f"Dropped: {self.recorder.dropped}")
        imgui.text(f"Rerun live: {'ON' if self.live_rerun.enabled else 'OFF'}")
        imgui.text(f"Selected hand: {'LEFT' if self.selected_side == 'l' else 'RIGHT'}")
        if self.env.left_arm_fixed:
            imgui.text("Left hand: LOCKED (palm up)")
        grasp_side = self.selected_side
        grasp_state = "CLOSED" if self.leader.grasp[grasp_side] >= 0.5 else "OPEN"
        imgui.text(f"{grasp_side.upper()} grasp target: {grasp_state}")
        if self.leader.returning_home[self.selected_side]:
            imgui.text(f"{self.selected_side.upper()} arm: HOME RETURN/HOLD")
        imgui.text(f"Task success: {self.observation['task']['success']}")
        if imgui.button("Reset robot + random can [R]"):
            self.reset()
        if imgui.button("Start / Finish [SPACE]"):
            self.toggle_recording()
        if imgui.button("Discard [BACKSPACE]"):
            self.recorder.discard()
        if imgui.button("Grab / Release [Q]"):
            self.leader.toggle_grasp(self.selected_side)
        if imgui.button("Return arm home [E]"):
            self.leader.return_home(self.selected_side)
        imgui.separator()
        if self.env.left_arm_fixed:
            imgui.text("Q: grab/release | E: return right arm home")
        else:
            imgui.text("TAB: select hand | Q: grab/release | E: arm home")
        imgui.text("Drag Gizmo arrows/rings to move selected hand")
        for name in self.env.camera_names:
            image = self.observation["images"].get(name)
            ok = image is not None and image.dtype == np.uint8 and image.size > 0
            imgui.text(f"{name}: {'OK' if ok else 'MISSING'}")
        imgui.end()

    def _sync_markers(self):
        for side in ("l", "r"):
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"ik_target_{side}")
            mocap_id = int(self.model.body_mocapid[body_id])
            position, quaternion = self.leader.targets[side]
            self.data.mocap_pos[mocap_id] = position
            self.data.mocap_quat[mocap_id] = quaternion

    def _draw_gizmo(self, viewport):
        position, quaternion = self.leader.targets[self.selected_side]
        matrix = render.pose_to_imguizmo_matrix(position, quaternion)
        view, projection = render._imguizmo_camera_matrices(self, viewport)
        main_viewport = imgui.get_main_viewport()
        gizmo = imguizmo.im_guizmo
        gizmo.begin_frame()
        gizmo.set_drawlist(imgui.get_foreground_draw_list(main_viewport))
        gizmo.set_rect(
            float(main_viewport.pos.x), float(main_viewport.pos.y),
            float(main_viewport.size.x), float(main_viewport.size.y))
        gizmo.set_orthographic(False)
        # pyimgui-bundle의 OPERATION은 IntFlag처럼 보이지만 OR 결과는 Python int가
        # 되어 manipulate()의 enum 인자 검사를 통과하지 못한다. 기존 teleop Gizmo와
        # 같이 각 operation을 enum 값 그대로 따로 호출한다.
        changed_translate = gizmo.manipulate(
            view, projection, gizmo.OPERATION.translate,
            gizmo.MODE.world, matrix)
        changed_rotate = gizmo.manipulate(
            view, projection, gizmo.OPERATION.rotate,
            gizmo.MODE.local, matrix)
        changed = changed_translate or changed_rotate
        self.gizmo_mouse_active = bool(gizmo.is_using_any() or gizmo.is_over())
        if changed:
            new_position, new_quaternion = render.imguizmo_matrix_to_pose(matrix)
            self.leader.set_target_pose(
                self.selected_side, new_position, new_quaternion)

    def _render(self):
        render_operator_frame(
            self,
            before_scene=self._sync_markers,
            after_scene=self._draw_gizmo,
        )

    def run(self):
        try:
            while not glfw.window_should_close(self.window):
                started = time.perf_counter()
                io = render.begin_frame(self)
                render.handle_camera_mouse(self, io)
                self._handle_keys(io)
                self._draw_panel()
                action = self.leader.get_action()
                episode_frame = self.recorder.frame
                self.recorder.record(self.observation, action)
                self.live_rerun.log(
                    self.observation,
                    action,
                    recording=self.recorder.recording,
                    episode_frame=episode_frame,
                )
                self.observation = self.env.step(action)
                self._render()
                render.end_frame(self, started)
        finally:
            if self.recorder.recording:
                self.recorder.discard()
            self.live_rerun.close()
            self.env.close()
            glfw.make_context_current(self.window)
            render.shutdown(self)


__all__ = ["RecordEpisodesApp"]
