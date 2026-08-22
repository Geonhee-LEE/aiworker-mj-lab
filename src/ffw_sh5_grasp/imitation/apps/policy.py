"""Interactive MuJoCo UI for running a trained arm-only ACT policy."""

import time

import glfw
import numpy as np
from imgui_bundle import imgui

from ...visualization import render
from ..runtime.runner import ACTPolicyRunner
from ..simulation.environment import AIWorkerMujocoEnv
from .base import KeyEdge, render_operator_frame


class ACTPolicyApp:
    """Run, pause, step and reset a checkpoint in its training environment."""

    def __init__(self, checkpoint, *, stats_path=None, device="auto",
                 seed=1000, max_steps=500, width=1440, height=900):
        if max_steps <= 0:
            raise ValueError("max_steps must be positive")
        self.runner = ACTPolicyRunner(checkpoint, stats_path, device=device)
        if self.runner.representation != "joint":
            raise ValueError(
                "task-space checkpoints require the teleop_app arm-only IK bridge")
        self.env = AIWorkerMujocoEnv(
            camera_names=self.runner.camera_names, render_images=True, seed=seed)
        self.model, self.data = self.env.model, self.env.data
        self.observation = self.env.reset(seed=seed)
        self.keys = KeyEdge()
        self.running = False
        self.max_steps = int(max_steps)
        self.frame = 0
        self.stop_reason = "ready"
        self.last_action = self.env.last_action.copy()
        self.last_policy_info = None
        self.frame_dt = 1.0 / self.env.actual_control_hz
        self.freq_ema = self.env.actual_control_hz
        self.gizmo_mouse_active = False
        self.last_mouse = [0.0, 0.0]
        render.setup_render(self, width, height)
        self.last_mouse = list(glfw.get_cursor_pos(self.window))

    def reset(self):
        self.observation = self.env.reset()
        self.runner.reset()
        self.frame = 0
        self.last_action = self.env.last_action.copy()
        self.last_policy_info = None
        self.running = False
        self.stop_reason = "reset"

    def start_policy(self):
        """Start or resume a rollout, resetting a completed rollout first."""
        if self.frame >= self.max_steps:
            self.reset()
        self.running = True
        self.stop_reason = None

    def pause_policy(self):
        self.running = False
        self.stop_reason = "paused"

    def step_policy(self):
        if self.frame >= self.max_steps:
            self.running = False
            self.stop_reason = "max steps reached"
            return False
        action, info = self.runner.get_action(self.observation)
        self.last_action = self.env.prepare_action(action)
        self.last_policy_info = info
        self.observation = self.env.step(self.last_action)
        self.frame += 1
        if self.frame >= self.max_steps:
            self.running = False
            self.stop_reason = "max steps reached"
        return True

    def _handle_keys(self, io):
        if io.want_capture_keyboard:
            return
        if self.keys.pressed(self.window, glfw.KEY_ESCAPE):
            glfw.set_window_should_close(self.window, True)
        if self.keys.pressed(self.window, glfw.KEY_R):
            self.reset()
        if self.keys.pressed(self.window, glfw.KEY_SPACE):
            if self.running:
                self.pause_policy()
            else:
                self.start_policy()
        if self.keys.pressed(self.window, glfw.KEY_N):
            self.step_policy()

    def _draw_panel(self):
        imgui.set_next_window_pos((10, 10), imgui.Cond_.first_use_ever)
        imgui.set_next_window_size((385, 485), imgui.Cond_.first_use_ever)
        imgui.begin("ACT Policy Teleop")
        imgui.text(f"Checkpoint policy: {self.runner.config.action_dim}D action")
        imgui.text(f"Device: {self.runner.device}")
        imgui.text(f"Policy frame: {self.frame} / {self.max_steps}")
        imgui.text(f"Control Hz: {self.env.actual_control_hz:.1f}")
        imgui.separator()
        changed, max_steps = imgui.input_int(
            "Max steps", self.max_steps, 10, 100)
        if changed:
            self.max_steps = max(1, max_steps)
            if self.frame >= self.max_steps:
                self.running = False
                self.stop_reason = "max steps reached"
        imgui.text(f"Policy: {'RUNNING' if self.running else 'PAUSED'}")
        if self.stop_reason is not None:
            imgui.text(f"Status: {self.stop_reason}")
        if imgui.button("Run policy" if not self.running else "Pause"):
            if self.running:
                self.pause_policy()
            else:
                self.start_policy()
        imgui.same_line()
        if imgui.button("Step [N]"):
            self.step_policy()
        imgui.same_line()
        if imgui.button("Reset [R]"):
            self.reset()
        imgui.separator()
        imgui.text("Head: fixed for cam_high")
        imgui.text(
            "Head joints: "
            f"{np.degrees(self.env.head_fixed_position[0]):.1f}, "
            f"{np.degrees(self.env.head_fixed_position[1]):.1f} deg")
        imgui.text("Left arm: locked outside workspace")
        imgui.text(
            "Can settled in box: "
            f"{self.observation['task']['success']}")
        imgui.text(
            f"Object error: {self.observation['task']['object_position_error']:.4f} m")
        if self.last_policy_info is not None:
            pad = self.last_policy_info["predicted_pad"]
            if pad is not None:
                imgui.text(f"Predicted chunk: {len(pad)} actions")
        imgui.separator()
        imgui.text("Policy observations")
        for name in self.runner.camera_names:
            image = self.observation["images"].get(name)
            valid = image is not None and image.dtype == np.uint8 and image.size > 0
            imgui.text(f"{name}: {'OK' if valid else 'MISSING'}")
        imgui.text("SPACE: run/pause | N: one action | R: reset | ESC: exit")
        imgui.end()

    def _render(self):
        render_operator_frame(self)

    def run(self):
        try:
            while not glfw.window_should_close(self.window):
                started = time.perf_counter()
                io = render.begin_frame(self)
                render.handle_camera_mouse(self, io)
                self._handle_keys(io)
                if self.running:
                    self.step_policy()
                self._draw_panel()
                self._render()
                render.end_frame(self, started)
        finally:
            self.env.close()
            glfw.make_context_current(self.window)
            render.shutdown(self)


__all__ = ["ACTPolicyApp"]
