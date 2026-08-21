"""Shared GLFW/ImGui mechanics for imitation-learning applications."""

import glfw
from imgui_bundle import imgui
import mujoco

from ...visualization import render


class KeyEdge:
    """Report a key only on its up-to-down transition."""

    def __init__(self):
        self._down = set()

    def pressed(self, window, key):
        """Return ``True`` once when ``key`` is newly pressed."""
        current = glfw.get_key(window, key) == glfw.PRESS
        previous = key in self._down
        if current:
            self._down.add(key)
        else:
            self._down.discard(key)
        return current and not previous


def render_operator_frame(app, *, before_scene=None, after_scene=None):
    """Render one MuJoCo scene and the current ImGui frame.

    ``before_scene`` updates model-side markers before MuJoCo builds the scene.
    ``after_scene`` receives the viewport and can draw an ImGuizmo overlay.
    """
    render.restore_window_render_target(app)
    if before_scene is not None:
        before_scene()
    framebuffer_w, framebuffer_h = glfw.get_framebuffer_size(app.window)
    viewport = mujoco.MjrRect(0, 0, framebuffer_w, framebuffer_h)
    mujoco.mjv_updateScene(
        app.model,
        app.data,
        app.opt,
        app.pert,
        app.cam,
        mujoco.mjtCatBit.mjCAT_ALL,
        app.scene,
    )
    mujoco.mjr_render(viewport, app.scene, app.context)
    if after_scene is not None:
        after_scene(viewport)

    imgui.render()
    imgui.backends.opengl3_render_draw_data(imgui.get_draw_data())
    if imgui.get_io().config_flags & imgui.ConfigFlags_.viewports_enable:
        main_context = glfw.get_current_context()
        imgui.update_platform_windows()
        imgui.render_platform_windows_default()
        glfw.make_context_current(main_context)
    glfw.swap_buffers(app.window)


__all__ = ["KeyEdge", "render_operator_frame"]
