"""Named RGB camera rendering for ALOHA-compatible observations."""

import mujoco
import numpy as np

DEFAULT_CAMERA_NAMES = ("cam_high", "cam_left_wrist", "cam_right_wrist")
POLICY_SELF_OCCLUDER_GROUP = 4
OPERATOR_MARKER_GROUP = 5


class MujocoCameraManager:
    """Render all configured fixed MJCF cameras into RGB uint8 arrays."""

    def __init__(
        self,
        model,
        data,
        *,
        width=320,
        height=240,
        camera_names=DEFAULT_CAMERA_NAMES,
        render_context=None,
        make_context_current=None,
    ):
        self.model = model
        self.data = data
        self.width = int(width)
        self.height = int(height)
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera width and height must be positive")
        self.camera_names = tuple(camera_names)
        if not self.camera_names or len(set(self.camera_names)) != len(
            self.camera_names
        ):
            raise ValueError("camera_names must be non-empty and unique")
        for name in self.camera_names:
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) < 0:
                raise ValueError(f"MuJoCo camera not found: {name}")
        self._scene_option = mujoco.MjvOption()
        # The imported head CAD encloses the calibrated ZED-M optical origin.
        # Keep its housing visible in the operator GUI but do not let the
        # simulated sensor render the opaque shell around itself.
        self._scene_option.geomgroup[POLICY_SELF_OCCLUDER_GROUP] = 0
        # IK targets and diagnostic sites belong to the operator UI, not to
        # observations recorded in HDF5 or streamed to Rerun.
        self._scene_option.geomgroup[OPERATOR_MARKER_GROUP] = 0
        self._scene_option.sitegroup[OPERATOR_MARKER_GROUP] = 0
        self._shared_context = render_context
        self._make_context_current = make_context_current
        if (render_context is None) != (make_context_current is None):
            raise ValueError(
                "render_context and make_context_current must be provided together"
            )
        self._shared_scene = (
            mujoco.MjvScene(model, maxgeom=10000)
            if render_context is not None
            else None
        )
        self._renderer = None

    def _get_renderer(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.model, height=self.height, width=self.width
            )
        return self._renderer

    def render(self):
        if self._shared_context is not None:
            return self._render_shared_context()
        renderer = self._get_renderer()
        images = {}
        for name in self.camera_names:
            renderer.update_scene(
                self.data, camera=name, scene_option=self._scene_option
            )
            image = np.asarray(renderer.render())
            if image.shape != (self.height, self.width, 3) or image.dtype != np.uint8:
                raise RuntimeError(
                    f"camera {name} returned {image.shape}/{image.dtype}, expected "
                    f"({self.height}, {self.width}, 3)/uint8"
                )
            images[name] = image.copy()
        return images

    def _render_shared_context(self):
        """Render policy cameras without creating a second GLFW context.

        Embedded teleop inference shares the visible window's ``MjrContext``.
        Only its framebuffer is switched temporarily; it is always restored to
        the window target before returning, including when rendering fails.
        """
        self._make_context_current()
        context = self._shared_context
        viewport = mujoco.MjrRect(0, 0, self.width, self.height)
        images = {}
        try:
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)
            for name in self.camera_names:
                camera_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_CAMERA, name
                )
                camera = mujoco.MjvCamera()
                camera.fixedcamid = camera_id
                camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
                mujoco.mjv_updateScene(
                    self.model,
                    self.data,
                    self._scene_option,
                    None,
                    camera,
                    mujoco.mjtCatBit.mjCAT_ALL,
                    self._shared_scene,
                )
                image = np.empty((self.height, self.width, 3), dtype=np.uint8)
                mujoco.mjr_render(viewport, self._shared_scene, context)
                mujoco.mjr_readPixels(image, None, viewport, context)
                # OpenGL framebuffer rows are bottom-to-top, while policy
                # tensors and ``mujoco.Renderer`` use top-to-bottom images.
                images[name] = np.flipud(image).copy()
        finally:
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, context)
        return images

    def close(self):
        # A shared context is owned by the teleop renderer and must not be
        # destroyed when policy control ends.
        self._shared_scene = None
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        self.close()


__all__ = [
    "DEFAULT_CAMERA_NAMES",
    "MujocoCameraManager",
    "OPERATOR_MARKER_GROUP",
    "POLICY_SELF_OCCLUDER_GROUP",
]
