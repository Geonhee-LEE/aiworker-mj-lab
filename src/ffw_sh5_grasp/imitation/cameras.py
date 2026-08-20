"""Named RGB camera rendering for ALOHA-compatible observations."""

import mujoco
import numpy as np


DEFAULT_CAMERA_NAMES = ("cam_high", "cam_left_wrist", "cam_right_wrist")


class MujocoCameraManager:
    """Render all configured fixed MJCF cameras into RGB uint8 arrays."""

    def __init__(self, model, data, *, width=320, height=240,
                 camera_names=DEFAULT_CAMERA_NAMES):
        self.model = model
        self.data = data
        self.width = int(width)
        self.height = int(height)
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera width and height must be positive")
        self.camera_names = tuple(camera_names)
        if not self.camera_names or len(set(self.camera_names)) != len(self.camera_names):
            raise ValueError("camera_names must be non-empty and unique")
        for name in self.camera_names:
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) < 0:
                raise ValueError(f"MuJoCo camera not found: {name}")
        self._renderer = None

    def _get_renderer(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.model, height=self.height, width=self.width)
        return self._renderer

    def render(self):
        renderer = self._get_renderer()
        images = {}
        for name in self.camera_names:
            renderer.update_scene(self.data, camera=name)
            image = np.asarray(renderer.render())
            if image.shape != (self.height, self.width, 3) or image.dtype != np.uint8:
                raise RuntimeError(
                    f"camera {name} returned {image.shape}/{image.dtype}, expected "
                    f"({self.height}, {self.width}, 3)/uint8")
            images[name] = image.copy()
        return images

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        self.close()


__all__ = ["DEFAULT_CAMERA_NAMES", "MujocoCameraManager"]
