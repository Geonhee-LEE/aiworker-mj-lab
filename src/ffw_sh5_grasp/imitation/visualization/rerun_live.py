"""Live Rerun stream for interactive demonstration recording."""

from ..data.schema import ACTION_NAMES
from .rerun_blueprints import live_recording_blueprint


class LiveRecordingRerunLogger:
    """Stream synchronized recorder frames to a detached Rerun Viewer."""

    def __init__(self, camera_names, *, enabled=True, port=9876,
                 application_id="aiworker_live_recording"):
        self.camera_names = tuple(camera_names)
        self.enabled = bool(enabled)
        self.port = int(port)
        self.application_id = str(application_id)
        self.recording = None
        self.frame = 0

    def start(self):
        if not self.enabled or self.recording is not None:
            return
        try:
            import rerun as rr
        except ImportError as error:
            raise RuntimeError(
                "Live episode visualization requires: pip install rerun-sdk"
            ) from error
        self.rr = rr
        self.recording = rr.RecordingStream(self.application_id)
        self.recording.__enter__()
        try:
            self.recording.spawn(
                port=self.port,
                hide_welcome_screen=True,
                default_blueprint=live_recording_blueprint(self.camera_names),
            )
        except Exception:
            self.recording.__exit__(None, None, None)
            self.recording = None
            raise

    def log(self, observation, action, *, recording, episode_frame):
        if self.recording is None:
            return
        self.recording.set_time("frame", sequence=self.frame)
        for name, image in observation["images"].items():
            self.recording.log(f"cameras/{name}", self.rr.Image(image))
        for index, name in enumerate(ACTION_NAMES):
            self.recording.log(
                f"state/qpos/{name}",
                self.rr.Scalars(observation["qpos"][index]),
            )
            self.recording.log(
                f"state/qvel/{name}",
                self.rr.Scalars(observation["qvel"][index]),
            )
            self.recording.log(
                f"expert/action/{name}", self.rr.Scalars(action[index]))
        task = observation["task"]
        self.recording.log(
            "task/success", self.rr.Scalars(float(task["success"])))
        self.recording.log(
            "task/object_position_error",
            self.rr.Scalars(task["object_position_error"]),
        )
        self.recording.log(
            "task/recording_active", self.rr.Scalars(float(recording)))
        self.recording.log(
            "task/episode_frame", self.rr.Scalars(float(episode_frame)))
        self.frame += 1

    def close(self):
        if self.recording is None:
            return
        try:
            self.recording.flush(timeout_sec=5.0)
            self.recording.disconnect()
        finally:
            self.recording.__exit__(None, None, None)
            self.recording = None


__all__ = ["LiveRecordingRerunLogger"]
