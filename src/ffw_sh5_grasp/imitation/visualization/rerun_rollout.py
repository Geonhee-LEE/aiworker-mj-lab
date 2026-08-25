"""Synchronized Rerun logs for policy rollout and expert comparison."""

from pathlib import Path

from ..data.schema import ACTION_NAMES
from .rerun_blueprints import rollout_blueprint

_POSE_COMPONENTS = ("x", "y", "z", "qw", "qx", "qy", "qz")
_TASK_ACTION_COMPONENTS = _POSE_COMPONENTS + ("grasp",)


class RolloutRerunLogger:
    def __init__(
        self,
        path,
        camera_names,
        *,
        enabled=True,
        application_id="aiworker_act_rollout",
        live=False,
        port=9877,
        frame_stride=1,
        image_jpeg_quality=85,
    ):
        self.path = Path(path)
        self.camera_names = tuple(camera_names)
        self.enabled = bool(enabled)
        self.application_id = application_id
        self.live = bool(live)
        self.port = int(port)
        self.frame_stride = int(frame_stride)
        if self.frame_stride <= 0:
            raise ValueError("Rerun frame stride must be positive")
        self.image_jpeg_quality = int(image_jpeg_quality)
        if not 1 <= self.image_jpeg_quality <= 100:
            raise ValueError("Rerun JPEG quality must be between 1 and 100")
        self.recording = None
        self.close_error = None

    def __enter__(self):
        if not self.enabled:
            return self
        try:
            import rerun as rr
        except ImportError as error:
            raise RuntimeError(
                "Rerun rollout logging requires: pip install rerun-sdk"
            ) from error
        self.rr = rr
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.recording = rr.RecordingStream(self.application_id)
        self.recording.__enter__()
        blueprint = rollout_blueprint(self.camera_names)
        try:
            self.recording.save(self.path, default_blueprint=blueprint)
            if self.live:
                self.recording.spawn(
                    port=self.port,
                    hide_welcome_screen=True,
                    default_blueprint=blueprint,
                )
                self.recording.send_blueprint(blueprint)
        except Exception:
            try:
                self.recording.__exit__(None, None, None)
            except Exception:
                # Preserve the original setup failure.  A half-open live
                # gRPC sink must not replace it with a secondary flush error.
                pass
            self.recording = None
            raise
        return self

    def log(
        self,
        frame,
        observation,
        action,
        *,
        predicted_chunk=None,
        expert_action=None,
        task_action=None,
        representation="joint",
        ik_metrics=None,
        temporal_metrics=None,
    ):
        if self.recording is None:
            return
        if int(frame) % self.frame_stride:
            return
        self.recording.set_time("frame", sequence=int(frame))
        for name, image in observation["images"].items():
            image_log = self.rr.Image(image)
            if hasattr(image_log, "compress"):
                image_log = image_log.compress(jpeg_quality=self.image_jpeg_quality)
            self.recording.log(f"cameras/{name}", image_log)
        for side, pose in observation.get("ee_pose", {}).items():
            for component, value in zip(_POSE_COMPONENTS, pose):
                self.recording.log(
                    f"state/ee_pose/{side}/{component}", self.rr.Scalars(float(value))
                )
        for index, name in enumerate(ACTION_NAMES):
            self.recording.log(
                f"state/qpos/{name}", self.rr.Scalars(observation["qpos"][index])
            )
            self.recording.log(
                f"policy/executed/{name}", self.rr.Scalars(action[index])
            )
            if expert_action is not None:
                self.recording.log(
                    f"expert/action/{name}", self.rr.Scalars(expert_action[index])
                )
        if predicted_chunk is not None:
            self.recording.log(
                "policy/action_chunk",
                self.rr.Tensor(
                    predicted_chunk, dim_names=("future_timestep", "action_dimension")
                ),
            )
        if task_action is not None:
            for component, value in zip(_TASK_ACTION_COMPONENTS, task_action):
                self.recording.log(
                    f"policy/task_target/{component}", self.rr.Scalars(float(value))
                )
        if ik_metrics is not None:
            for name, value in ik_metrics.items():
                self.recording.log(f"policy/ik/{name}", self.rr.Scalars(float(value)))
        if temporal_metrics is not None:
            for name, value in temporal_metrics.items():
                self.recording.log(
                    f"policy/temporal/{name}", self.rr.Scalars(float(value))
                )
        if int(frame) == 0:
            self.recording.log(
                "policy/representation", self.rr.TextLog(str(representation))
            )
        task = observation["task"]
        self.recording.log("task/success", self.rr.Scalars(float(task["success"])))
        self.recording.log(
            "task/object_position_error", self.rr.Scalars(task["object_position_error"])
        )

    def __exit__(self, type_, value, traceback):
        recording = self.recording
        # Clear ownership before touching the sink so repeated cleanup after
        # an exception is harmless.
        self.recording = None
        if recording is not None:
            self.close_error = None
            try:
                recording.flush(timeout_sec=2.0)
            except Exception as error:
                self.close_error = error
                try:
                    recording.disconnect()
                except Exception:
                    pass
            try:
                recording.__exit__(type_, value, traceback)
            except Exception as error:
                if self.close_error is None:
                    self.close_error = error
            # Rerun is optional diagnostics. A viewer/proxy disconnect must
            # never terminate policy control or mask the caller's exception.
            return False
        return False


__all__ = ["RolloutRerunLogger"]
