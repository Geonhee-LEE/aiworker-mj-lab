"""Synchronized Rerun logs for policy rollout and expert comparison."""

from pathlib import Path

from ..data.schema import ACTION_NAMES
from .rerun_blueprints import rollout_blueprint


class RolloutRerunLogger:
    def __init__(self, path, camera_names, *, enabled=True,
                 application_id="aiworker_act_rollout"):
        self.path = Path(path)
        self.camera_names = tuple(camera_names)
        self.enabled = bool(enabled)
        self.application_id = application_id
        self.recording = None

    def __enter__(self):
        if not self.enabled:
            return self
        try:
            import rerun as rr
        except ImportError as error:
            raise RuntimeError(
                "Rerun rollout logging requires: pip install rerun-sdk") from error
        self.rr = rr
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.recording = rr.RecordingStream(self.application_id)
        self.recording.__enter__()
        self.recording.save(
            self.path, default_blueprint=rollout_blueprint(self.camera_names))
        return self

    def log(self, frame, observation, action, *, predicted_chunk=None,
            expert_action=None):
        if self.recording is None:
            return
        self.recording.set_time("frame", sequence=int(frame))
        for name, image in observation["images"].items():
            self.recording.log(f"cameras/{name}", self.rr.Image(image))
        for index, name in enumerate(ACTION_NAMES):
            self.recording.log(
                f"state/qpos/{name}", self.rr.Scalars(observation["qpos"][index]))
            self.recording.log(
                f"policy/executed/{name}", self.rr.Scalars(action[index]))
            if expert_action is not None:
                self.recording.log(
                    f"expert/action/{name}",
                    self.rr.Scalars(expert_action[index]))
        if predicted_chunk is not None:
            self.recording.log(
                "policy/action_chunk",
                self.rr.Tensor(
                    predicted_chunk,
                    dim_names=("future_timestep", "action_dimension")))
        task = observation["task"]
        self.recording.log(
            "task/success", self.rr.Scalars(float(task["success"])))
        self.recording.log(
            "task/object_position_error",
            self.rr.Scalars(task["object_position_error"]))

    def __exit__(self, type_, value, traceback):
        if self.recording is not None:
            return self.recording.__exit__(type_, value, traceback)
        return False


__all__ = ["RolloutRerunLogger"]
