"""Convert an ALOHA HDF5 episode into a synchronized Rerun recording."""

from pathlib import Path

from ..action import ACTION_NAMES
from ..dataset import load_episode
from .rerun_blueprints import dataset_blueprint


def _rerun():
    try:
        import rerun as rr
    except ImportError as error:
        raise RuntimeError(
            "Rerun visualization requires: pip install rerun-sdk") from error
    return rr


def log_episode(episode_or_path, output_path, *, application_id="aiworker_dataset"):
    rr = _rerun()
    episode = (load_episode(episode_or_path)
               if not hasattr(episode_or_path, "action") else episode_or_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    blueprint = dataset_blueprint(tuple(episode.images))
    with rr.RecordingStream(application_id) as recording:
        recording.save(output_path, default_blueprint=blueprint)
        for frame in range(episode.length):
            recording.set_time("frame", sequence=frame)
            for name, images in episode.images.items():
                recording.log(f"cameras/{name}", rr.Image(images[frame]))
            for index, name in enumerate(ACTION_NAMES):
                recording.log(
                    f"state/qpos/{name}", rr.Scalars(episode.qpos[frame, index]))
                recording.log(
                    f"state/qvel/{name}", rr.Scalars(episode.qvel[frame, index]))
                recording.log(
                    f"expert/action/{name}", rr.Scalars(episode.action[frame, index]))
    return output_path


__all__ = ["log_episode"]
