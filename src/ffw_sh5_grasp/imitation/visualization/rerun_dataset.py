"""Convert an ALOHA HDF5 episode into a synchronized Rerun recording."""

from pathlib import Path

import mujoco

from ..data.episode import load_episode
from ..data.schema import ACTION_NAMES
from ..simulation.environment import AIWorkerMujocoEnv
from .rerun_blueprints import dataset_blueprint
from .rerun_robot import MujocoRobotRerunLogger

_POSE_COMPONENTS = ("x", "y", "z", "qw", "qx", "qy", "qz")


def _rerun():
    try:
        import rerun as rr
    except ImportError as error:
        raise RuntimeError(
            "Rerun visualization requires: pip install rerun-sdk"
        ) from error
    return rr


def _record_episode(recording, rr, episode):
    full_qpos = episode.debug.get("full_qpos")
    full_qvel = episode.debug.get("full_qvel")
    task_name = episode.attrs.get(
        "scenario_name", episode.attrs.get("task_name", "can_to_box")
    )
    with AIWorkerMujocoEnv(render_images=False, task_name=task_name) as environment:
        seed = int(episode.attrs.get("seed", -1))
        environment.reset(seed=None if seed < 0 else seed)
        robot = MujocoRobotRerunLogger(recording, environment.model, environment.data)
        for frame in range(episode.length):
            recording.set_time("frame", sequence=frame)
            if full_qpos is not None:
                environment.data.qpos[:] = full_qpos[frame]
                if full_qvel is not None:
                    environment.data.qvel[:] = full_qvel[frame]
                mujoco.mj_forward(environment.model, environment.data)
            if frame == 0:
                robot.log_geometry()
            robot.log_poses()
            for name, images in episode.images.items():
                recording.log(f"cameras/{name}", rr.Image(images[frame]))
            for index, name in enumerate(ACTION_NAMES):
                recording.log(
                    f"state/qpos/{name}", rr.Scalars(episode.qpos[frame, index])
                )
                recording.log(
                    f"state/qvel/{name}", rr.Scalars(episode.qvel[frame, index])
                )
                recording.log(
                    f"expert/action/{name}", rr.Scalars(episode.action[frame, index])
                )
            for side, poses in episode.ee_pose.items():
                for component, value in zip(_POSE_COMPONENTS, poses[frame]):
                    recording.log(
                        f"state/ee_pose/{side}/{component}", rr.Scalars(float(value))
                    )


def _episode(episode_or_path):
    return (
        load_episode(episode_or_path)
        if not hasattr(episode_or_path, "action")
        else episode_or_path
    )


def log_episode(episode_or_path, output_path, *, application_id="aiworker_dataset"):
    rr = _rerun()
    episode = _episode(episode_or_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with rr.RecordingStream(application_id) as recording:
        recording.save(
            output_path, default_blueprint=dataset_blueprint(tuple(episode.images))
        )
        _record_episode(recording, rr, episode)
    return output_path


def stream_episode(episode_or_path, *, port=9877, application_id="aiworker_dataset"):
    """Open Rerun and stream an episode without writing an intermediate file."""
    rr = _rerun()
    episode = _episode(episode_or_path)
    blueprint = dataset_blueprint(tuple(episode.images))
    with rr.RecordingStream(application_id) as recording:
        recording.spawn(
            port=int(port), hide_welcome_screen=True, default_blueprint=blueprint
        )
        recording.send_blueprint(blueprint)
        _record_episode(recording, rr, episode)
        recording.flush(timeout_sec=5.0)


__all__ = ["log_episode", "stream_episode"]
