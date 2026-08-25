import importlib.util
import pathlib
import sys
import tempfile

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.data.episode import EpisodeData  # noqa: E402
from ffw_sh5_grasp.imitation.visualization.rerun_dataset import (
    _rerun,  # noqa: E402
    log_episode,  # noqa: E402
)
from ffw_sh5_grasp.imitation.visualization.rerun_live import (  # noqa: E402
    LiveRecordingRerunLogger,
)
from ffw_sh5_grasp.imitation.visualization.rerun_rollout import (  # noqa: E402
    RolloutRerunLogger,
)
from ffw_sh5_grasp.imitation.visualization.rerun_training import (  # noqa: E402
    TrainingRerunLogger,
)


class _FakeRerun:
    @staticmethod
    def Image(value):
        return ("image", value)

    @staticmethod
    def Scalars(value):
        return ("scalar", value)

    @staticmethod
    def Tensor(value, **_kwargs):
        return ("tensor", value)

    @staticmethod
    def TextLog(value):
        return ("text", value)


class _FakeRecording:
    def __init__(self):
        self.paths = []
        self.flushes = []

    def set_time(self, *_args, **_kwargs):
        pass

    def log(self, path, _value):
        self.paths.append(path)

    def flush(self, *, timeout_sec):
        self.flushes.append(timeout_sec)


class _BrokenRecording:
    def __init__(self):
        self.disconnects = 0
        self.exits = 0

    def flush(self, *, timeout_sec):
        raise RuntimeError(f"gRPC connection severed after {timeout_sec}s")

    def disconnect(self):
        self.disconnects += 1

    def __exit__(self, *_args):
        self.exits += 1
        raise RuntimeError("gRPC sink already closed")


def test_live_rerun_logs_three_cameras_and_ee_pose():
    logger = LiveRecordingRerunLogger(("cam_high", "cam_left_wrist", "cam_right_wrist"))
    logger.rr = _FakeRerun
    logger.recording = _FakeRecording()
    observation = {
        "images": {name: np.zeros((8, 8, 3), np.uint8) for name in logger.camera_names},
        "qpos": np.zeros(16),
        "qvel": np.zeros(16),
        "ee_pose": {"left": np.zeros(7), "right": np.ones(7)},
        "task": {"success": False, "object_position_error": 0.2},
    }
    logger.log(observation, np.zeros(16), recording=False, episode_frame=0)
    assert all(
        f"cameras/{name}" in logger.recording.paths for name in logger.camera_names
    )
    assert "state/ee_pose/left/qw" in logger.recording.paths
    assert "state/ee_pose/right/qz" in logger.recording.paths
    assert logger.recording.flushes == [2.0]


def test_task_rollout_rerun_separates_pose_and_executed_joint_action():
    logger = RolloutRerunLogger("unused.rrd", ("cam_high",), frame_stride=5)
    logger.rr = _FakeRerun
    logger.recording = _FakeRecording()
    observation = {
        "images": {"cam_high": np.zeros((8, 8, 3), np.uint8)},
        "qpos": np.zeros(16),
        "ee_pose": {"right": np.array([0.4, 0.0, 0.9, 1.0, 0.0, 0.0, 0.0])},
        "task": {"success": False, "object_position_error": 0.2},
    }
    logger.log(
        0,
        observation,
        np.ones(16),
        predicted_chunk=np.zeros((90, 8)),
        task_action=np.array([0.5, 0.1, 0.8, 1.0, 0.0, 0.0, 0.0, 0.75]),
        representation="task",
        ik_metrics={"position_error_mm": 12.0},
        temporal_metrics={
            "proleptic_steps": 5,
            "target_timestep": 5,
            "ensemble_candidate_count": 1,
        },
    )
    paths = logger.recording.paths
    assert "policy/task_target/x" in paths
    assert "policy/task_target/grasp" in paths
    assert "policy/executed/right_arm_joint_1" in paths
    assert "policy/ik/position_error_mm" in paths
    assert "policy/temporal/proleptic_steps" in paths
    assert "policy/temporal/ensemble_candidate_count" in paths
    assert "policy/action_chunk" in paths

    path_count = len(paths)
    logger.log(1, observation, np.ones(16))
    assert len(logger.recording.paths) == path_count


def test_rollout_rerun_disconnect_does_not_escape_or_close_twice():
    logger = RolloutRerunLogger("unused.rrd", ("cam_high",))
    recording = _BrokenRecording()
    logger.recording = recording

    assert logger.__exit__(None, None, None) is False
    assert isinstance(logger.close_error, RuntimeError)
    assert logger.recording is None
    assert recording.disconnects == 1
    assert recording.exits == 1

    assert logger.__exit__(None, None, None) is False
    assert recording.disconnects == 1
    assert recording.exits == 1


def test_rerun_dependency_boundary():
    if importlib.util.find_spec("rerun") is None:
        try:
            _rerun()
        except RuntimeError as error:
            assert "pip install rerun-sdk" in str(error)
        else:
            raise AssertionError("missing Rerun dependency was not explained")
    else:
        assert _rerun().__name__ == "rerun"
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            episode = EpisodeData(
                qpos=np.zeros((1, 16), np.float32),
                qvel=np.zeros((1, 16), np.float32),
                images={"cam_high": np.zeros((1, 8, 8, 3), np.uint8)},
                action=np.zeros((1, 16), np.float32),
                debug={},
                attrs={},
            )
            log_episode(episode, root / "dataset.rrd")
            with TrainingRerunLogger(root / "training.rrd") as logger:
                logger.log_epoch(
                    {
                        "epoch": 0,
                        "train/loss": 1.0,
                        "val/loss": 0.9,
                        "train/l1": 0.5,
                        "val/l1": 0.4,
                        "train/kl": 0.1,
                        "val/kl": 0.1,
                        "train/pad": 0.2,
                        "val/pad": 0.2,
                        "learning_rate": 1e-4,
                    }
                )
            observation = {
                "qpos": np.zeros(16),
                "images": {"cam_high": np.zeros((8, 8, 3), np.uint8)},
                "task": {"success": False, "object_position_error": 0.2},
            }
            with RolloutRerunLogger(root / "rollout.rrd", ("cam_high",)) as logger:
                logger.log(
                    0,
                    observation,
                    np.zeros(16),
                    predicted_chunk=np.zeros((2, 8)),
                    task_action=np.array([0.4, 0.0, 0.9, 1.0, 0.0, 0.0, 0.0, 0.5]),
                    representation="task",
                    ik_metrics={"position_error_mm": 1.0},
                )
            assert all(
                (root / name).stat().st_size > 0
                for name in ("dataset.rrd", "training.rrd", "rollout.rrd")
            )


if __name__ == "__main__":
    test_rerun_dependency_boundary()
    test_live_rerun_logs_three_cameras_and_ee_pose()
    test_task_rollout_rerun_separates_pose_and_executed_joint_action()
    test_rollout_rerun_disconnect_does_not_escape_or_close_twice()
    print("PASS")
