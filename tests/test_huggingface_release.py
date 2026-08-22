from pathlib import Path

import h5py
import numpy as np

from scripts.prepare_huggingface_release import inspect_episode


def test_release_inspection_reads_episode_metadata(tmp_path: Path):
    path = tmp_path / "episode_000000.hdf5"
    frames = 3
    with h5py.File(path, "w") as root:
        root.create_dataset("action", data=np.zeros((frames, 16), np.float32))
        observations = root.create_group("observations")
        observations.create_dataset("qpos", data=np.zeros((frames, 16), np.float32))
        observations.create_dataset("qvel", data=np.zeros((frames, 16), np.float32))
        ee_pose = observations.create_group("ee_pose")
        ee_pose.create_dataset("left", data=np.zeros((frames, 7), np.float32))
        ee_pose.create_dataset("right", data=np.zeros((frames, 7), np.float32))
        images = observations.create_group("images")
        for camera in ("cam_high", "cam_left_wrist", "cam_right_wrist"):
            images.create_dataset(camera, data=np.zeros((frames, 2, 2, 3), np.uint8))
        root.attrs.update(
            success=True,
            control_hz=25.0,
            object_variant="orange",
            target_label="red",
            schema_version="1.1",
        )

    result = inspect_episode(path, include_hash=True)
    assert result["frames"] == 3
    assert result["duration_s"] == 0.12
    assert result["object_variant"] == "orange"
    assert result["target_label"] == "red"
    assert len(result["sha256"]) == 64
