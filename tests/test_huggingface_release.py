from pathlib import Path

import h5py
import numpy as np

from scripts.prepare_huggingface_release import inspect_episode
from scripts.publish_huggingface import set_revision_tag


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


def test_release_tag_moves_to_final_main_revision():
    class Api:
        def __init__(self):
            self.deleted = []
            self.created = []

        def list_repo_refs(self, _repo_id, *, repo_type):
            tag = type("Tag", (), {"name": "v3.1.0"})()
            return type("Refs", (), {"tags": [tag]})()

        def delete_tag(self, repo_id, *, tag, repo_type):
            self.deleted.append((repo_id, tag, repo_type))

        def repo_info(self, _repo_id, *, repo_type):
            return type("Info", (), {"sha": "final-main-sha"})()

        def create_tag(self, repo_id, **kwargs):
            self.created.append((repo_id, kwargs))

    api = Api()
    revision = set_revision_tag(api, "owner/repo", "dataset", "v3.1.0")
    assert revision == "final-main-sha"
    assert api.deleted == [("owner/repo", "v3.1.0", "dataset")]
    assert api.created == [
        (
            "owner/repo",
            {
                "tag": "v3.1.0",
                "tag_message": "Release v3.1.0",
                "revision": "final-main-sha",
                "repo_type": "dataset",
            },
        )
    ]
