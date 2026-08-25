import pathlib
import sys
import tempfile

import numpy as np
import torch
import yaml

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.act.representations import (  # noqa: E402
    create_representation,
)
from ffw_sh5_grasp.imitation.act.trainer import (  # noqa: E402
    train,
)
from ffw_sh5_grasp.imitation.act.training_config import (  # noqa: E402
    ACTTrainingConfig,
)
from ffw_sh5_grasp.imitation.data.episode import (  # noqa: E402
    EpisodeData,
    write_episode,
)
from ffw_sh5_grasp.imitation.simulation.environment import (  # noqa: E402
    AIWorkerMujocoEnv,
)


def _episode(path, length=3):
    with AIWorkerMujocoEnv(
        render_images=False, seed=4, task_name="can_color_sort"
    ) as env:
        observation = env.get_observation()
        qpos = np.repeat(observation["qpos"][None], length, axis=0)
        qvel = np.repeat(observation["qvel"][None], length, axis=0)
        action = np.repeat(env.last_action[None], length, axis=0)
        ee_pose = {
            name: np.repeat(values[None], length, axis=0)
            for name, values in observation["ee_pose"].items()
        }
        full_qpos = np.repeat(observation["debug"]["full_qpos"][None], length, axis=0)
        images = {
            name: np.zeros((length, 32, 32, 3), dtype=np.uint8)
            for name in ("cam_high", "cam_right_wrist")
        }
        return write_episode(
            path,
            EpisodeData(
                qpos=qpos,
                qvel=qvel,
                ee_pose=ee_pose,
                images=images,
                action=action,
                debug={"full_qpos": full_qpos},
                attrs={
                    "seed": 4,
                    "model_hash": env.model_hash,
                    "ee_pose_frame": "world",
                    "ee_pose_quaternion_order": "wxyz",
                    "task_name": "can_color_sort",
                    "scenario_name": "can_color_sort",
                    "success": True,
                },
            ),
        )


def _small_config(root, dataset_dir):
    return {
        "run_name": "task_smoke",
        "dataset_dir": str(dataset_dir),
        "output_dir": str(root / "outputs"),
        "representation": "task",
        "camera_names": ["cam_high", "cam_right_wrist"],
        "policy_side": "right",
        "state_dim": 8,
        "action_dim": 8,
        "chunk_size": 2,
        "hidden_dim": 32,
        "latent_dim": 8,
        "encoder_layers": 1,
        "decoder_layers": 1,
        "feedforward_dim": 64,
        "attention_heads": 4,
        "dropout": 0.0,
        "pretrained_backbone": False,
        "batch_size": 3,
        "epochs": 1,
        "learning_rate": 1e-4,
        "backbone_learning_rate": 1e-4,
        "weight_decay": 0.0,
        "kl_weight": 1.0,
        "validation_fraction": 0.1,
        "test_fraction": 0.1,
        "split_seed": 1,
        "training_seed": 1,
        "num_workers": 0,
        "prefetch_factor": 1,
        "device": "cpu",
        "checkpoint_interval": 100,
        "plot_interval": 100,
        "rerun": False,
    }


def test_joint_and_task_representations_share_an_8d_contract():
    with tempfile.TemporaryDirectory() as directory:
        path = _episode(pathlib.Path(directory) / "episode_000000.hdf5")
        joint = create_representation("joint").episode_features(path)
        task = create_representation("task").episode_features(path)

        assert joint.state.shape == joint.action.shape == (3, 8)
        assert task.state.shape == task.action.shape == (3, 8)
        assert np.allclose(task.state[:, :7], task.action[:, :7], atol=1e-3)
        assert np.allclose(np.linalg.norm(task.state[:, 3:7], axis=1), 1.0)
        assert np.all(task.state[:, 3] >= 0.0)


def test_task_smoke_training_writes_representation_metadata():
    with tempfile.TemporaryDirectory() as directory:
        root = pathlib.Path(directory)
        dataset_dir = root / "dataset"
        _episode(dataset_dir / "episode_000000.hdf5")
        values = _small_config(root, dataset_dir)
        config_path = root / "task.yaml"
        with config_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(values, stream)

        config = ACTTrainingConfig.load(config_path)
        assert config.representation == "task"
        run_dir = train(config_path)

        best = torch.load(
            run_dir / "checkpoints/policy_best.ckpt",
            map_location="cpu",
            weights_only=True,
        )
        last = torch.load(
            run_dir / "checkpoints/policy_last.ckpt",
            map_location="cpu",
            weights_only=True,
        )
        assert best["representation"] == "task"
        assert best["representation_metadata"]["ee_pose_frame"] == "world"
        assert "optimizer" not in best
        assert "optimizer" in last


def test_color_sort_joint_and_task_configs_differ_only_by_run_and_representation():
    paths = (
        REPO_ROOT / "config/imitation/act_color_sort_joint.yaml",
        REPO_ROOT / "config/imitation/act_color_sort_task.yaml",
    )
    values = []
    for path in paths:
        with path.open("r", encoding="utf-8") as stream:
            values.append(yaml.safe_load(stream))
    for item in values:
        item.pop("run_name")
        item.pop("representation")
    assert values[0] == values[1]


if __name__ == "__main__":
    test_joint_and_task_representations_share_an_8d_contract()
    test_task_smoke_training_writes_representation_metadata()
    test_color_sort_joint_and_task_configs_differ_only_by_run_and_representation()
    print("PASS")
