"""Closed-loop signed world-EE-Y Grad-CAM analysis for four ACT policies.

This script intentionally does not consume expert HDF5 observations during the
rollout.  HDF5 files are read only during the preflight audit that recomputes
the train-split normalization statistics.
"""

from __future__ import annotations

import argparse
import gc
import hashlib
import json
import math
import pickle
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import torch
from torch.nn import functional as F

from ffw_sh5_grasp.control import whole_body
from ffw_sh5_grasp.imitation.act.dataset_loader import compute_stats
from ffw_sh5_grasp.imitation.act.representations import create_representation
from ffw_sh5_grasp.imitation.data.schema import ARM_JOINTS
from ffw_sh5_grasp.imitation.runtime.runner import ACTPolicyRunner
from ffw_sh5_grasp.imitation.runtime.task_space import task_action_to_joint
from ffw_sh5_grasp.imitation.simulation.environment import AIWorkerMujocoEnv

SEED = 195958
FRAMES = tuple(range(100, 301, 20))
DECISION_FRAME = 120
CAMERAS = ("cam_high", "cam_right_wrist")
STABLE_SUCCESS_STEPS = 10
TASK_IK_SPEED_SCALE = 3.0
POLICIES = {
    "d097_joint": {
        "label": "D97 Joint",
        "data_count": 97,
        "representation": "joint",
        "checkpoint": Path(
            "outputs/act_modular/can_color_sort_act_joint/checkpoints/policy_best.ckpt"
        ),
    },
    "d097_task": {
        "label": "D97 Task",
        "data_count": 97,
        "representation": "task",
        "checkpoint": Path(
            "outputs/act_modular/can_color_sort_act_task/checkpoints/policy_best.ckpt"
        ),
    },
    "d150_joint": {
        "label": "D150 Joint",
        "data_count": 150,
        "representation": "joint",
        "checkpoint": Path(
            "outputs/act_modular/can_color_sort_act_joint_aug150/"
            "checkpoints/policy_best.ckpt"
        ),
    },
    "d150_task": {
        "label": "D150 Task",
        "data_count": 150,
        "representation": "task",
        "checkpoint": Path(
            "outputs/act_modular/can_color_sort_act_task_aug150/"
            "checkpoints/policy_best.ckpt"
        ),
    },
}


@dataclass(frozen=True)
class CamResult:
    images: np.ndarray
    correct_heatmaps: np.ndarray
    wrong_heatmaps: np.ndarray
    correct_raw_max: np.ndarray
    wrong_raw_max: np.ndarray
    gradient_abs_mean: np.ndarray
    current_ee_y: float
    predicted_ee_y: np.ndarray
    predicted_delta_y: np.ndarray
    predicted_chunk: np.ndarray
    positive_target_value: float
    negative_target_value: float


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _array_hash(*arrays: np.ndarray) -> str:
    digest = hashlib.sha256()
    for value in arrays:
        array = np.ascontiguousarray(value)
        digest.update(str(array.dtype).encode())
        digest.update(str(array.shape).encode())
        digest.update(array.tobytes())
    return digest.hexdigest()


def _json_value(value):
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    if isinstance(value, np.ndarray):
        return _json_value(value.tolist())
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        value = float(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"refusing to serialize non-finite value: {value}")
        return value
    return value


def _load_split(run_dir: Path) -> dict[str, list[Path]]:
    values = json.loads((run_dir / "episode_splits.json").read_text())
    return {
        name: [Path(item).resolve() for item in paths] for name, paths in values.items()
    }


def _preflight(repo: Path) -> dict:
    """Audit splits, checkpoint metadata, and exact train-only statistics."""
    audits = {}
    split_hashes = {}
    cached_stats = {}
    for key, spec in POLICIES.items():
        checkpoint_path = (repo / spec["checkpoint"]).resolve()
        run_dir = checkpoint_path.parent.parent
        stats_path = run_dir / "dataset_stats.pkl"
        split_path = run_dir / "episode_splits.json"
        if not checkpoint_path.is_file():
            raise FileNotFoundError(checkpoint_path)
        checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
        representation = spec["representation"]
        config = checkpoint["policy_config"]
        metadata = checkpoint.get("representation_metadata", {})
        expected_names = (
            [f"right_arm_joint_{index}" for index in range(1, 8)] + ["right_grasp"]
            if representation == "joint"
            else [
                "target_right_ee_x",
                "target_right_ee_y",
                "target_right_ee_z",
                "target_right_ee_qw",
                "target_right_ee_qx",
                "target_right_ee_qy",
                "target_right_ee_qz",
                "target_right_grasp",
            ]
        )
        checks = {
            "checkpoint_representation": checkpoint.get("representation")
            == representation,
            "metadata_representation": metadata.get("name") == representation,
            "action_names_match": metadata.get("action_names") == expected_names,
            "state_action_dim_8": config.get("state_dim") == 8
            and config.get("action_dim") == 8,
            "chunk_size_90": config.get("chunk_size") == 90,
            "camera_names_match": tuple(checkpoint.get("camera_names", ())) == CAMERAS,
        }
        if representation == "joint":
            checks["right_policy_indices"] = tuple(
                checkpoint.get("policy_indices", ())
            ) == tuple(range(8, 16))
        else:
            checks["world_wxyz_task_pose"] = (
                metadata.get("ee_pose_frame") == "world"
                and metadata.get("ee_pose_quaternion_order") == "wxyz"
            )

        splits = _load_split(run_dir)
        split_count = sum(len(paths) for paths in splits.values())
        checks["data_count_matches"] = split_count == spec["data_count"]
        split_hashes[key] = _sha256(split_path)

        cache_key = (representation, split_hashes[key])
        if cache_key not in cached_stats:
            representation_adapter = create_representation(representation)
            cached_stats[cache_key] = compute_stats(
                splits["train"], representation_adapter
            )
        recomputed = cached_stats[cache_key]
        with stats_path.open("rb") as stream:
            saved = pickle.load(stream)
        differences = {}
        for name in ("qpos_mean", "qpos_std", "action_mean", "action_std"):
            expected = np.asarray(getattr(recomputed, name))
            actual = np.asarray(saved[name])
            differences[name] = float(np.max(np.abs(expected - actual)))
        checks["statistics_exact_recompute"] = all(
            difference <= 1e-7 for difference in differences.values()
        )
        if not all(checks.values()):
            failed = [name for name, passed in checks.items() if not passed]
            raise ValueError(f"preflight failed for {key}: {failed}")
        audits[key] = {
            "checkpoint": str(checkpoint_path),
            "checkpoint_sha256": _sha256(checkpoint_path),
            "stats": str(stats_path),
            "stats_sha256": _sha256(stats_path),
            "split": str(split_path),
            "split_sha256": split_hashes[key],
            "split_counts": {name: len(paths) for name, paths in splits.items()},
            "checkpoint_epoch": int(checkpoint["epoch"]),
            "checkpoint_validation_loss": float(checkpoint["validation_loss"]),
            "statistics_max_abs_difference": differences,
            "checks": checks,
        }
        del checkpoint
        gc.collect()

    pair_checks = {
        "d097_joint_task_identical_split": (
            split_hashes["d097_joint"] == split_hashes["d097_task"]
        ),
        "d150_joint_task_identical_split": (
            split_hashes["d150_joint"] == split_hashes["d150_task"]
        ),
    }
    if not all(pair_checks.values()):
        raise ValueError(f"paired split validation failed: {pair_checks}")
    return {"policies": audits, "paired_data_checks": pair_checks}


def _right_jacobian_y(env: AIWorkerMujocoEnv) -> np.ndarray:
    site_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r")
    jacobian_position = np.zeros((3, env.model.nv), dtype=np.float64)
    jacobian_rotation = np.zeros((3, env.model.nv), dtype=np.float64)
    mujoco.mj_jacSite(
        env.model,
        env.data,
        jacobian_position,
        jacobian_rotation,
        site_id,
    )
    return jacobian_position[1, env.state_adapter.arm_dofs["r"]].copy()


def _signed_gradcam(
    runner: ACTPolicyRunner,
    observation: dict,
    env: AIWorkerMujocoEnv,
) -> CamResult:
    """Explain mean steps 1..89 EE-Y displacement with both target signs."""
    qpos, images = runner._inputs(observation)
    activation = None

    def capture(_module, _inputs, output):
        nonlocal activation
        activation = output

    handle = runner.policy.image_projection.register_forward_hook(capture)
    runner.policy.zero_grad(set_to_none=True)
    try:
        output = runner.policy(qpos, images)
        normalized = output["actions"]
        action_mean = torch.as_tensor(
            runner.stats.action_mean,
            device=normalized.device,
            dtype=normalized.dtype,
        )
        action_std = torch.as_tensor(
            runner.stats.action_std,
            device=normalized.device,
            dtype=normalized.dtype,
        )
        physical = normalized[0] * action_std + action_mean
        current_ee_y = float(observation["ee_pose"]["right"][1])
        if runner.representation == "joint":
            jacobian_y = torch.as_tensor(
                _right_jacobian_y(env),
                device=normalized.device,
                dtype=normalized.dtype,
            )
            current_right_q = torch.as_tensor(
                observation["qpos"][8:15],
                device=normalized.device,
                dtype=normalized.dtype,
            )
            projected_delta = ((physical[1:, :7] - current_right_q) * jacobian_y).sum(
                dim=-1
            )
            predicted_ee_y_tensor = current_ee_y + projected_delta
        else:
            predicted_ee_y_tensor = physical[1:, 1]
            projected_delta = predicted_ee_y_tensor - current_ee_y
        positive_score = projected_delta.mean()
        gradients = torch.autograd.grad(positive_score, activation)[0]
    finally:
        handle.remove()

    weights = gradients.mean(dim=(2, 3), keepdim=True)
    signed_raw = (weights * activation).sum(dim=1, keepdim=True)
    signed_raw = F.interpolate(
        signed_raw,
        size=images.shape[-2:],
        mode="bilinear",
        align_corners=False,
    )[:, 0]
    positive_raw = torch.relu(signed_raw)
    negative_raw = torch.relu(-signed_raw)
    positive_max = positive_raw.flatten(1).max(dim=1).values
    negative_max = negative_raw.flatten(1).max(dim=1).values
    positive = positive_raw / positive_max.clamp_min(1e-12)[:, None, None]
    negative = negative_raw / negative_max.clamp_min(1e-12)[:, None, None]
    gradient_abs_mean = gradients.abs().mean(dim=(1, 2, 3))
    predicted_ee_y = predicted_ee_y_tensor.detach().cpu().numpy().astype(np.float32)
    predicted_delta_y = predicted_ee_y - np.float32(current_ee_y)
    return CamResult(
        images=np.stack([observation["images"][name] for name in CAMERAS]),
        correct_heatmaps=positive.detach().cpu().numpy().astype(np.float32),
        wrong_heatmaps=negative.detach().cpu().numpy().astype(np.float32),
        correct_raw_max=positive_max.detach().cpu().numpy().astype(np.float32),
        wrong_raw_max=negative_max.detach().cpu().numpy().astype(np.float32),
        gradient_abs_mean=gradient_abs_mean.detach().cpu().numpy().astype(np.float32),
        current_ee_y=current_ee_y,
        predicted_ee_y=predicted_ee_y,
        predicted_delta_y=predicted_delta_y,
        predicted_chunk=physical.detach().cpu().numpy().astype(np.float32),
        positive_target_value=float(positive_score.detach().cpu()),
        negative_target_value=float(-positive_score.detach().cpu()),
    )


def _save_frame_npz(
    path: Path,
    result: CamResult,
    observation: dict,
    *,
    frame: int,
    stable_success: bool,
    normal_success_step: int | None,
):
    can_position = np.asarray(observation["debug"]["task_object_pose"][:3])
    target_position = np.asarray(observation["debug"]["target_position"])
    payload = {
        "camera_names": np.asarray(CAMERAS),
        "rgb_images": result.images,
        "correct_heatmaps": result.correct_heatmaps,
        "wrong_heatmaps": result.wrong_heatmaps,
        "correct_heatmap_raw_max": result.correct_raw_max,
        "wrong_heatmap_raw_max": result.wrong_raw_max,
        "raw_heatmap_maxima": np.stack((result.correct_raw_max, result.wrong_raw_max)),
        "gradient_absolute_mean": result.gradient_abs_mean,
        "current_ee_y": np.asarray(result.current_ee_y, dtype=np.float32),
        "predicted_ee_y": result.predicted_ee_y,
        "predicted_delta_y": result.predicted_delta_y,
        "predicted_chunk": result.predicted_chunk,
        "can_position": can_position.astype(np.float32),
        "target_position": target_position.astype(np.float32),
        "instantaneous_success": np.asarray(
            observation["task"]["success"], dtype=np.bool_
        ),
        "stable_success": np.asarray(stable_success, dtype=np.bool_),
        "post_success_continuation": np.asarray(
            normal_success_step is not None and frame > normal_success_step,
            dtype=np.bool_,
        ),
        "positive_target_value": np.asarray(
            result.positive_target_value, dtype=np.float32
        ),
        "negative_target_value": np.asarray(
            result.negative_target_value, dtype=np.float32
        ),
        "behavior_target_sign": np.asarray(
            1 if float(result.predicted_delta_y.mean()) >= 0.0 else -1,
            dtype=np.int8,
        ),
        "frame": np.asarray(frame, dtype=np.int64),
    }
    np.savez_compressed(path, **payload)


def _rollout_policy(repo: Path, output_dir: Path, key: str, spec: dict) -> dict:
    checkpoint = (repo / spec["checkpoint"]).resolve()
    policy_dir = output_dir / key
    frame_dir = policy_dir / "frames"
    frame_dir.mkdir(parents=True)
    runner = ACTPolicyRunner(
        checkpoint,
        device="auto",
        representation=spec["representation"],
        proleptic_steps=0,
    )
    observations = {}
    cam_results = {}
    actions = []
    task_actions = []
    can_positions = []
    ee_y_positions = []
    instantaneous_success = []
    stable_success_history = []
    prediction_replay_errors = []
    success_streak = 0
    stable_success = False
    normal_success_step = None

    with AIWorkerMujocoEnv(
        render_images=True,
        camera_names=CAMERAS,
        task_name="can_color_sort",
        randomize_bin_colors=True,
    ) as env:
        solver = None
        if runner.representation == "task":
            solver = whole_body.WholeBodyIK(
                env.model,
                {"r": "grasp_target_r", "l": "grasp_target_l"},
                ARM_JOINTS,
            )
        observation = env.reset(seed=SEED)
        runner.reset()
        if solver is not None:
            solver.rebase(env.data)
        initial_hash = _array_hash(
            observation["debug"]["full_qpos"],
            observation["debug"]["full_qvel"],
            observation["debug"]["task_object_pose"],
            observation["debug"]["target_position"],
        )
        initial_qpos = observation["debug"]["full_qpos"].copy()
        initial_qvel = observation["debug"]["full_qvel"].copy()
        initial_rgb = np.stack([observation["images"][name] for name in CAMERAS])

        for frame in range(301):
            can_positions.append(
                np.asarray(observation["debug"]["task_object_pose"][:3]).copy()
            )
            ee_y_positions.append(float(observation["ee_pose"]["right"][1]))
            instantaneous_success.append(bool(observation["task"]["success"]))
            stable_success_history.append(stable_success)
            if frame in FRAMES:
                result = _signed_gradcam(runner, observation, env)
                cam_results[frame] = result
                observations[frame] = {
                    "can_position": np.asarray(
                        observation["debug"]["task_object_pose"][:3]
                    ).copy(),
                    "target_position": np.asarray(
                        observation["debug"]["target_position"]
                    ).copy(),
                    "instantaneous_success": bool(observation["task"]["success"]),
                    "stable_success": stable_success,
                }
            if frame == 300:
                final_observation = observation
                break

            joint_action, policy_info = runner.get_action(observation)
            predicted_chunk = policy_info["predicted_chunk"]
            if frame in FRAMES:
                result = cam_results[frame]
                if runner.representation == "joint":
                    replay_y = predicted_chunk[:, 8:15]
                    cam_y = result.predicted_chunk[:, :7]
                    error = float(np.max(np.abs(replay_y - cam_y)))
                else:
                    error = float(
                        np.max(
                            np.abs(predicted_chunk[:, 1] - result.predicted_chunk[:, 1])
                        )
                    )
                prediction_replay_errors.append(error)
            task_action = None
            if runner.representation == "task":
                task_action = joint_action.copy()
                joint_action, _diagnostics = task_action_to_joint(
                    env,
                    solver,
                    task_action,
                    speed_scale=TASK_IK_SPEED_SCALE,
                )
            joint_action = env.prepare_action(joint_action)
            actions.append(joint_action.copy())
            if task_action is not None:
                task_actions.append(task_action.copy())
            observation = env.step(joint_action)
            if observation["task"]["success"]:
                success_streak += 1
            else:
                success_streak = 0
            if not stable_success and success_streak >= STABLE_SUCCESS_STEPS:
                stable_success = True
                normal_success_step = frame + 1

        final_rgb = np.stack([final_observation["images"][name] for name in CAMERAS])
        final_can_position = np.asarray(
            final_observation["debug"]["task_object_pose"][:3]
        ).copy()
        target_position = np.asarray(
            final_observation["debug"]["target_position"]
        ).copy()
        environment = {
            "initial_state_sha256": initial_hash,
            "initial_rgb_sha256": _array_hash(initial_rgb),
            "model_sha256": env.model_hash,
            "object_variant": final_observation["task"]["object_variant"],
            "target_label": final_observation["task"]["target_label"],
            "bin_colors_swapped": bool(env.task.bin_colors_swapped),
            "bin_color_layout": env.task.bin_color_layout,
            "control_hz": float(env.actual_control_hz),
        }

    for frame in FRAMES:
        obs = observations[frame]
        _save_frame_npz(
            frame_dir / f"frame_{frame:04d}.npz",
            cam_results[frame],
            {
                "debug": {
                    "task_object_pose": np.r_[obs["can_position"], [1, 0, 0, 0]],
                    "target_position": obs["target_position"],
                },
                "task": {"success": obs["instantaneous_success"]},
            },
            frame=frame,
            stable_success=obs["stable_success"],
            normal_success_step=normal_success_step,
        )

    rollout_payload = {
        "camera_names": np.asarray(CAMERAS),
        "frames": np.arange(301, dtype=np.int64),
        "can_positions": np.asarray(can_positions, dtype=np.float32),
        "ee_y_positions": np.asarray(ee_y_positions, dtype=np.float32),
        "instantaneous_success": np.asarray(instantaneous_success, dtype=np.bool_),
        "stable_success": np.asarray(stable_success_history, dtype=np.bool_),
        "executed_joint_actions": np.asarray(actions, dtype=np.float32),
        "initial_full_qpos": np.asarray(initial_qpos, dtype=np.float32),
        "initial_full_qvel": np.asarray(initial_qvel, dtype=np.float32),
        "initial_rgb_images": initial_rgb,
        "final_rgb_images": final_rgb,
        "final_can_position": final_can_position.astype(np.float32),
        "target_position": target_position.astype(np.float32),
        "normal_success_step": np.asarray(
            -1 if normal_success_step is None else normal_success_step,
            dtype=np.int64,
        ),
    }
    if task_actions:
        rollout_payload["executed_task_actions"] = np.asarray(
            task_actions, dtype=np.float32
        )
    np.savez_compressed(policy_dir / "rollout.npz", **rollout_payload)

    frame_summaries = {}
    for frame, result in cam_results.items():
        mean_delta = float(result.predicted_delta_y.mean())
        frame_summaries[str(frame)] = {
            "current_ee_y": result.current_ee_y,
            "predicted_delta_y_mean": mean_delta,
            "predicted_delta_y_final": float(result.predicted_delta_y[-1]),
            "behavior_target_sign": 1 if mean_delta >= 0.0 else -1,
            "correct_raw_max": result.correct_raw_max,
            "wrong_raw_max": result.wrong_raw_max,
            "gradient_abs_mean": result.gradient_abs_mean,
            "instantaneous_success": observations[frame]["instantaneous_success"],
            "stable_success": observations[frame]["stable_success"],
            "post_success_continuation": (
                normal_success_step is not None and frame > normal_success_step
            ),
        }
    summary = {
        "label": spec["label"],
        "representation": spec["representation"],
        "checkpoint": str(checkpoint),
        "environment": environment,
        "normal_evaluation_success": normal_success_step is not None,
        "normal_success_step": normal_success_step,
        "post_success_continuation_end_frame": (
            300
            if normal_success_step is not None and normal_success_step < 300
            else None
        ),
        "final_instantaneous_success": bool(instantaneous_success[-1]),
        "final_can_position": final_can_position,
        "target_position": target_position,
        "final_position_error_m": float(
            np.linalg.norm(final_can_position - target_position)
        ),
        "initial_can_y": float(can_positions[0][1]),
        "final_can_y": float(final_can_position[1]),
        "target_y": float(target_position[1]),
        "max_repeated_inference_difference": max(prediction_replay_errors, default=0.0),
        "frames": frame_summaries,
    }
    del runner
    gc.collect()
    torch.cuda.empty_cache()
    return {
        "summary": summary,
        "cam_results": cam_results,
        "can_positions": np.asarray(can_positions),
        "initial_rgb": initial_rgb,
        "final_rgb": final_rgb,
    }


def _overlay(image: np.ndarray, heatmap: np.ndarray, alpha: float = 0.45):
    import matplotlib.pyplot as plt

    color = plt.get_cmap("turbo")(heatmap)[..., :3]
    return np.clip(
        (1.0 - alpha) * image.astype(np.float32) / 255.0 + alpha * color,
        0.0,
        1.0,
    )


def _comparison_figure(results: dict, output_dir: Path, behavior: bool):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(4, 2, figsize=(12, 14), squeeze=False)
    for row, (key, spec) in enumerate(POLICIES.items()):
        result = results[key]["cam_results"][DECISION_FRAME]
        mean_delta = float(result.predicted_delta_y.mean())
        sign = 1 if mean_delta >= 0.0 else -1
        heatmaps = (
            result.correct_heatmaps
            if not behavior or sign > 0
            else result.wrong_heatmaps
        )
        for column, camera in enumerate(CAMERAS):
            axes[row, column].imshow(_overlay(result.images[column], heatmaps[column]))
            axes[row, column].axis("off")
            direction = "+Y" if not behavior else f"{sign:+d}Y"
            axes[row, column].set_title(
                f"{spec['label']} | {camera} | target {direction}\n"
                f"predicted mean ΔY={mean_delta:+.4f} m",
                fontsize=10,
            )
    definition = (
        "Behavior-targeted: sign(mean predicted EE-Y displacement over future steps 1..89)"
        if behavior
        else "Correct-target comparison: identical world-frame +Y target for every policy"
    )
    figure.suptitle(
        f"Closed-loop ACT signed EE-Y Grad-CAM | frame {DECISION_FRAME}\n{definition}",
        fontsize=14,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
    filename = (
        "comparison_behavior_targeted_frame_0120.png"
        if behavior
        else "comparison_correct_plus_y_frame_0120.png"
    )
    figure.savefig(output_dir / filename, dpi=180, bbox_inches="tight")
    plt.close(figure)


def _timeline_figures(results: dict, output_dir: Path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    timeline_dir = output_dir / "timelines"
    timeline_dir.mkdir()
    for key, spec in POLICIES.items():
        success_step = results[key]["summary"]["normal_success_step"]
        for camera_index, camera in enumerate(CAMERAS):
            for zoom, frames in (
                (False, FRAMES),
                (True, tuple(frame for frame in FRAMES if frame >= 180)),
            ):
                figure, axes = plt.subplots(
                    len(frames), 3, figsize=(12, 3.05 * len(frames)), squeeze=False
                )
                for row, frame in enumerate(frames):
                    result = results[key]["cam_results"][frame]
                    panels = (
                        result.images[camera_index],
                        _overlay(
                            result.images[camera_index],
                            result.correct_heatmaps[camera_index],
                        ),
                        _overlay(
                            result.images[camera_index],
                            result.wrong_heatmaps[camera_index],
                        ),
                    )
                    post = success_step is not None and frame > success_step
                    for column, panel in enumerate(panels):
                        axes[row, column].imshow(panel)
                        axes[row, column].set_xticks([])
                        axes[row, column].set_yticks([])
                        if post:
                            for spine in axes[row, column].spines.values():
                                spine.set_edgecolor("crimson")
                                spine.set_linewidth(3.0)
                    suffix = " | POST-SUCCESS CONTINUATION" if post else ""
                    axes[row, 0].set_ylabel(
                        f"frame {frame}{suffix}",
                        fontsize=9,
                        color="crimson" if post else "black",
                    )
                    if row == 0:
                        for column, title in enumerate(
                            ("RGB", "+Y Grad-CAM", "-Y Grad-CAM")
                        ):
                            axes[row, column].set_title(title)
                range_label = "frames 180-300 (zoom)" if zoom else "frames 100-300"
                normal = (
                    "no stable success by frame 300"
                    if success_step is None
                    else f"normal evaluation success step={success_step}; red=post-success"
                )
                figure.suptitle(
                    f"{spec['label']} | {camera} | signed world EE-Y timeline | "
                    f"{range_label}\n{normal}",
                    fontsize=13,
                )
                figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
                suffix = "zoom_0180_0300" if zoom else "full_0100_0300"
                figure.savefig(
                    timeline_dir / f"{key}_{camera}_{suffix}.png",
                    dpi=145,
                    bbox_inches="tight",
                )
                plt.close(figure)


def _behavior_figures(results: dict, output_dir: Path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    behavior_dir = output_dir / "behavior"
    behavior_dir.mkdir()
    for key, spec in POLICIES.items():
        item = results[key]
        summary = item["summary"]
        prediction = item["cam_results"][DECISION_FRAME]
        can_positions = item["can_positions"]
        figure = plt.figure(figsize=(15, 12))
        grid = figure.add_gridspec(3, 4, height_ratios=(1.05, 1, 1))
        trajectory_axis = figure.add_subplot(grid[0, :2])
        prediction_axis = figure.add_subplot(grid[0, 2:])
        trajectory_axis.plot(np.arange(301), can_positions[:, 1], label="can world Y")
        trajectory_axis.axhline(
            summary["target_y"], color="tab:blue", linestyle="--", label="correct box Y"
        )
        if summary["normal_success_step"] is not None:
            trajectory_axis.axvline(
                summary["normal_success_step"], color="tab:green", linestyle=":"
            )
        trajectory_axis.axvline(DECISION_FRAME, color="black", linestyle=":")
        trajectory_axis.set(
            xlabel="closed-loop frame",
            ylabel="world Y (m)",
            title="Actual can trajectory",
        )
        trajectory_axis.grid(alpha=0.25)
        trajectory_axis.legend(fontsize=8)
        prediction_axis.plot(
            np.arange(1, 90), prediction.predicted_ee_y, label="predicted EE Y"
        )
        prediction_axis.axhline(
            prediction.current_ee_y, color="black", linestyle=":", label="current EE Y"
        )
        prediction_axis.set(
            xlabel="future ACT chunk step (1..89)",
            ylabel="world EE Y (m)",
            title=f"89-step EE-Y prediction at frame {DECISION_FRAME}",
        )
        prediction_axis.grid(alpha=0.25)
        prediction_axis.legend(fontsize=8)

        for camera_index, camera in enumerate(CAMERAS):
            axis = figure.add_subplot(grid[1, camera_index * 2 : camera_index * 2 + 2])
            axis.imshow(prediction.images[camera_index])
            axis.set_title(f"Decision frame {DECISION_FRAME} RGB | {camera}")
            axis.axis("off")
            axis = figure.add_subplot(grid[2, camera_index * 2 : camera_index * 2 + 2])
            axis.imshow(item["final_rgb"][camera_index])
            axis.set_title(f"Final frame 300 RGB | {camera}")
            axis.axis("off")
        final_position = np.asarray(summary["final_can_position"])
        target_position = np.asarray(summary["target_position"])
        status = (
            f"success={summary['normal_evaluation_success']} | "
            f"normal success step={summary['normal_success_step']} | "
            f"final instantaneous success={summary['final_instantaneous_success']}\n"
            f"final can xyz={np.array2string(final_position, precision=4)} m | "
            f"correct box xyz={np.array2string(target_position, precision=4)} m | "
            f"error={summary['final_position_error_m']:.4f} m"
        )
        figure.suptitle(
            f"{spec['label']} | one closed-loop rollout, seed {SEED}, PTE/F=0\n{status}",
            fontsize=13,
        )
        figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.93))
        figure.savefig(
            behavior_dir / f"{key}_complete_behavior.png",
            dpi=170,
            bbox_inches="tight",
        )
        plt.close(figure)

    figure, axis = plt.subplots(figsize=(11, 5.5))
    for key, spec in POLICIES.items():
        axis.plot(
            np.arange(301), results[key]["can_positions"][:, 1], label=spec["label"]
        )
    target_y = results["d097_joint"]["summary"]["target_y"]
    axis.axhline(target_y, color="black", linestyle="--", label="correct blue box Y")
    axis.axvline(DECISION_FRAME, color="black", linestyle=":", alpha=0.7)
    axis.set(
        xlabel="closed-loop frame",
        ylabel="can world Y (m)",
        title="Actual can world-Y trajectories | identical initial environment state",
    )
    axis.grid(alpha=0.25)
    axis.legend(ncol=3, fontsize=9)
    figure.tight_layout()
    figure.savefig(
        behavior_dir / "all_policy_can_y_trajectories.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(figure)


def _validate_npz(output_dir: Path) -> dict:
    files = sorted(output_dir.rglob("*.npz"))
    failures = []
    numeric_arrays = 0
    for path in files:
        with np.load(path, allow_pickle=False) as values:
            for name in values.files:
                value = values[name]
                if np.issubdtype(value.dtype, np.number):
                    numeric_arrays += 1
                    if not np.all(np.isfinite(value)):
                        failures.append(f"{path}:{name}")
    if failures:
        raise ValueError(f"non-finite NPZ values: {failures}")
    return {
        "npz_file_count": len(files),
        "numeric_array_count": numeric_arrays,
        "all_numeric_values_finite": True,
    }


def _write_readme(output_dir: Path, summary: dict):
    lines = [
        "# Closed-loop ACT signed world-EE-Y Grad-CAM",
        "",
        f"- Environment seed: `{SEED}`",
        "- Task: `can_color_sort`",
        "- PTE/F: `0`",
        "- Cameras: `cam_high`, `cam_right_wrist`",
        f"- Analysis frames: `{list(FRAMES)}`",
        "- Rollout source: actual MuJoCo closed-loop observations and policy actions; no expert trajectory is used for Grad-CAM.",
        "- The HDF5 training set is read only by the preflight audit to recompute normalization statistics.",
        "",
        "## Target definition",
        "",
        "For every selected observation, the scalar target is the mean predicted world-frame EE-Y displacement over future ACT chunk steps 1..89. Step 0 is excluded, so the plotted prediction has exactly 89 future points.",
        "",
        "For Joint policies, each predicted right-arm joint target is linearly projected with the current MuJoCo right-EE translational Jacobian row `J_y`. Gradients are taken with respect to normalized policy output, and the linear weights include `action_std`, so the derivative is in physical joint units. The same current Jacobian is used for every future step.",
        "",
        "For Task policies, output index 1 is the absolute world-frame EE Y coordinate. Its normalization `action_std[1]` is included by de-normalizing before forming the target.",
        "",
        "The +Y map uses the positive target. The -Y map uses its exact negative. Each map is the ReLU of its signed Grad-CAM evidence and is independently normalized per camera for visualization. Raw maxima are stored so normalized maps are not mistaken for equal absolute strength.",
        "",
        "The correct-target comparison always selects +Y for all policies. The behavior-targeted comparison selects the sign of the policy's mean predicted displacement, without using success, can outcome, or target-box coordinates.",
        "",
        "## Closed-loop and success handling",
        "",
        "Each policy starts from a separately constructed environment reset with the same seed. The initial full physical state must match exactly across all four rollouts. Initial RGB is compared separately with a one-level uint8 rasterization tolerance because independent MuJoCo GPU render contexts can differ at edge pixels by one quantization level. Policies execute their own actions through the normal Joint controller or Task IK bridge. Stable success uses 10 consecutive instantaneous-success observations. Rollout continues through observation frame 300; frames strictly after the normal stable-success step are labeled `POST-SUCCESS CONTINUATION`.",
        "",
        "## Files",
        "",
        "- `comparison_correct_plus_y_frame_0120.png`: 4x2 comparison with the identical correct +Y target.",
        "- `comparison_behavior_targeted_frame_0120.png`: 4x2 comparison explaining each policy's predicted direction.",
        "- `timelines/`: full 100-300 and zoomed 180-300 RGB/+Y/-Y timelines for every policy and camera.",
        "- `behavior/`: complete behavior figures and the cross-policy can-Y plot.",
        "- `<policy>/frames/frame_*.npz`: frame RGB, both heatmaps, raw strengths, gradients, 89-step predictions, geometry, and success state.",
        "- `<policy>/rollout.npz`: closed-loop trajectory, actions, initial/final RGB, and success series.",
        "- `summary.json`: numerical results, hashes, preflight checks, and finite-value validation.",
        "",
        "## Limitations",
        "",
        "Grad-CAM is a spatial sensitivity visualization at the selected feature layer. It does not prove causal color recognition, object identity, or that highlighted pixels are necessary for the action. Joint-space EE-Y uses a local Jacobian linearization and therefore is not an exact nonlinear FK trajectory for large joint changes. Per-map normalization supports spatial comparison but not magnitude comparison; use the saved raw maxima and gradient means for strength. Behavior after normal success is counterfactual continuation of the same policy and is not part of the normal evaluation score.",
        "",
        "## Key rollout results",
        "",
    ]
    for key, spec in POLICIES.items():
        value = summary["policies"][key]
        lines.append(
            f"- {spec['label']}: success=`{value['normal_evaluation_success']}`, "
            f"normal step=`{value['normal_success_step']}`, final can Y=`{value['final_can_y']:.5f}` m, "
            f"target Y=`{value['target_y']:.5f}` m, final error=`{value['final_position_error_m']:.5f}` m."
        )
    output_dir.joinpath("README.md").write_text("\n".join(lines) + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(
            "outputs/analysis/closed_loop_gradcam_ee_y_seed195958_20260825_r2"
        ),
    )
    args = parser.parse_args(argv)
    repo = Path(__file__).resolve().parents[1]
    output_dir = (repo / args.output_dir).resolve()
    if output_dir.exists():
        raise FileExistsError(
            f"refusing to overwrite existing analysis directory: {output_dir}"
        )
    output_dir.mkdir(parents=True)
    print(f"PRELIGHT -> {output_dir}", flush=True)
    preflight = _preflight(repo)
    results = {}
    for key, spec in POLICIES.items():
        print(f"ROLLOUT {spec['label']}", flush=True)
        results[key] = _rollout_policy(repo, output_dir, key, spec)

    initial_hashes = {
        key: value["summary"]["environment"]["initial_state_sha256"]
        for key, value in results.items()
    }
    same_initial_state = len(set(initial_hashes.values())) == 1
    if not same_initial_state:
        raise ValueError(f"initial states differ: {initial_hashes}")
    reference_rgb = results["d097_joint"]["initial_rgb"].astype(np.int16)
    initial_rgb_differences = {
        key: {
            "maximum_absolute_uint8_difference": int(
                np.max(np.abs(value["initial_rgb"].astype(np.int16) - reference_rgb))
            ),
            "different_value_count": int(
                np.count_nonzero(value["initial_rgb"].astype(np.int16) != reference_rgb)
            ),
        }
        for key, value in results.items()
    }
    initial_rgb_within_raster_tolerance = all(
        value["maximum_absolute_uint8_difference"] <= 1
        for value in initial_rgb_differences.values()
    )
    if not initial_rgb_within_raster_tolerance:
        raise ValueError(
            f"initial RGB differs beyond tolerance: {initial_rgb_differences}"
        )
    print("FIGURES", flush=True)
    _comparison_figure(results, output_dir, behavior=False)
    _comparison_figure(results, output_dir, behavior=True)
    _timeline_figures(results, output_dir)
    _behavior_figures(results, output_dir)
    finite_validation = _validate_npz(output_dir)
    summary = {
        "analysis": {
            "task": "can_color_sort",
            "environment_seed": SEED,
            "pte_f": 0,
            "object_variant": "blue",
            "bin_layout": "swapped; blue world +Y, red world -Y",
            "camera_names": list(CAMERAS),
            "frames": list(FRAMES),
            "decision_frame": DECISION_FRAME,
            "rollout_final_frame": 300,
            "stable_success_steps": STABLE_SUCCESS_STEPS,
            "task_ik_speed_scale": TASK_IK_SPEED_SCALE,
            "target": "mean world-frame EE-Y displacement, ACT future steps 1..89",
            "feature_layer": "policy.image_projection",
            "cheating_controls": {
                "expert_trajectory_used_for_rollout_or_gradcam": False,
                "success_used_in_gradcam_target": False,
                "outcome_based_target_selection": False,
                "environment_state_injection": False,
                "expert_or_correct_action_injection": False,
            },
        },
        "preflight": preflight,
        "same_initial_state": same_initial_state,
        "initial_state_hashes": initial_hashes,
        "initial_rgb_reference": "d097_joint",
        "initial_rgb_differences": initial_rgb_differences,
        "initial_rgb_within_one_uint8_level": initial_rgb_within_raster_tolerance,
        "finite_validation": finite_validation,
        "policies": {key: value["summary"] for key, value in results.items()},
    }
    (output_dir / "summary.json").write_text(
        json.dumps(_json_value(summary), indent=2) + "\n"
    )
    _write_readme(output_dir, _json_value(summary))
    print(f"DONE {output_dir}", flush=True)


if __name__ == "__main__":
    main()
