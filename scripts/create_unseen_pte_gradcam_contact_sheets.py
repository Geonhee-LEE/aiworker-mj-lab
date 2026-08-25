#!/usr/bin/env python3
"""Create outcome-independent Grad-CAM sheets for the orange/blue PTE sweep."""

from __future__ import annotations

import argparse
import gc
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import torch
from analyze_closed_loop_ee_y_gradcam import (
    CAMERAS,
    POLICIES,
    STABLE_SUCCESS_STEPS,
    TASK_IK_SPEED_SCALE,
    _array_hash,
    _overlay,
    _signed_gradcam,
)

from ffw_sh5_grasp.control import whole_body
from ffw_sh5_grasp.imitation.data.schema import ARM_JOINTS
from ffw_sh5_grasp.imitation.runtime.runner import ACTPolicyRunner
from ffw_sh5_grasp.imitation.runtime.task_space import task_action_to_joint
from ffw_sh5_grasp.imitation.simulation.environment import AIWorkerMujocoEnv

plt.switch_backend("Agg")

FRAMES = (0, 100, 200, 300)
COLORS = ("orange", "blue")
REPRESENTATIONS = ("joint", "task")
DATA_COUNTS = (97, 150)


def _read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _read_trials(path: Path):
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def _select_best_f(rows):
    """Maximize success, then minimize penalized time, then prefer lower F."""
    selected = {}
    for policy in POLICIES:
        candidates = [row for row in rows if row["policy"] == policy]
        if not candidates:
            raise ValueError(f"missing heatmap rows for {policy}")
        best = min(
            candidates,
            key=lambda row: (
                -float(row["success_rate"]),
                float(row["mean_penalized_time_s"]),
                int(row["f_steps"]),
            ),
        )
        selected[policy] = int(best["f_steps"])
    return selected


def _load_protocol(evaluation_dir: Path, heatmap_summary: Path):
    heatmap = _read_json(heatmap_summary)
    protocol = heatmap["protocol"]
    if protocol.get("object_variants") != list(COLORS):
        raise ValueError(
            f"expected orange/blue evaluation, got {protocol.get('object_variants')}"
        )
    selected_f = _select_best_f(heatmap["rows"])
    trials_by_policy = {}
    for policy, f_steps in selected_f.items():
        condition = evaluation_dir / policy / f"f_{f_steps:03d}"
        config = _read_json(condition / "evaluation_config.json")
        trials = _read_trials(condition / "trials.jsonl")
        if config.get("object_variants") != list(COLORS):
            raise ValueError(f"variant mismatch for {policy} F={f_steps}")
        expected_seeds = list(
            range(int(protocol["seed_first"]), int(protocol["seed_last"]) + 1)
        )
        actual_seeds = [int(trial["seed"]) for trial in trials]
        if actual_seeds != expected_seeds:
            raise ValueError(f"seed mismatch for {policy} F={f_steps}")
        trials_by_policy[policy] = {int(row["seed"]): row for row in trials}

    # Select the first occurrence of each color without consulting success.
    reference = trials_by_policy["d097_joint"]
    display_seeds = {
        color: min(
            seed
            for seed, trial in reference.items()
            if trial["object_variant"] == color
        )
        for color in COLORS
    }
    for color, seed in display_seeds.items():
        for policy, trials in trials_by_policy.items():
            if trials[seed]["object_variant"] != color:
                raise ValueError(f"seed {seed} color differs for {policy}")
    return heatmap, selected_f, trials_by_policy, display_seeds


def _save_frame(path, result, snapshot, rollout):
    np.savez_compressed(
        path,
        camera_names=np.asarray(CAMERAS),
        rgb_images=result.images,
        positive_y_heatmaps=result.correct_heatmaps,
        negative_y_heatmaps=result.wrong_heatmaps,
        correct_heatmaps=result.correct_heatmaps,
        wrong_heatmaps=result.wrong_heatmaps,
        positive_y_raw_max=result.correct_raw_max,
        negative_y_raw_max=result.wrong_raw_max,
        raw_heatmap_maxima=np.stack((result.correct_raw_max, result.wrong_raw_max)),
        gradient_absolute_mean=result.gradient_abs_mean,
        current_ee_y=np.asarray(result.current_ee_y, dtype=np.float32),
        predicted_ee_y=result.predicted_ee_y,
        predicted_delta_y=result.predicted_delta_y,
        predicted_chunk=result.predicted_chunk,
        behavior_target_sign=np.asarray(snapshot["behavior_sign"], dtype=np.int8),
        can_position=snapshot["can_position"].astype(np.float32),
        target_position=snapshot["target_position"].astype(np.float32),
        instantaneous_success=np.asarray(snapshot["instantaneous_success"]),
        stable_success=np.asarray(snapshot["stable_success"]),
        post_success_continuation=np.asarray(
            rollout["normal_success_step"] is not None
            and snapshot["frame"] > rollout["normal_success_step"]
        ),
        frame=np.asarray(snapshot["frame"], dtype=np.int64),
        seed=np.asarray(rollout["seed"], dtype=np.int64),
        pte_steps=np.asarray(rollout["pte_steps"], dtype=np.int64),
        object_variant=np.asarray(rollout["object_variant"]),
        target_label=np.asarray(rollout["target_label"]),
        bin_colors_swapped=np.asarray(rollout["bin_colors_swapped"]),
        normal_success_step=np.asarray(
            -1
            if rollout["normal_success_step"] is None
            else rollout["normal_success_step"],
            dtype=np.int64,
        ),
    )


def _rollout(repo, output_dir, policy, f_steps, seed, expected_trial):
    spec = POLICIES[policy]
    runner = ACTPolicyRunner(
        (repo / spec["checkpoint"]).resolve(),
        device="auto",
        representation=spec["representation"],
        proleptic_steps=f_steps,
    )
    results = {}
    snapshots = {}
    success_streak = 0
    stable_success = False
    normal_success_step = None
    with AIWorkerMujocoEnv(
        render_images=True,
        camera_names=CAMERAS,
        task_name="can_color_sort",
        object_variants=COLORS,
        randomize_bin_colors=True,
    ) as env:
        solver = None
        if runner.representation == "task":
            solver = whole_body.WholeBodyIK(
                env.model,
                {"r": "grasp_target_r", "l": "grasp_target_l"},
                ARM_JOINTS,
            )
        observation = env.reset(seed=seed)
        runner.reset()
        if solver is not None:
            solver.rebase(env.data)
        initial_state_hash = _array_hash(
            observation["debug"]["full_qpos"],
            observation["debug"]["full_qvel"],
            observation["debug"]["task_object_pose"],
            observation["debug"]["target_position"],
        )
        initial_rgb_hash = _array_hash(
            *[observation["images"][camera] for camera in CAMERAS]
        )
        for frame in range(301):
            if frame in FRAMES:
                result = _signed_gradcam(runner, observation, env)
                results[frame] = result
                snapshots[frame] = {
                    "frame": frame,
                    "can_position": np.asarray(
                        observation["debug"]["task_object_pose"][:3]
                    ).copy(),
                    "target_position": np.asarray(
                        observation["debug"]["target_position"]
                    ).copy(),
                    "instantaneous_success": bool(observation["task"]["success"]),
                    "stable_success": stable_success,
                    "behavior_sign": (
                        1 if float(result.predicted_delta_y.mean()) >= 0.0 else -1
                    ),
                }
            if frame == 300:
                break
            action, _policy_info = runner.get_action(observation)
            if runner.representation == "task":
                action, _diagnostics = task_action_to_joint(
                    env,
                    solver,
                    action,
                    speed_scale=TASK_IK_SPEED_SCALE,
                )
            observation = env.step(env.prepare_action(action))
            success_streak = success_streak + 1 if observation["task"]["success"] else 0
            if not stable_success and success_streak >= STABLE_SUCCESS_STEPS:
                stable_success = True
                normal_success_step = frame + 1

        rollout = {
            "policy": policy,
            "label": spec["label"],
            "representation": runner.representation,
            "pte_steps": f_steps,
            "seed": seed,
            "object_variant": observation["task"]["object_variant"],
            "target_label": observation["task"]["target_label"],
            "bin_colors_swapped": bool(env.task.bin_colors_swapped),
            "bin_color_layout": env.task.bin_color_layout,
            "initial_state_sha256": initial_state_hash,
            "initial_rgb_sha256": initial_rgb_hash,
            "normal_success": normal_success_step is not None,
            "normal_success_step": normal_success_step,
            "expected_evaluation_success": bool(expected_trial["success"]),
            "expected_evaluation_step": int(expected_trial["steps"]),
        }
    if rollout["object_variant"] != expected_trial["object_variant"]:
        raise ValueError(f"rollout color mismatch for {policy}")
    if rollout["bin_colors_swapped"] != bool(expected_trial["bin_colors_swapped"]):
        raise ValueError(f"rollout layout mismatch for {policy}")
    if rollout["normal_success"] != rollout["expected_evaluation_success"]:
        raise ValueError(f"success replay mismatch for {policy}")
    rollout["success_step_replay_difference"] = (
        rollout["normal_success_step"] - rollout["expected_evaluation_step"]
        if rollout["normal_success"]
        else None
    )

    frame_dir = output_dir / policy / "frames"
    frame_dir.mkdir(parents=True)
    for frame in FRAMES:
        _save_frame(
            frame_dir / f"frame_{frame:04d}.npz",
            results[frame],
            snapshots[frame],
            rollout,
        )
    del runner
    gc.collect()
    torch.cuda.empty_cache()
    return {"summary": rollout, "results": results, "snapshots": snapshots}


def _panel_label(axis, text):
    axis.text(
        0.02,
        0.95,
        text,
        transform=axis.transAxes,
        color="white",
        fontsize=7.4,
        fontweight="bold",
        va="top",
        bbox={"facecolor": "black", "alpha": 0.78, "pad": 3},
    )


def _target_label(axis, sign):
    axis.text(
        0.98,
        0.95,
        f"target {'+' if sign > 0 else '-'}Y",
        transform=axis.transAxes,
        color="white",
        fontsize=7.2,
        fontweight="bold",
        ha="right",
        va="top",
        bbox={"facecolor": "black", "alpha": 0.68, "pad": 2.5},
    )


def _status_label(axis, frame, normal_success_step):
    if normal_success_step is None:
        text, color = "evaluation: failed", "#B91C1C"
    elif frame > normal_success_step:
        text, color = "post-success continuation", "#15803D"
    else:
        return
    axis.text(
        0.98,
        0.04,
        text,
        transform=axis.transAxes,
        color="white",
        fontsize=7.5,
        fontweight="bold",
        ha="right",
        va="bottom",
        bbox={"facecolor": color, "alpha": 0.82, "pad": 2.5},
    )


def _save_sheet(output_dir, color, seed, rollouts, mode):
    figure, axes = plt.subplots(4, 8, figsize=(25.5, 11.2), squeeze=False)
    for group, representation in enumerate(REPRESENTATIONS):
        for data_index, data_count in enumerate(DATA_COUNTS):
            policy = f"d{data_count:03d}_{representation}"
            rollout = rollouts[policy]
            for camera_index, camera in enumerate(CAMERAS):
                row = data_index * len(CAMERAS) + camera_index
                for frame_index, frame in enumerate(FRAMES):
                    column = group * len(FRAMES) + frame_index
                    result = rollout["results"][frame]
                    snapshot = rollout["snapshots"][frame]
                    if mode == "fixed_plus_y":
                        heatmaps, sign = result.correct_heatmaps, 1
                    elif snapshot["behavior_sign"] > 0:
                        heatmaps, sign = result.correct_heatmaps, 1
                    else:
                        heatmaps, sign = result.wrong_heatmaps, -1
                    axes[row, column].imshow(
                        _overlay(result.images[camera_index], heatmaps[camera_index])
                    )
                    axes[row, column].axis("off")
                    if row == 0:
                        axes[row, column].set_title(f"frame {frame}", fontsize=11)
                    if frame_index == 0:
                        short_camera = camera.removeprefix("cam_")
                        _panel_label(
                            axes[row, column],
                            f"D{data_count} {representation.title()} | "
                            f"{short_camera} | F{rollout['summary']['pte_steps']}",
                        )
                    _target_label(axes[row, column], sign)
                    _status_label(
                        axes[row, column],
                        frame,
                        rollout["summary"]["normal_success_step"],
                    )

    definition = (
        "Fixed-target comparison: identical world-frame +Y sensitivity "
        "(not a correctness claim)"
        if mode == "fixed_plus_y"
        else "Behavior-targeted: frame-wise sign of mean predicted world EE-Y"
    )
    layout = (
        "swapped"
        if next(iter(rollouts.values()))["summary"]["bin_colors_swapped"]
        else "default"
    )
    figure.suptitle(
        "Orange/Blue PTE sweep — closed-loop signed EE-Y Grad-CAM\n"
        f"{definition} | {color} can | seed {seed} | {layout} bins | "
        "F chosen by success rate, then penalized time",
        fontsize=15,
        fontweight="bold",
        y=0.992,
    )
    figure.text(
        0.255,
        0.91,
        "Joint-space Grad-CAM",
        ha="center",
        fontsize=14,
        color="#1F5AA6",
        fontweight="bold",
    )
    figure.text(
        0.745,
        0.91,
        "Task-space Grad-CAM",
        ha="center",
        fontsize=14,
        color="#1F5AA6",
        fontweight="bold",
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.885), pad=0.7)
    path = output_dir / f"{color}_{mode}_gradcam_frames_0000_0100_0200_0300.png"
    figure.savefig(path, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(figure)
    return path


def _finite_npz(output_dir):
    checked = 0
    for path in output_dir.rglob("*.npz"):
        with np.load(path) as values:
            for name in values.files:
                value = values[name]
                if (
                    np.issubdtype(value.dtype, np.number)
                    and not np.isfinite(value).all()
                ):
                    raise ValueError(f"non-finite {name} in {path}")
        checked += 1
    return checked


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--evaluation-dir",
        type=Path,
        default=Path(
            "outputs/evaluation/"
            "can_color_sort_d097_unseen_orange_blue_seed195958_n100_20260825"
        ),
    )
    parser.add_argument(
        "--heatmap-summary",
        type=Path,
        default=Path(
            "outputs/analysis/"
            "pte_success_time_heatmaps_d097_d150_unseen_orange_blue_"
            "seed195958_20260825/summary.json"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/analysis/unseen_orange_blue_best_pte_gradcam_20260826"),
    )
    args = parser.parse_args(argv)
    repo = Path(__file__).resolve().parents[1]
    evaluation_dir = (repo / args.evaluation_dir).resolve()
    heatmap_summary = (repo / args.heatmap_summary).resolve()
    output_dir = (repo / args.output_dir).resolve()
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite {output_dir}")
    output_dir.mkdir(parents=True)

    heatmap, selected_f, trials, display_seeds = _load_protocol(
        evaluation_dir, heatmap_summary
    )
    color_outputs = {}
    initial_hashes = {}
    for color in COLORS:
        seed = display_seeds[color]
        color_dir = output_dir / color
        color_dir.mkdir()
        print(f"{color.upper()} seed={seed}", flush=True)
        rollouts = {}
        for policy in POLICIES:
            print(f"  {POLICIES[policy]['label']} F={selected_f[policy]}", flush=True)
            rollouts[policy] = _rollout(
                repo,
                color_dir,
                policy,
                selected_f[policy],
                seed,
                trials[policy][seed],
            )
        hashes = {
            policy: value["summary"]["initial_state_sha256"]
            for policy, value in rollouts.items()
        }
        if len(set(hashes.values())) != 1:
            raise ValueError(f"initial states differ for {color}: {hashes}")
        initial_hashes[color] = hashes
        figures = [
            _save_sheet(color_dir, color, seed, rollouts, mode)
            for mode in ("fixed_plus_y", "behavior_targeted")
        ]
        color_outputs[color] = {
            "seed": seed,
            "figures": [str(path.resolve()) for path in figures],
            "policies": {
                policy: value["summary"] for policy, value in rollouts.items()
            },
        }

    finite_files = _finite_npz(output_dir)
    summary = {
        "source_evaluation": str(evaluation_dir),
        "source_heatmap_summary": str(heatmap_summary),
        "selection": {
            "pte_rule": (
                "highest 100-episode success rate; ties use lowest mean penalized "
                "time; remaining ties use lower F"
            ),
            "selected_f": selected_f,
            "display_seed_rule": (
                "lowest protocol seed producing each color; success was not consulted"
            ),
            "display_seeds": display_seeds,
        },
        "gradcam_target": (
            "mean predicted world-frame EE-Y displacement over ACT steps 1..89; "
            "both +Y and -Y maps saved"
        ),
        "success_used_in_gradcam_target": False,
        "outcome_used_for_display_seed": False,
        "heatmap_protocol": heatmap["protocol"],
        "initial_state_hashes": initial_hashes,
        "finite_npz_files": finite_files,
        "colors": color_outputs,
        "limitation": (
            "Grad-CAM is a local gradient-based association map and does not prove "
            "causal color recognition or pixel necessity."
        ),
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    (output_dir / "README.md").write_text(
        "# Orange/Blue best-PTE closed-loop Grad-CAM\n\n"
        "F is selected from the 100-episode heatmap by maximum success rate, then "
        "minimum penalized time. The first seed producing each color is used without "
        "consulting success. Each rollout continues through frame 300; green labels "
        "mark post-success continuation and red labels mark normal evaluation failure.\n\n"
        "The fixed +Y sheet is a direction-controlled comparison, not a claim that +Y "
        "is always the correct bin direction. The behavior-targeted sheet uses only the "
        "sign of the policy's predicted EE-Y displacement. Both signed maps and raw "
        "magnitudes are stored in NPZ files. Grad-CAM does not prove causal color "
        "recognition.\n",
        encoding="utf-8",
    )
    print(output_dir, flush=True)


if __name__ == "__main__":
    main()
