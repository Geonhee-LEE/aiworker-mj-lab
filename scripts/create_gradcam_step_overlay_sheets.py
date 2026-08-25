"""Apply saved signed Grad-CAM maps to frame 0/100/200/300 sheets."""

import gc
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import torch
from analyze_closed_loop_ee_y_gradcam import (
    CAMERAS,
    POLICIES,
    SEED,
    _signed_gradcam,
)

from ffw_sh5_grasp.imitation.runtime.runner import ACTPolicyRunner
from ffw_sh5_grasp.imitation.simulation.environment import AIWorkerMujocoEnv

plt.switch_backend("Agg")

ROOT = Path("outputs/analysis/closed_loop_gradcam_ee_y_seed195958_20260825_r2")
FRAMES = (0, 100, 200, 300)
DISPLAY_POLICIES = (
    ("d097_task", "D97 Task"),
    ("d097_joint", "D97 Joint"),
    ("d150_task", "D150 Task"),
    ("d150_joint", "D150 Joint"),
)


def overlay(image, heatmap, alpha=0.45):
    color = plt.get_cmap("turbo")(heatmap)[..., :3]
    return np.clip(
        (1.0 - alpha) * image.astype(np.float32) / 255.0 + alpha * color,
        0.0,
        1.0,
    )


def compute_frame_zero(repo, output_dir, policy):
    spec = POLICIES[policy]
    checkpoint = (repo / spec["checkpoint"]).resolve()
    policy_dir = ROOT / policy
    with np.load(policy_dir / "rollout.npz") as rollout:
        initial_images = rollout["initial_rgb_images"].copy()
    runner = ACTPolicyRunner(
        checkpoint,
        device="auto",
        representation=spec["representation"],
        proleptic_steps=0,
    )
    with AIWorkerMujocoEnv(
        render_images=True,
        camera_names=CAMERAS,
        task_name="can_color_sort",
        randomize_bin_colors=True,
    ) as env:
        observation = env.reset(seed=SEED)
        observation["images"] = {
            camera: initial_images[index] for index, camera in enumerate(CAMERAS)
        }
        result = _signed_gradcam(runner, observation, env)
    frame_zero_dir = output_dir / "frame_0000_gradcam"
    frame_zero_dir.mkdir(exist_ok=True)
    np.savez_compressed(
        frame_zero_dir / f"{policy}_frame_0000.npz",
        camera_names=np.asarray(CAMERAS),
        rgb_images=result.images,
        correct_heatmaps=result.correct_heatmaps,
        wrong_heatmaps=result.wrong_heatmaps,
        correct_heatmap_raw_max=result.correct_raw_max,
        wrong_heatmap_raw_max=result.wrong_raw_max,
        gradient_absolute_mean=result.gradient_abs_mean,
        current_ee_y=np.asarray(result.current_ee_y, dtype=np.float32),
        predicted_ee_y=result.predicted_ee_y,
        predicted_delta_y=result.predicted_delta_y,
        behavior_target_sign=np.asarray(
            1 if float(result.predicted_delta_y.mean()) >= 0.0 else -1,
            dtype=np.int8,
        ),
    )
    del runner
    gc.collect()
    torch.cuda.empty_cache()


def load_policy_frames(output_dir, policy):
    values = {}
    for frame in FRAMES:
        path = (
            output_dir / "frame_0000_gradcam" / f"{policy}_frame_0000.npz"
            if frame == 0
            else ROOT / policy / "frames" / f"frame_{frame:04d}.npz"
        )
        with np.load(path) as data:
            values[frame] = {
                "images": data["rgb_images"].copy(),
                "correct": data["correct_heatmaps"].copy(),
                "wrong": data["wrong_heatmaps"].copy(),
                "sign": int(data["behavior_target_sign"]),
            }
    return values


def selected_heatmap(frame_values, mode):
    if mode == "correct_plus_y" or frame_values["sign"] > 0:
        return frame_values["correct"], 1
    return frame_values["wrong"], -1


def panel_label(axis, text):
    axis.text(
        0.02,
        0.95,
        text,
        transform=axis.transAxes,
        color="white",
        fontsize=10,
        fontweight="bold",
        va="top",
        bbox={"facecolor": "black", "alpha": 0.75, "pad": 4},
    )


def target_label(axis, sign):
    axis.text(
        0.98,
        0.95,
        f"target {'+' if sign > 0 else '-'}Y",
        transform=axis.transAxes,
        color="white",
        fontsize=9,
        fontweight="bold",
        ha="right",
        va="top",
        bbox={"facecolor": "black", "alpha": 0.65, "pad": 3},
    )


def save_policy_sheet(output_dir, policy, label, frames, mode):
    figure, axes = plt.subplots(2, 4, figsize=(16, 6.6), squeeze=False)
    for row, camera in enumerate(CAMERAS):
        for column, frame in enumerate(FRAMES):
            heatmaps, sign = selected_heatmap(frames[frame], mode)
            axes[row, column].imshow(
                overlay(frames[frame]["images"][row], heatmaps[row])
            )
            axes[row, column].axis("off")
            axes[row, column].set_title(f"frame {frame}", fontsize=12)
            target_label(axes[row, column], sign)
            if column == 0:
                panel_label(axes[row, column], camera)
    definition = (
        "identical correct world-frame +Y target"
        if mode == "correct_plus_y"
        else "behavior-targeted sign from each frame's mean predicted EE-Y"
    )
    figure.suptitle(
        f"{label} | signed EE-Y Grad-CAM overlay | {definition}\n"
        f"same closed-loop rollout | seed {SEED} | PTE/F=0",
        fontsize=14,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.91))
    figure.savefig(
        output_dir / f"{policy}_{mode}_gradcam_frames_0000_0100_0200_0300.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(figure)


def save_overview(output_dir, all_frames, mode):
    figure, axes = plt.subplots(8, 4, figsize=(16, 23), squeeze=False)
    for policy_index, (policy, label) in enumerate(DISPLAY_POLICIES):
        for camera_index, camera in enumerate(CAMERAS):
            row = policy_index * 2 + camera_index
            for column, frame in enumerate(FRAMES):
                values = all_frames[policy][frame]
                heatmaps, sign = selected_heatmap(values, mode)
                axes[row, column].imshow(
                    overlay(values["images"][camera_index], heatmaps[camera_index])
                )
                axes[row, column].axis("off")
                target_label(axes[row, column], sign)
                if row == 0:
                    axes[row, column].set_title(f"frame {frame}", fontsize=13)
                if column == 0:
                    panel_label(axes[row, column], f"{label} | {camera}")
    definition = (
        "Correct-target comparison: identical world-frame +Y target"
        if mode == "correct_plus_y"
        else "Behavior-targeted: frame-wise sign(mean predicted EE-Y, steps 1..89)"
    )
    figure.suptitle(
        "ACT closed-loop signed EE-Y Grad-CAM | frame 0 / 100 / 200 / 300\n"
        f"{definition} | seed {SEED} | blue can | swapped bins | PTE/F=0",
        fontsize=15,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.965))
    figure.savefig(
        output_dir / f"all_policies_{mode}_gradcam_frames_0000_0100_0200_0300.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(figure)


def main():
    repo = Path(__file__).resolve().parents[1]
    output_dir = ROOT / "step_gradcam_contact_sheets"
    output_dir.mkdir(exist_ok=False)
    for policy, _label in DISPLAY_POLICIES:
        print(f"frame 0 Grad-CAM: {policy}", flush=True)
        compute_frame_zero(repo, output_dir, policy)
    all_frames = {
        policy: load_policy_frames(output_dir, policy)
        for policy, _label in DISPLAY_POLICIES
    }
    for mode in ("correct_plus_y", "behavior_targeted"):
        for policy, label in DISPLAY_POLICIES:
            save_policy_sheet(output_dir, policy, label, all_frames[policy], mode)
        save_overview(output_dir, all_frames, mode)
    print(output_dir.resolve())


if __name__ == "__main__":
    main()
