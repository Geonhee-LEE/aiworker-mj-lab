"""Create frame 0/100/200/300 RGB contact sheets from saved rollouts."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

plt.switch_backend("Agg")


ROOT = Path("outputs/analysis/closed_loop_gradcam_ee_y_seed195958_20260825_r2")
FRAMES = (0, 100, 200, 300)
CAMERAS = ("cam_high", "cam_right_wrist")
POLICIES = (
    ("d097_task", "D97 Task"),
    ("d097_joint", "D97 Joint"),
    ("d150_task", "D150 Task"),
    ("d150_joint", "D150 Joint"),
)


def load_images(policy):
    policy_dir = ROOT / policy
    with np.load(policy_dir / "rollout.npz") as rollout:
        initial = rollout["initial_rgb_images"].copy()
    images = {0: initial}
    for frame in FRAMES[1:]:
        with np.load(policy_dir / "frames" / f"frame_{frame:04d}.npz") as values:
            images[frame] = values["rgb_images"].copy()
    return images


def save_policy_sheet(output_dir, policy, label, images):
    figure, axes = plt.subplots(2, 4, figsize=(16, 6.6), squeeze=False)
    for row, camera in enumerate(CAMERAS):
        for column, frame in enumerate(FRAMES):
            axes[row, column].imshow(images[frame][row])
            axes[row, column].axis("off")
            axes[row, column].set_title(f"frame {frame}", fontsize=12)
            if column == 0:
                axes[row, column].text(
                    0.02,
                    0.95,
                    camera,
                    transform=axes[row, column].transAxes,
                    color="white",
                    fontsize=11,
                    fontweight="bold",
                    va="top",
                    bbox={"facecolor": "black", "alpha": 0.72, "pad": 4},
                )
    figure.suptitle(
        f"{label} | same closed-loop rollout | seed 195958 | PTE/F=0",
        fontsize=15,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    figure.savefig(
        output_dir / f"{policy}_rgb_frames_0000_0100_0200_0300.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(figure)


def save_overview(output_dir, all_images):
    figure, axes = plt.subplots(8, 4, figsize=(16, 23), squeeze=False)
    for policy_index, (policy, label) in enumerate(POLICIES):
        for camera_index, camera in enumerate(CAMERAS):
            row = policy_index * 2 + camera_index
            for column, frame in enumerate(FRAMES):
                axes[row, column].imshow(all_images[policy][frame][camera_index])
                axes[row, column].axis("off")
                if row == 0:
                    axes[row, column].set_title(f"frame {frame}", fontsize=13)
                if column == 0:
                    axes[row, column].text(
                        0.02,
                        0.95,
                        f"{label} | {camera}",
                        transform=axes[row, column].transAxes,
                        color="white",
                        fontsize=10,
                        fontweight="bold",
                        va="top",
                        bbox={"facecolor": "black", "alpha": 0.75, "pad": 4},
                    )
    figure.suptitle(
        "ACT closed-loop RGB progression | frame 0 / 100 / 200 / 300\n"
        "seed 195958 | blue can | swapped bins | PTE/F=0",
        fontsize=16,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.965))
    figure.savefig(
        output_dir / "all_policies_rgb_frames_0000_0100_0200_0300.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(figure)


def main():
    output_dir = ROOT / "step_contact_sheets_v2"
    output_dir.mkdir(exist_ok=False)
    all_images = {}
    for policy, label in POLICIES:
        images = load_images(policy)
        all_images[policy] = images
        save_policy_sheet(output_dir, policy, label, images)
    save_overview(output_dir, all_images)
    print(output_dir.resolve())


if __name__ == "__main__":
    main()
