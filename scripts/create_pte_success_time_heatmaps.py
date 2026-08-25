"""Build PTE success-rate and completion-time heatmaps from closed-loop trials."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

POLICIES = (
    ("d097_joint", "D97 Joint"),
    ("d097_task", "D97 Task"),
    ("d150_joint", "D150 Joint"),
    ("d150_task", "D150 Task"),
)
F_VALUES = (0, 5, 10, 15, 20)
CONTROL_HZ = 25.0


def load_json(path: Path):
    return json.loads(path.read_text())


def validate_and_collect(source_dir: Path):
    summary_rows = load_json(source_dir / "summary.json")
    summaries = {(row["policy"], int(row["pte_steps"])): row for row in summary_rows}
    expected = {(policy, f_value) for policy, _ in POLICIES for f_value in F_VALUES}
    if set(summaries) != expected:
        raise ValueError("summary.json does not contain the expected 4 x 5 grid")

    collected = []
    common_seeds = None
    common_config = None
    for policy, label in POLICIES:
        for f_value in F_VALUES:
            condition_dir = source_dir / policy / f"f_{f_value:03d}"
            config = load_json(condition_dir / "evaluation_config.json")
            trials = [
                json.loads(line)
                for line in (condition_dir / "trials.jsonl").read_text().splitlines()
                if line.strip()
            ]
            seeds = tuple(int(trial["seed"]) for trial in trials)
            if len(trials) != int(config["num_episodes"]):
                raise ValueError(f"trial count mismatch: {policy} F={f_value}")
            if common_seeds is None:
                common_seeds = seeds
            elif seeds != common_seeds:
                raise ValueError("conditions do not use an identical ordered seed set")

            invariant_config = {
                key: config[key]
                for key in (
                    "num_episodes",
                    "max_steps",
                    "seed",
                    "task_name",
                    "temporal_decay",
                    "task_ik_speed_scale",
                    "stable_success_steps",
                    "camera_names",
                    "chunk_size",
                )
            }
            if common_config is None:
                common_config = invariant_config
            elif invariant_config != common_config:
                raise ValueError("evaluation protocol differs between conditions")

            summary = summaries[(policy, f_value)]
            success_times = [
                float(trial["completion_time_s"])
                for trial in trials
                if trial["success"]
            ]
            all_times = np.asarray(
                [float(trial["completion_time_s"]) for trial in trials], dtype=float
            )
            if not np.isfinite(all_times).all():
                raise ValueError(f"non-finite completion time: {policy} F={f_value}")
            success_count = sum(bool(trial["success"]) for trial in trials)
            success_rate = success_count / len(trials)
            penalized_mean = float(all_times.mean())
            if success_count != int(summary["success_count"]):
                raise ValueError(f"success count mismatch: {policy} F={f_value}")
            if not np.isclose(success_rate, summary["success_rate"]):
                raise ValueError(f"success rate mismatch: {policy} F={f_value}")
            if not np.isclose(penalized_mean, summary["mean_penalized_time_s"]):
                raise ValueError(f"mean time mismatch: {policy} F={f_value}")

            collected.append(
                {
                    "policy": policy,
                    "label": label,
                    "data_count": int(summary["data_count"]),
                    "representation": summary["representation"],
                    "f_steps": f_value,
                    "lookahead_s": f_value / CONTROL_HZ,
                    "num_episodes": len(trials),
                    "success_count": success_count,
                    "success_rate": success_rate,
                    "success_rate_pct": 100.0 * success_rate,
                    "mean_success_time_s": (
                        float(np.mean(success_times)) if success_times else None
                    ),
                    "mean_penalized_time_s": penalized_mean,
                    "timeout_s": float(config["max_steps"]) / CONTROL_HZ,
                    "checkpoint": config["checkpoint"],
                }
            )
    return collected, common_seeds, common_config


def matrices(rows):
    by_key = {(row["policy"], row["f_steps"]): row for row in rows}
    success = np.asarray(
        [
            [by_key[(policy, f_value)]["success_rate_pct"] for f_value in F_VALUES]
            for policy, _ in POLICIES
        ]
    )
    mean_time = np.asarray(
        [
            [by_key[(policy, f_value)]["mean_penalized_time_s"] for f_value in F_VALUES]
            for policy, _ in POLICIES
        ]
    )
    counts = np.asarray(
        [
            [by_key[(policy, f_value)]["success_count"] for f_value in F_VALUES]
            for policy, _ in POLICIES
        ]
    )
    return success, mean_time, counts


def text_color(image, value, norm):
    red, green, blue, _alpha = image.cmap(norm(value))
    luminance = 0.2126 * red + 0.7152 * green + 0.0722 * blue
    return "black" if luminance > 0.55 else "white"


def setup_axis(axis, title):
    axis.set_title(title, fontsize=14, fontweight="bold", pad=12)
    axis.set_xticks(range(len(F_VALUES)))
    axis.set_xticklabels(
        [f"F={value}\n({value / CONTROL_HZ:.1f} s)" for value in F_VALUES]
    )
    axis.set_yticks(range(len(POLICIES)))
    axis.set_yticklabels([label for _policy, label in POLICIES])
    axis.set_xlabel("Proleptic horizon")
    axis.tick_params(length=0)
    axis.set_xticks(np.arange(-0.5, len(F_VALUES), 1), minor=True)
    axis.set_yticks(np.arange(-0.5, len(POLICIES), 1), minor=True)
    axis.grid(which="minor", color="white", linewidth=2)
    axis.tick_params(which="minor", bottom=False, left=False)


def create_combined_figure(output_dir, success, mean_time, counts, num_episodes):
    fig, axes = plt.subplots(1, 2, figsize=(15.5, 6.4), constrained_layout=True)
    success_image = axes[0].imshow(success, cmap="RdYlGn", vmin=0, vmax=100)
    setup_axis(axes[0], "Success rate")
    for row in range(success.shape[0]):
        for column in range(success.shape[1]):
            value = success[row, column]
            axes[0].text(
                column,
                row,
                f"{value:.0f}%\n({counts[row, column]}/{num_episodes})",
                ha="center",
                va="center",
                fontsize=11,
                fontweight="bold",
                color=text_color(success_image, value, success_image.norm),
            )
    success_bar = fig.colorbar(success_image, ax=axes[0], shrink=0.84, pad=0.02)
    success_bar.set_label("Success rate (%)")

    time_image = axes[1].imshow(
        mean_time,
        cmap="RdYlGn_r",
        vmin=float(mean_time.min()),
        vmax=float(mean_time.max()),
    )
    setup_axis(axes[1], "Mean evaluation time (failures = 20 s)")
    for row in range(mean_time.shape[0]):
        for column in range(mean_time.shape[1]):
            value = mean_time[row, column]
            axes[1].text(
                column,
                row,
                f"{value:.2f} s",
                ha="center",
                va="center",
                fontsize=11,
                fontweight="bold",
                color=text_color(time_image, value, time_image.norm),
            )
    time_bar = fig.colorbar(time_image, ax=axes[1], shrink=0.84, pad=0.02)
    time_bar.set_label("Mean time (s), lower is better")

    fig.suptitle(
        "PTE Horizon Sweep — D97/D150 Joint and Task Policies",
        fontsize=18,
        fontweight="bold",
    )
    fig.text(
        0.5,
        0.935,
        f"{num_episodes} identical-seed closed-loop episodes per condition | 25 Hz",
        ha="center",
        fontsize=11,
        color="#444444",
    )
    png_path = output_dir / "pte_success_rate_and_mean_time_heatmap.png"
    pdf_path = output_dir / "pte_success_rate_and_mean_time_heatmap.pdf"
    fig.savefig(png_path, dpi=220, facecolor="white")
    fig.savefig(pdf_path, facecolor="white")
    plt.close(fig)
    return png_path, pdf_path


def write_outputs(output_dir, rows, seeds, common_config, source_dir):
    output_dir.mkdir(parents=True, exist_ok=False)
    success, mean_time, counts = matrices(rows)
    png_path, pdf_path = create_combined_figure(
        output_dir, success, mean_time, counts, len(seeds)
    )

    csv_path = output_dir / "heatmap_data.csv"
    fields = [
        "policy",
        "label",
        "data_count",
        "representation",
        "f_steps",
        "lookahead_s",
        "num_episodes",
        "success_count",
        "success_rate",
        "success_rate_pct",
        "mean_success_time_s",
        "mean_penalized_time_s",
        "timeout_s",
        "checkpoint",
    ]
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    payload = {
        "source_directory": str(source_dir.resolve()),
        "protocol": {
            **common_config,
            "control_hz": CONTROL_HZ,
            "seed_first": seeds[0],
            "seed_last": seeds[-1],
            "identical_ordered_seed_set": True,
            "f_values": list(F_VALUES),
            "mean_time_definition": (
                "arithmetic mean over all trials; failures retain the 20 s timeout"
            ),
        },
        "rows": rows,
        "figure_png": str(png_path.resolve()),
        "figure_pdf": str(pdf_path.resolve()),
    }
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(payload, indent=2) + "\n")

    readme_path = output_dir / "README.md"
    readme_path.write_text(
        "# PTE success/time heatmap\n\n"
        "The two panels use 100 real MuJoCo closed-loop evaluations per cell. "
        "Every policy/F condition uses the identical ordered seed set "
        f"{seeds[0]}–{seeds[-1]}.\n\n"
        "- Left: stable-success rate (10 consecutive success steps).\n"
        "- Right: arithmetic mean evaluation time over all episodes. Failed "
        "episodes remain at the 20.0 s timeout, preventing low-success settings "
        "from appearing artificially fast.\n"
        "- F seconds are computed at 25 Hz: 0, 0.2, 0.4, 0.6, and 0.8 s.\n"
        f"- Source: `{source_dir.resolve()}`\n"
    )
    return png_path, pdf_path, csv_path, summary_path, readme_path


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-dir",
        type=Path,
        default=Path("outputs/evaluation/can_color_sort_pte_balanced_seed195958"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/analysis/pte_success_time_heatmaps_seed195958_20260825"),
    )
    args = parser.parse_args(argv)
    repo = Path(__file__).resolve().parents[1]
    source_dir = (repo / args.source_dir).resolve()
    output_dir = (repo / args.output_dir).resolve()
    rows, seeds, common_config = validate_and_collect(source_dir)
    paths = write_outputs(output_dir, rows, seeds, common_config, source_dir)
    for path in paths:
        print(path.resolve())


if __name__ == "__main__":
    main()
