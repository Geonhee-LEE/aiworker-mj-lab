#!/usr/bin/env python3
"""Generate reproducible documentation charts from color-sort evaluations."""

from __future__ import annotations

import csv
import json
import math
from collections import Counter
from pathlib import Path

import h5py
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RESULT_DIR = ROOT / "outputs/evaluation/can_color_sort_pte_m005"
ASSET_DIR = ROOT / "docs/assets/evaluation"

POLICIES = (
    ("d097_joint", "D97 Joint", 97, "joint"),
    ("d097_task", "D97 Task", 97, "task"),
    ("d150_joint", "D150 Joint", 150, "joint"),
    ("d150_task", "D150 Task", 150, "task"),
)
PTE_STEPS = (0, 5, 10, 15, 20)
COLORS = {
    "d097_joint": "#7A8AA0",
    "d097_task": "#20A4B8",
    "d150_joint": "#1F5AA6",
    "d150_task": "#EF8A17",
}
VARIANT_COLORS = {
    "green": "#2CA058",
    "red": "#D93A32",
    "orange": "#EF8A17",
    "blue": "#1F5AA6",
}


def _save(fig, name):
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(
        ASSET_DIR / name,
        format="svg",
        bbox_inches="tight",
        facecolor="white",
        metadata={"Date": None},
    )
    plt.close(fig)


def _load_summary():
    path = RESULT_DIR / "summary.csv"
    if not path.is_file():
        raise FileNotFoundError(f"evaluation summary is missing: {path}")
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    numeric = (
        "data_count",
        "pte_steps",
        "lookahead_s",
        "num_episodes",
        "success_count",
        "success_rate",
        "success_ci95_low",
        "success_ci95_high",
        "median_completion_time_s",
        "mean_penalized_time_s",
        "penalized_speedup_vs_f0",
        "mean_policy_inference_ms",
        "p95_ik_position_error_mm",
        "minimum_collision_distance_m",
        "max_collision_constraint_violation",
    )
    for row in rows:
        for name in numeric:
            row[name] = float(row[name]) if row[name] else math.nan
    return rows


def _load_trials(policy, steps):
    path = RESULT_DIR / policy / f"f_{steps:03d}/evaluation.json"
    if not path.is_file():
        raise FileNotFoundError(f"evaluation result is missing: {path}")
    return json.loads(path.read_text(encoding="utf-8"))["episodes"]


def _subset(rows, policy):
    return sorted(
        (row for row in rows if row["policy"] == policy),
        key=lambda row: row["pte_steps"],
    )


def _style():
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.edgecolor": "#D8E1EC",
            "axes.labelcolor": "#44546A",
            "axes.titlecolor": "#0F1B2D",
            "xtick.color": "#44546A",
            "ytick.color": "#44546A",
            "svg.fonttype": "none",
        }
    )


def success_rate_chart(rows):
    fig, ax = plt.subplots(figsize=(9.6, 5.1), constrained_layout=True)
    for policy, label, _count, _representation in POLICIES:
        subset = _subset(rows, policy)
        x = np.asarray([row["pte_steps"] for row in subset])
        y = np.asarray([row["success_rate"] * 100.0 for row in subset])
        low = np.asarray([row["success_ci95_low"] * 100.0 for row in subset])
        high = np.asarray([row["success_ci95_high"] * 100.0 for row in subset])
        ax.errorbar(
            x,
            y,
            yerr=[np.maximum(0.0, y - low), np.maximum(0.0, high - y)],
            label=label,
            color=COLORS[policy],
            marker="o",
            linewidth=2.2,
            markersize=6,
            capsize=3,
        )
    ax.axvspan(-0.5, 10.5, color="#E9F2FC", alpha=0.58, zorder=-2)
    ax.axvline(10.5, color="#D93A32", linestyle="--", linewidth=1.2)
    ax.text(10.9, 7, "Reliability cliff", color="#D93A32", fontsize=9)
    ax.set(
        title="Success rate vs. PTE look-ahead (Wilson 95% CI)",
        xlabel="PTE future step f  (25 Hz)",
        ylabel="Success rate (%)",
        xticks=PTE_STEPS,
        ylim=(-3, 105),
    )
    ax.grid(axis="y", alpha=0.22)
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(frameon=False, ncol=2, loc="lower left")
    _save(fig, "success-rate-vs-pte.svg")


def success_heatmap(rows):
    matrix = np.asarray(
        [
            [
                next(
                    row["success_rate"]
                    for row in rows
                    if row["policy"] == policy and row["pte_steps"] == steps
                )
                * 100.0
                for steps in PTE_STEPS
            ]
            for policy, _label, _count, _representation in POLICIES
        ]
    )
    fig, ax = plt.subplots(figsize=(8.7, 3.8), constrained_layout=True)
    image = ax.imshow(matrix, cmap="RdYlGn", vmin=0, vmax=100, aspect="auto")
    for row_index in range(matrix.shape[0]):
        for column_index in range(matrix.shape[1]):
            value = matrix[row_index, column_index]
            ax.text(
                column_index,
                row_index,
                f"{value:.0f}%",
                ha="center",
                va="center",
                color="white" if value < 35 or value > 82 else "#0F1B2D",
                fontweight="bold",
            )
    ax.set(
        title="Success-rate matrix",
        xticks=range(len(PTE_STEPS)),
        xticklabels=[f"f={v}" for v in PTE_STEPS],
        yticks=range(len(POLICIES)),
        yticklabels=[item[1] for item in POLICIES],
    )
    fig.colorbar(image, ax=ax, label="Success rate (%)", shrink=0.85)
    _save(fig, "success-rate-heatmap.svg")


def penalized_time_chart(rows):
    fig, ax = plt.subplots(figsize=(9.6, 4.9), constrained_layout=True)
    for policy, label, _count, _representation in POLICIES:
        subset = _subset(rows, policy)
        ax.plot(
            [row["pte_steps"] for row in subset],
            [row["mean_penalized_time_s"] for row in subset],
            label=label,
            color=COLORS[policy],
            marker="o",
            linewidth=2.2,
            markersize=6,
        )
    ax.axhline(20.0, color="#D8E1EC", linewidth=1.0)
    ax.axvspan(-0.5, 10.5, color="#E9F2FC", alpha=0.58, zorder=-2)
    ax.set(
        title="Penalized completion time (failures count as 20 s)",
        xlabel="PTE future step f",
        ylabel="Mean penalized time (s)",
        xticks=PTE_STEPS,
        ylim=(6.5, 20.8),
    )
    ax.grid(axis="y", alpha=0.22)
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(frameon=False, ncol=2, loc="upper left")
    _save(fig, "penalized-time-vs-pte.svg")


def pareto_chart(rows):
    fig, ax = plt.subplots(figsize=(7.4, 5.4), constrained_layout=True)
    for policy, label, _count, _representation in POLICIES:
        subset = _subset(rows, policy)
        x = [row["mean_penalized_time_s"] for row in subset]
        y = [row["success_rate"] * 100.0 for row in subset]
        ax.plot(x, y, color=COLORS[policy], linewidth=1.2, alpha=0.6)
        ax.scatter(x, y, color=COLORS[policy], s=50, label=label)
        for row, x_value, y_value in zip(subset, x, y):
            ax.annotate(
                f"f{int(row['pte_steps'])}",
                (x_value, y_value),
                xytext=(4, 4),
                textcoords="offset points",
                fontsize=8,
                color=COLORS[policy],
            )
    ax.annotate(
        "Preferred direction",
        xy=(7.5, 100),
        xytext=(11.5, 83),
        arrowprops={"arrowstyle": "->", "color": "#2CA058"},
        color="#2CA058",
        fontweight="bold",
    )
    ax.set(
        title="Reliability–speed trade-off",
        xlabel="Mean penalized time (s)  ← faster",
        ylabel="Success rate (%)  → more reliable",
        xlim=(7, 20.5),
        ylim=(-3, 105),
    )
    ax.grid(alpha=0.2)
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(frameon=False, fontsize=9)
    _save(fig, "success-time-pareto.svg")


def dataset_composition_chart():
    run_paths = {
        "D97": ROOT / "outputs/act_modular/can_color_sort_act_joint",
        "D150": ROOT / "outputs/act_modular/can_color_sort_act_joint_aug150",
    }
    variants = ("green", "red", "orange", "blue")
    totals = {}
    train = {}
    for label, run_path in run_paths.items():
        splits = json.loads(
            (run_path / "episode_splits.json").read_text(encoding="utf-8")
        )
        total_counts = Counter()
        train_counts = Counter()
        for split_name, paths in splits.items():
            for path in paths:
                with h5py.File(path, "r") as root:
                    variant = root.attrs["object_variant"]
                    if isinstance(variant, bytes):
                        variant = variant.decode("utf-8")
                total_counts[str(variant)] += 1
                if split_name == "train":
                    train_counts[str(variant)] += 1
        totals[label] = total_counts
        train[label] = train_counts

    fig, axes = plt.subplots(
        1, 2, figsize=(9.6, 4.1), sharey=True, constrained_layout=True
    )
    x = np.arange(len(variants))
    width = 0.35
    for ax, (title, counts) in zip(
        axes, (("All episodes", totals), ("Train split only", train))
    ):
        for offset, label in ((-width / 2, "D97"), (width / 2, "D150")):
            values = [counts[label][variant] for variant in variants]
            bars = ax.bar(
                x + offset,
                values,
                width,
                label=label,
                color="#7A8AA0" if label == "D97" else "#1F5AA6",
            )
            ax.bar_label(bars, padding=2, fontsize=8)
        ax.set_title(title)
        ax.set_xticks(x, [name.capitalize() for name in variants])
        ax.grid(axis="y", alpha=0.18)
        ax.spines[["top", "right"]].set_visible(False)
    axes[0].set_ylabel("Episodes")
    axes[1].legend(frameon=False)
    fig.suptitle("Dataset color composition: D97 changes both count and coverage")
    _save(fig, "dataset-color-composition.svg")


def variant_success_chart():
    variants = ("green", "red", "orange", "blue")
    rates = {}
    counts = {}
    for policy, _label, _count, _representation in POLICIES:
        trials = _load_trials(policy, 0)
        total = Counter(item["object_variant"] for item in trials)
        success = Counter(item["object_variant"] for item in trials if item["success"])
        rates[policy] = [
            success[variant] / total[variant] * 100.0 for variant in variants
        ]
        counts[policy] = [total[variant] for variant in variants]

    fig, axes = plt.subplots(
        1, 2, figsize=(10.2, 4.6), sharey=True, constrained_layout=True
    )
    x = np.arange(len(variants))
    width = 0.36
    for ax, data_count in zip(axes, (97, 150)):
        policies = [item for item in POLICIES if item[2] == data_count]
        for offset, (policy, label, _count, _representation) in zip(
            (-width / 2, width / 2), policies
        ):
            bars = ax.bar(
                x + offset, rates[policy], width, label=label, color=COLORS[policy]
            )
            annotations = [
                f"{rate:.0f}%\n(n={count})"
                for rate, count in zip(rates[policy], counts[policy])
            ]
            ax.bar_label(bars, labels=annotations, padding=2, fontsize=7.5)
        ax.set(
            title=f"D{data_count}, f=0",
            xticks=x,
            xticklabels=[name.capitalize() for name in variants],
            ylim=(0, 116),
        )
        ax.grid(axis="y", alpha=0.2)
        ax.spines[["top", "right"]].set_visible(False)
        ax.legend(frameon=False, fontsize=9, loc="lower left")
    axes[0].set_ylabel("Success rate (%)")
    fig.suptitle("Per-color success exposes D97 out-of-distribution behavior")
    _save(fig, "f0-success-by-color.svg")


def paired_outcome_chart():
    labels = []
    joint_only = []
    task_only = []
    for data_count in (97, 150):
        joint_name = f"d{data_count:03d}_joint"
        task_name = f"d{data_count:03d}_task"
        for steps in PTE_STEPS:
            joint = {
                item["seed"]: item["success"]
                for item in _load_trials(joint_name, steps)
            }
            task = {
                item["seed"]: item["success"] for item in _load_trials(task_name, steps)
            }
            labels.append(f"D{data_count}\nf{steps}")
            joint_only.append(sum(joint[seed] and not task[seed] for seed in joint))
            task_only.append(sum(task[seed] and not joint[seed] for seed in joint))

    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(10.2, 4.8), constrained_layout=True)
    ax.bar(x, task_only, color="#20A4B8", label="Task only succeeds")
    ax.bar(x, -np.asarray(joint_only), color="#1F5AA6", label="Joint only succeeds")
    for index, value in enumerate(task_only):
        ax.text(index, value + 1, str(value), ha="center", va="bottom", fontsize=8)
    for index, value in enumerate(joint_only):
        ax.text(index, -value - 1, str(value), ha="center", va="top", fontsize=8)
    ax.axhline(0, color="#44546A", linewidth=0.9)
    ax.axvline(4.5, color="#D8E1EC", linewidth=1.2)
    ax.set(
        title="Paired-seed discordant outcomes",
        xlabel="Dataset and PTE condition",
        ylabel="Episodes (Task-only + / Joint-only −)",
        xticks=x,
        xticklabels=labels,
        ylim=(-82, 30),
    )
    ax.grid(axis="y", alpha=0.18)
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(frameon=False, loc="lower left")
    _save(fig, "paired-seed-outcomes.svg")


def task_ik_chart(rows):
    fig, axes = plt.subplots(1, 2, figsize=(9.8, 4.3), constrained_layout=True)
    for policy, label, _count, representation in POLICIES:
        if representation != "task":
            continue
        subset = _subset(rows, policy)
        x = [row["pte_steps"] for row in subset]
        axes[0].plot(
            x,
            [row["p95_ik_position_error_mm"] for row in subset],
            marker="o",
            linewidth=2.2,
            label=label,
            color=COLORS[policy],
        )
        axes[1].plot(
            x,
            [row["minimum_collision_distance_m"] * 1000.0 for row in subset],
            marker="o",
            linewidth=2.2,
            label=label,
            color=COLORS[policy],
        )
    axes[0].set(
        title="IK position error",
        xlabel="PTE future step f",
        ylabel="P95 position error (mm)",
        xticks=PTE_STEPS,
    )
    axes[1].set(
        title="Closest monitored collision pair",
        xlabel="PTE future step f",
        ylabel="Minimum distance (mm)",
        xticks=PTE_STEPS,
    )
    for ax in axes:
        ax.grid(alpha=0.2)
        ax.spines[["top", "right"]].set_visible(False)
        ax.legend(frameon=False)
    fig.suptitle("Task-space execution diagnostics")
    _save(fig, "task-ik-diagnostics.svg")


def main():
    _style()
    rows = _load_summary()
    success_rate_chart(rows)
    success_heatmap(rows)
    penalized_time_chart(rows)
    pareto_chart(rows)
    dataset_composition_chart()
    variant_success_chart()
    paired_outcome_chart()
    task_ik_chart(rows)
    print(f"wrote 8 evaluation charts to {ASSET_DIR}")


if __name__ == "__main__":
    main()
