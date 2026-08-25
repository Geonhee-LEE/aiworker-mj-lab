"""Create bar-chart images for training and PTE evaluation distributions."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import h5py
import matplotlib.pyplot as plt
import numpy as np

VARIANTS = ("green", "red", "orange", "blue")
VARIANT_COLORS = {
    "green": "#2CA058",
    "red": "#D93A32",
    "orange": "#EF8A17",
    "blue": "#1F5AA6",
}
POLICIES = (
    ("d097_joint", "D97 Joint"),
    ("d097_task", "D97 Task"),
    ("d150_joint", "D150 Joint"),
    ("d150_task", "D150 Task"),
)
F_VALUES = (0, 5, 10, 15, 20)
SPLIT_FILES = {
    "D97 Joint": Path(
        "outputs/act_modular/can_color_sort_act_joint/episode_splits.json"
    ),
    "D97 Task": Path("outputs/act_modular/can_color_sort_act_task/episode_splits.json"),
    "D150 Joint": Path(
        "outputs/act_modular/can_color_sort_act_joint_aug150/episode_splits.json"
    ),
    "D150 Task": Path(
        "outputs/act_modular/can_color_sort_act_task_aug150/episode_splits.json"
    ),
}


def read_json(path: Path):
    return json.loads(path.read_text())


def read_trials(path: Path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def episode_variant(repo: Path, relative_path: str):
    with h5py.File(repo / relative_path, "r") as episode:
        variant = episode.attrs["object_variant"]
        if isinstance(variant, bytes):
            variant = variant.decode()
        return str(variant)


def collect_dataset_distribution(repo: Path):
    splits = {
        name: read_json(repo / relative_path)
        for name, relative_path in SPLIT_FILES.items()
    }
    if splits["D97 Joint"] != splits["D97 Task"]:
        raise ValueError("D97 Joint and Task do not use identical episode splits")
    if splits["D150 Joint"] != splits["D150 Task"]:
        raise ValueError("D150 Joint and Task do not use identical episode splits")

    rows = []
    counts = {}
    for dataset, split_data in (
        ("D97", splits["D97 Joint"]),
        ("D150", splits["D150 Joint"]),
    ):
        counts[dataset] = {}
        for split_name, paths in split_data.items():
            split_counts = Counter(episode_variant(repo, path) for path in paths)
            counts[dataset][split_name] = split_counts
            for variant in VARIANTS:
                rows.append(
                    {
                        "dataset": dataset,
                        "split": split_name,
                        "variant": variant,
                        "episode_count": split_counts[variant],
                    }
                )
        all_counts = sum(counts[dataset].values(), Counter())
        counts[dataset]["all"] = all_counts
        for variant in VARIANTS:
            rows.append(
                {
                    "dataset": dataset,
                    "split": "all",
                    "variant": variant,
                    "episode_count": all_counts[variant],
                }
            )
    if sum(counts["D97"]["all"].values()) != 97:
        raise ValueError("D97 does not contain 97 episodes")
    if sum(counts["D150"]["all"].values()) != 150:
        raise ValueError("D150 does not contain 150 episodes")
    return rows, counts


def scenario_signature(trials):
    return tuple(
        (
            int(trial["seed"]),
            trial["object_variant"],
            bool(trial["bin_colors_swapped"]),
        )
        for trial in trials
    )


def collect_evaluation_distribution(source_dir: Path):
    reference_signature = None
    reference_trials = None
    outcome_rows = []
    for policy, label in POLICIES:
        for f_value in F_VALUES:
            trials = read_trials(
                source_dir / policy / f"f_{f_value:03d}" / "trials.jsonl"
            )
            signature = scenario_signature(trials)
            if reference_signature is None:
                reference_signature = signature
                reference_trials = trials
            elif signature != reference_signature:
                raise ValueError("evaluation conditions do not share one scenario set")
            success_count = sum(bool(trial["success"]) for trial in trials)
            outcome_rows.append(
                {
                    "policy": policy,
                    "label": label,
                    "f_steps": f_value,
                    "lookahead_s": f_value / 25.0,
                    "num_episodes": len(trials),
                    "success_count": success_count,
                    "failure_count": len(trials) - success_count,
                }
            )

    variant_counts = Counter(trial["object_variant"] for trial in reference_trials)
    layout_counts = Counter(
        "swapped" if trial["bin_colors_swapped"] else "default"
        for trial in reference_trials
    )
    scenario_rows = []
    for variant in VARIANTS:
        for layout in ("default", "swapped"):
            count = sum(
                trial["object_variant"] == variant
                and ("swapped" if trial["bin_colors_swapped"] else "default") == layout
                for trial in reference_trials
            )
            scenario_rows.append(
                {"variant": variant, "layout": layout, "episode_count": count}
            )
    return (
        outcome_rows,
        scenario_rows,
        variant_counts,
        layout_counts,
        reference_signature,
    )


def style_axis(axis):
    axis.grid(axis="y", alpha=0.18, zorder=0)
    axis.spines[["top", "right"]].set_visible(False)
    axis.set_axisbelow(True)


def grouped_dataset_bars(axis, counts, split_name, title):
    x = np.arange(len(VARIANTS))
    width = 0.36
    for offset, dataset, color in (
        (-width / 2, "D97", "#7A8AA0"),
        (width / 2, "D150", "#1F5AA6"),
    ):
        values = [counts[dataset][split_name][variant] for variant in VARIANTS]
        bars = axis.bar(x + offset, values, width, label=dataset, color=color, zorder=2)
        axis.bar_label(bars, padding=3, fontsize=9, fontweight="bold")
    axis.set_title(title, fontweight="bold")
    axis.set_xticks(x, [variant.capitalize() for variant in VARIANTS])
    axis.set_ylabel("Episodes")
    axis.legend(frameon=False)
    style_axis(axis)


def create_distribution_overview(
    output_dir, dataset_counts, variant_counts, layout_counts, scenario_rows
):
    fig, axes = plt.subplots(2, 2, figsize=(14.5, 9.2), constrained_layout=True)
    grouped_dataset_bars(axes[0, 0], dataset_counts, "all", "All dataset episodes")
    grouped_dataset_bars(axes[0, 1], dataset_counts, "train", "Training split only")

    variants = list(VARIANTS)
    values = [variant_counts[variant] for variant in variants]
    bars = axes[1, 0].bar(
        variants,
        values,
        color=[VARIANT_COLORS[variant] for variant in variants],
        width=0.62,
        zorder=2,
    )
    axes[1, 0].bar_label(
        bars,
        labels=[f"{value}\n({value:.0f}%)" for value in values],
        padding=3,
        fontweight="bold",
    )
    axes[1, 0].set(
        title="Evaluation seeds by can color",
        ylabel="Episodes",
        ylim=(0, 31),
    )
    axes[1, 0].title.set_fontweight("bold")
    style_axis(axes[1, 0])

    x = np.arange(len(VARIANTS))
    bottom = np.zeros(len(VARIANTS))
    for layout, color in (("default", "#6C7A89"), ("swapped", "#EF8A17")):
        values = [
            next(
                row["episode_count"]
                for row in scenario_rows
                if row["variant"] == variant and row["layout"] == layout
            )
            for variant in VARIANTS
        ]
        bars = axes[1, 1].bar(
            x,
            values,
            bottom=bottom,
            label=layout.capitalize(),
            color=color,
            width=0.62,
            zorder=2,
        )
        labels = [str(value) if value else "" for value in values]
        axes[1, 1].bar_label(
            bars,
            labels=labels,
            label_type="center",
            color="white",
            fontsize=9,
            fontweight="bold",
        )
        bottom += values
    axes[1, 1].set(
        title="Evaluation color × bin-layout balance",
        ylabel="Episodes",
        xticks=x,
        xticklabels=[variant.capitalize() for variant in VARIANTS],
        ylim=(0, 31),
    )
    axes[1, 1].title.set_fontweight("bold")
    axes[1, 1].legend(frameon=False, ncol=2)
    style_axis(axes[1, 1])

    fig.suptitle(
        "Training Dataset and Closed-loop Evaluation Distributions",
        fontsize=18,
        fontweight="bold",
    )
    png = output_dir / "data_distribution_bar_overview.png"
    pdf = output_dir / "data_distribution_bar_overview.pdf"
    fig.savefig(png, dpi=220, facecolor="white")
    fig.savefig(pdf, facecolor="white")
    plt.close(fig)
    return png, pdf


def create_outcome_bars(output_dir, outcome_rows):
    by_key = {(row["policy"], row["f_steps"]): row for row in outcome_rows}
    fig, axes = plt.subplots(
        2, 2, figsize=(13.4, 8.3), sharey=True, constrained_layout=True
    )
    for axis, (policy, label) in zip(axes.flat, POLICIES):
        success = np.asarray(
            [by_key[(policy, f_value)]["success_count"] for f_value in F_VALUES]
        )
        failure = 100 - success
        x = np.arange(len(F_VALUES))
        success_bars = axis.bar(
            x, success, color="#2CA058", label="Success", width=0.66, zorder=2
        )
        failure_bars = axis.bar(
            x,
            failure,
            bottom=success,
            color="#C9CED6",
            label="Failure",
            width=0.66,
            zorder=2,
        )
        axis.bar_label(
            success_bars,
            labels=[str(value) if value else "" for value in success],
            label_type="center",
            color="white",
            fontweight="bold",
        )
        axis.bar_label(
            failure_bars,
            labels=[str(value) if value else "" for value in failure],
            label_type="center",
            color="#303744",
            fontweight="bold",
        )
        axis.set_title(label, fontweight="bold")
        axis.set_xticks(
            x,
            [f"F={value}\n{value / 25.0:.1f}s" for value in F_VALUES],
        )
        axis.set_ylim(0, 108)
        axis.set_ylabel("Episodes")
        style_axis(axis)
    axes[0, 0].legend(frameon=False, ncol=2, loc="lower left")
    fig.suptitle(
        "PTE Evaluation Outcome Distribution",
        fontsize=18,
        fontweight="bold",
    )
    png = output_dir / "pte_success_failure_distribution_bar.png"
    pdf = output_dir / "pte_success_failure_distribution_bar.pdf"
    fig.savefig(png, dpi=220, facecolor="white")
    fig.savefig(pdf, facecolor="white")
    plt.close(fig)
    return png, pdf


def write_csv(path: Path, rows, fields):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--evaluation-dir",
        type=Path,
        default=Path("outputs/evaluation/can_color_sort_pte_balanced_seed195958"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(
            "outputs/analysis/data_distribution_bar_charts_seed195958_20260825_r2"
        ),
    )
    args = parser.parse_args(argv)
    repo = Path(__file__).resolve().parents[1]
    evaluation_dir = (repo / args.evaluation_dir).resolve()
    output_dir = (repo / args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=False)

    dataset_rows, dataset_counts = collect_dataset_distribution(repo)
    outcome_rows, scenario_rows, variant_counts, layout_counts, signature = (
        collect_evaluation_distribution(evaluation_dir)
    )
    overview_paths = create_distribution_overview(
        output_dir, dataset_counts, variant_counts, layout_counts, scenario_rows
    )
    outcome_paths = create_outcome_bars(output_dir, outcome_rows)

    dataset_csv = output_dir / "training_dataset_distribution.csv"
    scenario_csv = output_dir / "evaluation_scenario_distribution.csv"
    outcome_csv = output_dir / "evaluation_outcome_distribution.csv"
    write_csv(
        dataset_csv,
        dataset_rows,
        ("dataset", "split", "variant", "episode_count"),
    )
    write_csv(
        scenario_csv,
        scenario_rows,
        ("variant", "layout", "episode_count"),
    )
    write_csv(
        outcome_csv,
        outcome_rows,
        (
            "policy",
            "label",
            "f_steps",
            "lookahead_s",
            "num_episodes",
            "success_count",
            "failure_count",
        ),
    )

    summary = {
        "evaluation_source": str(evaluation_dir),
        "joint_task_splits_identical": {"D97": True, "D150": True},
        "dataset_all_counts": {
            dataset: dict(dataset_counts[dataset]["all"]) for dataset in ("D97", "D150")
        },
        "dataset_train_counts": {
            dataset: dict(dataset_counts[dataset]["train"])
            for dataset in ("D97", "D150")
        },
        "evaluation_seed_first": signature[0][0],
        "evaluation_seed_last": signature[-1][0],
        "evaluation_variant_counts": dict(variant_counts),
        "evaluation_layout_counts": dict(layout_counts),
        "evaluation_conditions_share_scenarios": True,
        "files": [
            str(path.resolve())
            for path in (
                *overview_paths,
                *outcome_paths,
                dataset_csv,
                scenario_csv,
                outcome_csv,
            )
        ],
    }
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")
    readme_path = output_dir / "README.md"
    readme_path.write_text(
        "# Data distribution bar charts\n\n"
        "- `data_distribution_bar_overview`: D97/D150 color coverage for all "
        "episodes and training splits, plus the evaluation color/layout balance.\n"
        "- `pte_success_failure_distribution_bar`: raw success/failure counts for "
        "each policy and F value.\n"
        "- D97 Joint/Task and D150 Joint/Task split files were verified identical.\n"
        "- Every policy/F evaluation uses the same ordered seeds 195958–196057.\n"
    )
    for path in (
        *overview_paths,
        *outcome_paths,
        dataset_csv,
        scenario_csv,
        outcome_csv,
        summary_path,
        readme_path,
    ):
        print(path.resolve())


if __name__ == "__main__":
    main()
