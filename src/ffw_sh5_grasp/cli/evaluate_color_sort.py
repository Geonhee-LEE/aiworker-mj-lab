"""Run the fixed color-sort Joint/Task/Data/PTE experiment matrix."""

import argparse
import csv
import json
from pathlib import Path

from ffw_sh5_grasp.imitation.runtime.evaluation import evaluate

DEFAULT_POLICIES = {
    "d097_joint": {
        "data_count": 97,
        "representation": "joint",
        "checkpoint": Path(
            "outputs/act_modular/can_color_sort_act_joint/"
            "checkpoints/policy_best.ckpt"),
    },
    "d097_task": {
        "data_count": 97,
        "representation": "task",
        "checkpoint": Path(
            "outputs/act_modular/can_color_sort_act_task/"
            "checkpoints/policy_best.ckpt"),
    },
    "d150_joint": {
        "data_count": 150,
        "representation": "joint",
        "checkpoint": Path(
            "outputs/act_modular/can_color_sort_act_joint_aug150/"
            "checkpoints/policy_best.ckpt"),
    },
    "d150_task": {
        "data_count": 150,
        "representation": "task",
        "checkpoint": Path(
            "outputs/act_modular/can_color_sort_act_task_aug150/"
            "checkpoints/policy_best.ckpt"),
    },
}


def _write_summary(path, rows):
    fieldnames = (
        "policy", "data_count", "representation", "pte_steps",
        "lookahead_s", "num_episodes", "success_count", "success_rate",
        "success_ci95_low", "success_ci95_high",
        "median_completion_time_s", "mean_penalized_time_s",
        "penalized_speedup_vs_f0", "mean_policy_inference_ms",
        "p95_ik_position_error_mm", "minimum_collision_distance_m",
        "max_collision_constraint_violation",
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate color-sort data count x representation x PTE with "
            "identical episode seeds."))
    parser.add_argument(
        "--policies", nargs="+", choices=tuple(DEFAULT_POLICIES),
        default=list(DEFAULT_POLICIES))
    parser.add_argument(
        "--pte-steps", nargs="+", type=int,
        default=[0, 5, 10, 15, 20])
    parser.add_argument("--num-episodes", type=int, default=100)
    parser.add_argument("--max-steps", type=int, default=500)
    parser.add_argument("--seed", type=int, default=10000)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--temporal-decay", type=float, default=0.05)
    parser.add_argument("--task-ik-speed-scale", type=float, default=3.0)
    parser.add_argument("--stable-success-steps", type=int, default=10)
    parser.add_argument(
        "--output-dir", type=Path,
        default=Path("outputs/evaluation/can_color_sort_pte_m005"))
    parser.add_argument(
        "--rerun", action="store_true",
        help="write per-trial RRD files (disabled for unbiased timing by default)")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    if args.num_episodes <= 0 or args.max_steps <= 0:
        parser.error("--num-episodes and --max-steps must be positive")
    if args.stable_success_steps <= 0:
        parser.error("--stable-success-steps must be positive")
    if any(step < 0 for step in args.pte_steps):
        parser.error("--pte-steps values must be non-negative")
    pte_steps = tuple(dict.fromkeys(args.pte_steps))
    selected = [(name, DEFAULT_POLICIES[name]) for name in args.policies]
    total = len(selected) * len(pte_steps) * args.num_episodes
    print(
        f"color-sort matrix: policies={len(selected)} "
        f"pte={list(pte_steps)} episodes/cell={args.num_episodes} "
        f"total_rollouts={total}",
        flush=True)
    if args.dry_run:
        for name, policy in selected:
            for steps in pte_steps:
                print(
                    f"{name} representation={policy['representation']} "
                    f"data={policy['data_count']} f={steps}")
        return

    rows = []
    baselines = {}
    evaluations = {}
    for name, policy in selected:
        checkpoint = policy["checkpoint"]
        if not checkpoint.is_file():
            raise FileNotFoundError(f"missing checkpoint: {checkpoint}")
        for steps in pte_steps:
            cell_dir = args.output_dir / name / f"f_{steps:03d}"
            print(f"START {name} f={steps} -> {cell_dir}", flush=True)
            result = evaluate(
                checkpoint,
                output_dir=cell_dir,
                num_episodes=args.num_episodes,
                max_steps=args.max_steps,
                seed=args.seed,
                device=args.device,
                rerun=args.rerun,
                task_name="can_color_sort",
                representation=policy["representation"],
                proleptic_steps=steps,
                temporal_decay=args.temporal_decay,
                task_ik_speed_scale=args.task_ik_speed_scale,
                stable_success_steps=args.stable_success_steps,
                progress=True,
            )
            evaluations[(name, steps)] = result
            if steps == 0:
                baselines[name] = result["mean_penalized_time_s"]
            print(
                f"DONE {name} f={steps} "
                f"success={result['success_count']}/{result['num_episodes']} "
                f"penalized_time={result['mean_penalized_time_s']:.3f}s",
                flush=True)

    for name, policy in selected:
        baseline = baselines.get(name)
        if baseline is None and (name, 0) in evaluations:
            baseline = evaluations[(name, 0)]["mean_penalized_time_s"]
        for steps in pte_steps:
            result = evaluations[(name, steps)]
            ci_low, ci_high = result["success_rate_ci95"]
            penalized = result["mean_penalized_time_s"]
            rows.append({
                "policy": name,
                "data_count": policy["data_count"],
                "representation": policy["representation"],
                "pte_steps": steps,
                "lookahead_s": steps / result["control_hz"],
                "num_episodes": result["num_episodes"],
                "success_count": result["success_count"],
                "success_rate": result["success_rate"],
                "success_ci95_low": ci_low,
                "success_ci95_high": ci_high,
                "median_completion_time_s": result[
                    "median_completion_time_s"],
                "mean_penalized_time_s": penalized,
                "penalized_speedup_vs_f0": (
                    baseline / penalized
                    if baseline is not None and penalized else None),
                "mean_policy_inference_ms": result[
                    "mean_policy_inference_ms"],
                "p95_ik_position_error_mm": result[
                    "p95_ik_position_error_mm"],
                "minimum_collision_distance_m": result[
                    "minimum_collision_distance_m"],
                "max_collision_constraint_violation": result[
                    "max_collision_constraint_violation"],
            })
    _write_summary(args.output_dir / "summary.csv", rows)
    (args.output_dir / "summary.json").write_text(
        json.dumps(rows, indent=2), encoding="utf-8")
    print(f"summary: {args.output_dir / 'summary.csv'}", flush=True)


if __name__ == "__main__":
    main()
