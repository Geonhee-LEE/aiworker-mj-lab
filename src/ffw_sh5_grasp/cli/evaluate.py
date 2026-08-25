"""Evaluate a trained ACT checkpoint in the arm-only environment."""

import argparse
import json
from pathlib import Path

from ffw_sh5_grasp.imitation.runtime.evaluation import evaluate


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--stats", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--num-episodes", type=int, default=10)
    parser.add_argument("--max-steps", type=int, default=500)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--task", choices=("can_to_box", "can_color_sort"), default="can_to_box"
    )
    parser.add_argument(
        "--variant",
        action="append",
        dest="object_variants",
        choices=("green", "red", "orange", "blue"),
        help=(
            "평가에 스폰할 캔 variant를 제한합니다. 여러 색은 옵션을 반복합니다 "
            "(예: --variant orange --variant blue)."
        ),
    )
    parser.add_argument(
        "--representation", choices=("auto", "joint", "task"), default="auto"
    )
    parser.add_argument("--pte-steps", type=int, default=0)
    parser.add_argument("--temporal-decay", type=float, default=0.05)
    parser.add_argument("--task-ik-speed-scale", type=float)
    parser.add_argument("--stable-success-steps", type=int, default=10)
    parser.add_argument("--progress", action="store_true")
    parser.add_argument("--no-rerun", action="store_true")
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="show the policy rollout in the MuJoCo viewer",
    )
    args = parser.parse_args(argv)
    result = evaluate(
        args.checkpoint,
        stats_path=args.stats,
        output_dir=args.output_dir,
        num_episodes=args.num_episodes,
        max_steps=args.max_steps,
        seed=args.seed,
        device=args.device,
        rerun=not args.no_rerun,
        viewer=args.viewer,
        task_name=args.task,
        object_variants=args.object_variants,
        representation=args.representation,
        proleptic_steps=args.pte_steps,
        temporal_decay=args.temporal_decay,
        task_ik_speed_scale=args.task_ik_speed_scale,
        stable_success_steps=args.stable_success_steps,
        progress=args.progress,
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
