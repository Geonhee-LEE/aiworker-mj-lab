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
    parser.add_argument("--no-rerun", action="store_true")
    parser.add_argument("--viewer", action="store_true",
                        help="show the policy rollout in the MuJoCo viewer")
    args = parser.parse_args(argv)
    result = evaluate(
        args.checkpoint, stats_path=args.stats, output_dir=args.output_dir,
        num_episodes=args.num_episodes, max_steps=args.max_steps,
        seed=args.seed, device=args.device, rerun=not args.no_rerun,
        viewer=args.viewer)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
