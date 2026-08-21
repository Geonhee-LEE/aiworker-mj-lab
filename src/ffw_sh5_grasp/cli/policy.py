"""Open the interactive arm-only ACT policy teleoperation UI."""

import argparse
from pathlib import Path

from ffw_sh5_grasp.imitation.apps.policy import ACTPolicyApp


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--stats", type=Path)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--max-steps", type=int, default=500)
    args = parser.parse_args(argv)
    ACTPolicyApp(
        args.checkpoint, stats_path=args.stats, device=args.device,
        seed=args.seed, max_steps=args.max_steps).run()


if __name__ == "__main__":
    main()
