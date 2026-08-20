#!/usr/bin/env python3
"""Write a synchronized dataset .rrd recording."""

import argparse
from pathlib import Path

from ffw_sh5_grasp.imitation.visualization.rerun_dataset import log_episode


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    output = args.output or args.episode.with_suffix(".rrd")
    print(log_episode(args.episode, output))


if __name__ == "__main__":
    main()
