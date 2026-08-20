#!/usr/bin/env python3
"""Train the configured 16D ACT policy."""

import argparse
from pathlib import Path

from ffw_sh5_grasp.imitation.act.trainer import train


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path, default=Path("config/imitation/act.yaml"))
    args = parser.parse_args(argv)
    print(train(args.config))


if __name__ == "__main__":
    main()
