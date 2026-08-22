"""Launch an isolated joint/task modular ACT training run."""

import argparse
from pathlib import Path

from ffw_sh5_grasp.imitation.act.modular_trainer import train_modular


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args(argv)
    print(train_modular(args.config))


if __name__ == "__main__":
    main()
