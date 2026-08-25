"""Write a synchronized dataset .rrd recording."""

import argparse
from pathlib import Path

from ffw_sh5_grasp.imitation.visualization.rerun_dataset import (
    log_episode,
    stream_episode,
)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--live",
        action="store_true",
        help="stream directly to Rerun without writing an .rrd",
    )
    parser.add_argument("--port", type=int, default=9877)
    args = parser.parse_args(argv)
    if args.live:
        stream_episode(args.episode, port=args.port)
        return
    output = args.output or args.episode.with_suffix(".rrd")
    print(log_episode(args.episode, output))


if __name__ == "__main__":
    main()
