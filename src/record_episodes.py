#!/usr/bin/env python3
"""Launch the arm-only can-to-box demonstration recorder."""

import argparse
from pathlib import Path

from ffw_sh5_grasp.imitation.record_app import RecordEpisodesApp


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-name", default="can_to_box")
    parser.add_argument("--dataset-dir", type=Path)
    parser.add_argument("--seed", type=int)
    parser.add_argument(
        "--no-live-rerun", action="store_true",
        help="do not spawn the live Rerun Viewer")
    parser.add_argument("--rerun-port", type=int, default=9876)
    args = parser.parse_args(argv)
    dataset_dir = (args.dataset_dir
                   if args.dataset_dir is not None
                   else Path("datasets") / args.task_name)
    RecordEpisodesApp(
        dataset_dir, task_name=args.task_name, seed=args.seed,
        live_rerun=not args.no_live_rerun,
        rerun_port=args.rerun_port).run()


if __name__ == "__main__":
    main()
