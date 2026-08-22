"""Launch the arm-only can-to-box demonstration recorder."""

import argparse
from pathlib import Path

from ffw_sh5_grasp.imitation.apps.recording import RecordEpisodesApp
from ffw_sh5_grasp.imitation.simulation.task import TASK_NAMES


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--task-name", choices=TASK_NAMES, default="can_to_box")
    parser.add_argument("--dataset-dir", type=Path)
    parser.add_argument("--seed", type=int)
    parser.add_argument(
        "--variant", action="append", dest="object_variants",
        help=("수집할 캔 variant를 제한합니다. 여러 색은 옵션을 반복합니다 "
              "(예: --variant orange --variant blue)."))
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
        object_variants=args.object_variants,
        live_rerun=not args.no_live_rerun,
        rerun_port=args.rerun_port).run()


if __name__ == "__main__":
    main()
