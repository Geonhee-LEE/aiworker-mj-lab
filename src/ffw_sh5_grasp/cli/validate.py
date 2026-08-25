"""Validate recorded ALOHA-style episodes before training."""

import argparse
import json
from pathlib import Path

from ffw_sh5_grasp.imitation.data.validation import inspect_dataset


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Check HDF5 schema, alignment, cameras and finite values."
    )
    parser.add_argument("--dataset-dir", required=True, type=Path)
    parser.add_argument(
        "--camera",
        action="append",
        default=[],
        help="required camera name; may be repeated",
    )
    parser.add_argument(
        "--json", action="store_true", help="emit a machine-readable report"
    )
    args = parser.parse_args(argv)
    report = inspect_dataset(args.dataset_dir, required_cameras=tuple(args.camera))
    values = report.as_dict()
    if args.json:
        print(json.dumps(values, indent=2))
    else:
        print(
            f"episodes={values['episode_count']} "
            f"success={values['success_count']} "
            f"frames={values['total_frames']} valid={values['valid']}"
        )
        for episode in report.episodes:
            for error in episode.errors:
                print(f"ERROR {episode.path}: {error}")
    return 0 if report.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
