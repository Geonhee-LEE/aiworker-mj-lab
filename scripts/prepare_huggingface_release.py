#!/usr/bin/env python3
"""Validate local artifacts and build reproducible Hugging Face manifests."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

import h5py
import torch
import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET_DIR = ROOT / "datasets" / "can_color_sort"
DEFAULT_OUTPUTS_DIR = ROOT / "outputs" / "act_modular"
DEFAULT_RELEASE_DIR = ROOT / "huggingface" / "release"
REQUIRED_CAMERAS = ("cam_high", "cam_left_wrist", "cam_right_wrist")
MODEL_RUNS = {
    "d097_joint": "can_color_sort_act_joint",
    "d097_task": "can_color_sort_act_task",
    "d150_joint": "can_color_sort_act_joint_aug150",
    "d150_task": "can_color_sort_act_task_aug150",
}
MODEL_ALLOWLIST = (
    "checkpoints/policy_best.ckpt",
    "config.yaml",
    "dataset_stats.pkl",
    "episode_splits.json",
    "metrics/metrics.csv",
    "plots/kl.png",
    "plots/l1.png",
    "plots/learning_rate.png",
    "plots/loss.png",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _text_attr(root, name: str, default: str = "") -> str:
    value = root.attrs.get(name, default)
    return value.decode("utf-8") if isinstance(value, bytes) else str(value)


def inspect_episode(path: Path, *, include_hash: bool) -> dict:
    with h5py.File(path, "r") as root:
        required = (
            "action",
            "observations/qpos",
            "observations/qvel",
            "observations/ee_pose/left",
            "observations/ee_pose/right",
        )
        missing = [name for name in required if name not in root]
        if missing:
            raise ValueError(f"{path.name}: missing datasets {missing}")
        frames = int(root["action"].shape[0])
        if root["action"].shape != (frames, 16):
            raise ValueError(f"{path.name}: action must have shape [T,16]")
        cameras = tuple(sorted(root["observations/images"].keys()))
        missing_cameras = sorted(set(REQUIRED_CAMERAS) - set(cameras))
        if missing_cameras:
            raise ValueError(f"{path.name}: missing cameras {missing_cameras}")
        for camera in REQUIRED_CAMERAS:
            images = root[f"observations/images/{camera}"]
            if len(images) != frames or images.ndim != 4 or images.shape[-1] != 3:
                raise ValueError(f"{path.name}: invalid camera array {camera}")
        success = bool(root.attrs.get("success", False))
        if not success:
            raise ValueError(f"{path.name}: unsuccessful episode is not releasable")
        control_hz = float(root.attrs.get("control_hz", 0.0))
        if control_hz <= 0.0:
            raise ValueError(f"{path.name}: invalid control_hz")
        result = {
            "file": path.name,
            "frames": frames,
            "duration_s": round(frames / control_hz, 3),
            "control_hz": control_hz,
            "success": success,
            "object_variant": _text_attr(root, "object_variant", "unknown"),
            "target_label": _text_attr(root, "target_label", "unknown"),
            "schema_version": _text_attr(root, "schema_version", "missing"),
            "camera_names": ",".join(cameras),
            "size_bytes": path.stat().st_size,
        }
    result["sha256"] = _sha256(path) if include_hash else ""
    return result


def _episode_count(run_dir: Path) -> int:
    with (run_dir / "episode_splits.json").open(encoding="utf-8") as stream:
        splits = json.load(stream)
    episodes = {Path(path).name for paths in splits.values() for path in paths}
    return len(episodes)


def inspect_models(outputs_dir: Path, *, include_hash: bool) -> list[dict]:
    models = []
    for release_name, run_name in MODEL_RUNS.items():
        run_dir = outputs_dir / run_name
        missing = [name for name in MODEL_ALLOWLIST if not (run_dir / name).is_file()]
        if missing:
            raise FileNotFoundError(f"{run_name}: missing release files {missing}")
        with (run_dir / "config.yaml").open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        checkpoint = run_dir / "checkpoints" / "policy_best.ckpt"
        checkpoint_data = torch.load(
            checkpoint,
            map_location="cpu",
            weights_only=True,
            mmap=True,
        )
        if checkpoint_data.get("representation") != config["representation"]:
            raise ValueError(f"{run_name}: checkpoint representation mismatch")
        policy_config = checkpoint_data.get("policy_config", {})
        if policy_config.get("state_dim") != 8 or policy_config.get("action_dim") != 8:
            raise ValueError(f"{run_name}: checkpoint must use an 8D contract")
        if checkpoint_data.get("camera_names") != [
            "cam_high",
            "cam_right_wrist",
        ]:
            raise ValueError(f"{run_name}: unexpected checkpoint cameras")
        if "optimizer" in checkpoint_data:
            raise ValueError(f"{run_name}: best checkpoint contains optimizer state")
        models.append(
            {
                "name": release_name,
                "run_name": run_name,
                "representation": config["representation"],
                "episodes": _episode_count(run_dir),
                "checkpoint_size_bytes": checkpoint.stat().st_size,
                "checkpoint_sha256": _sha256(checkpoint) if include_hash else "",
                "best_epoch": int(checkpoint_data["epoch"]),
                "global_step": int(checkpoint_data["global_step"]),
                "validation_loss": float(checkpoint_data["validation_loss"]),
                "files": list(MODEL_ALLOWLIST),
            }
        )
    return models


def prepare(
    dataset_dir: Path, outputs_dir: Path, release_dir: Path, *, include_hash: bool
):
    episode_paths = sorted(dataset_dir.glob("episode_*.hdf5"))
    if not episode_paths:
        raise FileNotFoundError(f"no episodes found in {dataset_dir}")
    episodes = [
        inspect_episode(path, include_hash=include_hash) for path in episode_paths
    ]
    release_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = release_dir / "dataset_manifest.csv"
    with manifest_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(episodes[0]))
        writer.writeheader()
        writer.writerows(episodes)

    variants = Counter(item["object_variant"] for item in episodes)
    targets = Counter(item["target_label"] for item in episodes)
    summary = {
        "dataset": "can_color_sort",
        "episode_count": len(episodes),
        "success_count": sum(item["success"] for item in episodes),
        "total_frames": sum(item["frames"] for item in episodes),
        "total_duration_s": round(sum(item["duration_s"] for item in episodes), 3),
        "total_size_bytes": sum(item["size_bytes"] for item in episodes),
        "object_variant_counts": dict(sorted(variants.items())),
        "target_label_counts": dict(sorted(targets.items())),
        "required_cameras": list(REQUIRED_CAMERAS),
        "manifest_has_sha256": include_hash,
    }
    (release_dir / "dataset_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    models = inspect_models(outputs_dir, include_hash=include_hash)
    (release_dir / "model_manifest.json").write_text(
        json.dumps({"models": models}, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return summary, models


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-dir", type=Path, default=DEFAULT_DATASET_DIR)
    parser.add_argument("--outputs-dir", type=Path, default=DEFAULT_OUTPUTS_DIR)
    parser.add_argument("--release-dir", type=Path, default=DEFAULT_RELEASE_DIR)
    parser.add_argument(
        "--skip-sha256",
        action="store_true",
        help="skip expensive hashes for a quick local inspection",
    )
    args = parser.parse_args(argv)
    summary, models = prepare(
        args.dataset_dir,
        args.outputs_dir,
        args.release_dir,
        include_hash=not args.skip_sha256,
    )
    print(
        json.dumps({"dataset": summary, "models": models}, indent=2, ensure_ascii=False)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
