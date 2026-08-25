"""Discover trained ACT checkpoints under the repository output directory."""

from dataclasses import dataclass
from pathlib import Path

import yaml

from ...paths import REPO_ROOT

ACT_OUTPUT_DIR = REPO_ROOT / "outputs" / "act"
ACT_MODULAR_OUTPUT_DIR = REPO_ROOT / "outputs" / "act_modular"
ACT_OUTPUT_DIRS = (ACT_OUTPUT_DIR, ACT_MODULAR_OUTPUT_DIR)


@dataclass(frozen=True)
class PolicyRun:
    """One training run and its directly contained checkpoint files."""

    name: str
    path: Path
    checkpoints: tuple[Path, ...]
    representation: str = "joint"


def _checkpoint_key(path):
    priority = {"policy_best.ckpt": 0, "policy_last.ckpt": 1}
    return priority.get(path.name, 2), path.name.lower()


def _run_representation(run_dir):
    """Read cheap run metadata without loading a potentially large checkpoint."""
    config_path = Path(run_dir) / "config.yaml"
    if not config_path.is_file():
        return "joint"
    try:
        with config_path.open("r", encoding="utf-8") as stream:
            values = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError):
        return "joint"
    if not isinstance(values, dict):
        return "joint"
    representation = str(values.get("representation", "joint")).lower()
    return representation if representation in ("joint", "task") else "joint"


def discover_policy_runs(output_dir=None):
    """Return newest-first runs from standard and modular output roots.

    Passing ``output_dir`` preserves the previous single-root behavior, which
    is useful to inspect an isolated training directory in tests and tools.
    """
    output_dirs = ACT_OUTPUT_DIRS if output_dir is None else (Path(output_dir),)
    runs = []
    for root in output_dirs:
        if not root.is_dir():
            continue
        for run_dir in root.iterdir():
            checkpoint_dir = run_dir / "checkpoints"
            if not run_dir.is_dir() or not checkpoint_dir.is_dir():
                continue
            checkpoints = tuple(
                sorted(
                    (path for path in checkpoint_dir.glob("*.ckpt") if path.is_file()),
                    key=_checkpoint_key,
                )
            )
            if checkpoints:
                runs.append(
                    PolicyRun(
                        run_dir.name, run_dir, checkpoints, _run_representation(run_dir)
                    )
                )
    runs.sort(
        key=lambda run: max(path.stat().st_mtime for path in run.checkpoints),
        reverse=True,
    )
    return tuple(runs)


__all__ = [
    "ACT_MODULAR_OUTPUT_DIR",
    "ACT_OUTPUT_DIR",
    "ACT_OUTPUT_DIRS",
    "PolicyRun",
    "discover_policy_runs",
]
