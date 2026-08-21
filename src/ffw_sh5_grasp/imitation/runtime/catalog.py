"""Discover trained ACT checkpoints under the repository output directory."""

from dataclasses import dataclass
from pathlib import Path

from ...paths import REPO_ROOT


ACT_OUTPUT_DIR = REPO_ROOT / "outputs" / "act"


@dataclass(frozen=True)
class PolicyRun:
    """One training run and its directly contained checkpoint files."""

    name: str
    path: Path
    checkpoints: tuple[Path, ...]


def _checkpoint_key(path):
    priority = {"policy_best.ckpt": 0, "policy_last.ckpt": 1}
    return priority.get(path.name, 2), path.name.lower()


def discover_policy_runs(output_dir=ACT_OUTPUT_DIR):
    """Return newest-first runs found at ``<output_dir>/<run>/checkpoints``."""
    output_dir = Path(output_dir)
    if not output_dir.is_dir():
        return ()
    runs = []
    for run_dir in output_dir.iterdir():
        checkpoint_dir = run_dir / "checkpoints"
        if not run_dir.is_dir() or not checkpoint_dir.is_dir():
            continue
        checkpoints = tuple(sorted(
            (path for path in checkpoint_dir.glob("*.ckpt") if path.is_file()),
            key=_checkpoint_key,
        ))
        if checkpoints:
            runs.append(PolicyRun(run_dir.name, run_dir, checkpoints))
    runs.sort(
        key=lambda run: max(path.stat().st_mtime for path in run.checkpoints),
        reverse=True,
    )
    return tuple(runs)


__all__ = ["ACT_OUTPUT_DIR", "PolicyRun", "discover_policy_runs"]
