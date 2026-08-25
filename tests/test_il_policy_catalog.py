import pathlib
import sys
import tempfile

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.runtime.catalog import (  # noqa: E402
    discover_policy_runs,
)


def test_policy_catalog_discovers_only_standard_run_checkpoints():
    with tempfile.TemporaryDirectory() as directory:
        output_dir = pathlib.Path(directory)
        checkpoints = output_dir / "run_a" / "checkpoints"
        checkpoints.mkdir(parents=True)
        for name in ("first_policy.ckpt", "policy_last.ckpt", "policy_best.ckpt"):
            (checkpoints / name).touch()
        copied = output_dir / "run_a" / "checkpoints copy"
        copied.mkdir()
        (copied / "ignored.ckpt").touch()
        (output_dir / "not_a_run.ckpt").touch()

        task_checkpoints = output_dir / "run_task" / "checkpoints"
        task_checkpoints.mkdir(parents=True)
        (task_checkpoints / "policy_best.ckpt").touch()
        (output_dir / "run_task" / "config.yaml").write_text(
            "representation: task\n", encoding="utf-8"
        )

        runs = discover_policy_runs(output_dir)

    runs_by_name = {run.name: run for run in runs}
    assert set(runs_by_name) == {"run_a", "run_task"}
    assert runs_by_name["run_a"].representation == "joint"
    assert runs_by_name["run_task"].representation == "task"
    assert [path.name for path in runs_by_name["run_a"].checkpoints] == [
        "policy_best.ckpt",
        "policy_last.ckpt",
        "first_policy.ckpt",
    ]


if __name__ == "__main__":
    test_policy_catalog_discovers_only_standard_run_checkpoints()
    print("PASS")
