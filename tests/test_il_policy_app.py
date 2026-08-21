import pathlib
import sys
import tempfile

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.apps.policy import ACTPolicyApp  # noqa: E402
from ffw_sh5_grasp.imitation.runtime.catalog import (  # noqa: E402
    discover_policy_runs,
)


class _Runner:
    def __init__(self):
        self.reset_count = 0

    def get_action(self, _observation):
        return np.zeros(16), {"predicted_chunk": np.zeros((2, 16))}

    def reset(self):
        self.reset_count += 1


class _Env:
    def __init__(self, success_at=None):
        self.success_at = success_at
        self.step_count = 0
        self.last_action = np.zeros(16)

    def reset(self):
        self.step_count = 0
        self.last_action = np.zeros(16)
        return {"task": {"success": False}}

    def prepare_action(self, action):
        return action

    def step(self, _action):
        self.step_count += 1
        return {
            "task": {
                "success": self.success_at == self.step_count,
            },
        }


def _app(max_steps=2, success_at=None):
    app = ACTPolicyApp.__new__(ACTPolicyApp)
    app.runner = _Runner()
    app.env = _Env(success_at=success_at)
    app.observation = app.env.reset()
    app.max_steps = max_steps
    app.frame = 0
    app.running = True
    app.stop_reason = None
    app.last_action = app.env.last_action.copy()
    app.last_policy_info = None
    return app


def test_policy_rollout_stops_at_max_steps_and_restarts():
    app = _app(max_steps=2)

    assert app.step_policy()
    assert app.running
    assert app.step_policy()
    assert not app.running
    assert app.frame == 2
    assert app.stop_reason == "max steps reached"
    assert not app.step_policy()

    app.start_policy()
    assert app.running
    assert app.frame == 0
    assert app.runner.reset_count == 1


def test_policy_rollout_continues_after_can_is_placed():
    app = _app(max_steps=2, success_at=1)

    assert app.step_policy()
    assert app.running
    assert app.frame == 1
    assert app.observation["task"]["success"]
    assert app.step_policy()
    assert not app.running
    assert app.stop_reason == "max steps reached"


def test_policy_catalog_discovers_only_standard_run_checkpoints():
    with tempfile.TemporaryDirectory() as directory:
        output_dir = pathlib.Path(directory)
        checkpoints = output_dir / "run_a" / "checkpoints"
        checkpoints.mkdir(parents=True)
        for name in ("first_policy.ckpt", "policy_last.ckpt",
                     "policy_best.ckpt"):
            (checkpoints / name).touch()
        copied = output_dir / "run_a" / "checkpoints copy"
        copied.mkdir()
        (copied / "ignored.ckpt").touch()
        (output_dir / "not_a_run.ckpt").touch()

        runs = discover_policy_runs(output_dir)

    assert [run.name for run in runs] == ["run_a"]
    assert [path.name for path in runs[0].checkpoints] == [
        "policy_best.ckpt", "policy_last.ckpt", "first_policy.ckpt",
    ]


if __name__ == "__main__":
    test_policy_rollout_stops_at_max_steps_and_restarts()
    test_policy_rollout_continues_after_can_is_placed()
    test_policy_catalog_discovers_only_standard_run_checkpoints()
    print("PASS")
