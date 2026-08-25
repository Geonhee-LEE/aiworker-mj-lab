"""Optional imitation-learning dependencies stay outside basic teleop imports."""

import json
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = REPO_ROOT / "src"


def _fresh_import(script):
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(SRC_DIR)
    return subprocess.check_output(
        [sys.executable, "-c", script],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
    ).strip()


def test_basic_teleop_import_skips_imitation_optionals():
    output = _fresh_import(
        "import json, sys; "
        "import ffw_sh5_grasp.application.teleop; "
        "print(json.dumps({name: name in sys.modules for name in "
        "('h5py', 'torch', 'torchvision', 'matplotlib', 'rerun', 'wandb')}))"
    )
    assert json.loads(output) == {
        "h5py": False,
        "torch": False,
        "torchvision": False,
        "matplotlib": False,
        "rerun": False,
        "wandb": False,
    }


def test_data_public_api_loads_hdf5_only_when_requested():
    output = _fresh_import(
        "import json, sys; "
        "import ffw_sh5_grasp.imitation.data as data; "
        "before = 'h5py' in sys.modules; "
        "visible = 'EpisodeData' in dir(data); "
        "episode_type = data.EpisodeData; "
        "print(json.dumps([before, visible, 'h5py' in sys.modules, "
        "episode_type.__name__]))"
    )
    assert json.loads(output) == [False, True, True, "EpisodeData"]
