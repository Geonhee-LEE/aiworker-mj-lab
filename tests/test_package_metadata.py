from ffw_sh5_grasp import __version__
from ffw_sh5_grasp.cli import COMMANDS


def test_release_version_and_training_commands():
    assert __version__ == "3.1.0"
    assert "train" in COMMANDS
    assert "train-modular" in COMMANDS
    assert "evaluate-color-sort" in COMMANDS
