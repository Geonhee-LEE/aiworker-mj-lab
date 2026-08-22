from ffw_sh5_grasp import __version__
from ffw_sh5_grasp.cli import COMMANDS


def test_release_version_and_modular_commands():
    assert __version__ == "3.0.0"
    assert "train-modular" in COMMANDS
    assert "evaluate-color-sort" in COMMANDS
