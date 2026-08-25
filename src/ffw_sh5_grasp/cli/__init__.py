"""Single command dispatcher for imitation-learning workflows."""

import sys
from importlib import import_module

COMMANDS = {
    "compare": ("compare", "compare expert and ACT actions in Rerun"),
    "evaluate": ("evaluate", "evaluate an ACT checkpoint in MuJoCo"),
    "evaluate-color-sort": (
        "evaluate_color_sort",
        "run the color-sort Joint/Task/PTE matrix",
    ),
    "gradcam": ("gradcam", "explain ACT actions with camera Grad-CAM"),
    "record": ("record", "record demonstration episodes"),
    "replay": ("replay", "replay an episode through MuJoCo physics"),
    "rerun": ("rerun", "write or stream a Rerun episode"),
    "train": ("train", "train a joint- or task-space ACT policy"),
    "train-modular": (
        "train",
        "deprecated alias for 'train'",
    ),
    "validate": ("validate", "validate an HDF5 episode dataset"),
    "visualize": ("visualize", "create an RGB episode video"),
}


def _print_help(stream=sys.stdout):
    print("usage: python3 src/il.py <command> [options]\n", file=stream)
    print("commands:", file=stream)
    width = max(len(command) for command in COMMANDS)
    for command, (_module, description) in COMMANDS.items():
        print(f"  {command:<{width}}  {description}", file=stream)
    print(
        "\nRun 'python3 src/il.py <command> --help' for command options.",
        file=stream,
    )


def main(argv=None):
    """Dispatch one IL subcommand without importing unrelated dependencies."""
    arguments = list(sys.argv[1:] if argv is None else argv)
    if not arguments or arguments[0] in ("-h", "--help"):
        _print_help()
        return 0
    command = arguments.pop(0)
    if command not in COMMANDS:
        print(f"unknown IL command: {command}\n", file=sys.stderr)
        _print_help(sys.stderr)
        return 2
    module_name, _description = COMMANDS[command]
    module = import_module(f"{__name__}.{module_name}")
    result = module.main(arguments)
    return 0 if result is None else result


__all__ = ["COMMANDS", "main"]
