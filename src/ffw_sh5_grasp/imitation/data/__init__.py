"""Episode schema, persistence, validation, recording, and replay helpers.

This package is intentionally independent of MuJoCo and PyTorch so dataset
inspection tools stay lightweight. HDF5 episode I/O is imported lazily so the
teleoperation runtime can use the shared schema without requiring ``h5py``.
"""

from importlib import import_module

from .schema import ACTION_DIM, ACTION_NAMES

_LAZY_EXPORTS = {
    "EpisodeData": (".episode", "EpisodeData"),
    "load_episode": (".episode", "load_episode"),
    "resolve_episode_path": (".paths", "resolve_episode_path"),
    "write_episode": (".episode", "write_episode"),
}


def __getattr__(name):
    """Load optional episode-I/O exports only when callers request them."""
    target = _LAZY_EXPORTS.get(name)
    if target is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attribute_name = target
    value = getattr(import_module(module_name, __name__), attribute_name)
    globals()[name] = value
    return value


def __dir__():
    """Expose lazy public names to interactive help and IDE inspection."""
    return sorted(set(globals()) | set(_LAZY_EXPORTS))

__all__ = [
    "ACTION_DIM",
    "ACTION_NAMES",
    "EpisodeData",
    "load_episode",
    "resolve_episode_path",
    "write_episode",
]
