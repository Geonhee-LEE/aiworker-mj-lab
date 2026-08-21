"""ALOHA-style imitation-learning components for FFW-SH5.

The package root stays dependency-light. Import concrete functionality from
``data``, ``simulation``, ``act``, ``runtime``, or ``apps``.
"""

from .data.schema import ACTION_DIM, ACTION_NAMES

__all__ = ["ACTION_DIM", "ACTION_NAMES"]
