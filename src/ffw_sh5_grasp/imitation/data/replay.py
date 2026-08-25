"""Deterministic action-sequence replay helpers."""

import numpy as np

from .episode import load_episode


def replay_episode(
    env, episode_or_path, *, compare=True, atol=5e-4, step_callback=None
):
    episode = (
        load_episode(episode_or_path)
        if not hasattr(episode_or_path, "action")
        else episode_or_path
    )
    seed = int(episode.attrs.get("seed", -1))
    observation = env.reset(seed=None if seed < 0 else seed)
    qpos = []
    maximum_error = 0.0
    for frame, action in enumerate(episode.action):
        if step_callback is not None and not step_callback(frame, env):
            break
        current = np.asarray(observation["qpos"])
        qpos.append(current.copy())
        if compare:
            maximum_error = max(
                maximum_error, float(np.max(np.abs(current - episode.qpos[frame])))
            )
        observation = env.step(action)
    return {
        "qpos": np.stack(qpos),
        "maximum_qpos_error": maximum_error,
        "reproduced": (not compare) or maximum_error <= atol,
        "final_task": observation["task"],
    }


__all__ = ["replay_episode"]
