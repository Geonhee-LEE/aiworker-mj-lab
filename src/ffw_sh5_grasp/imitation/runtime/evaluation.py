"""Closed-loop MuJoCo evaluation and aggregate metric output."""

import json
import time
from pathlib import Path

import numpy as np

from ..simulation.environment import AIWorkerMujocoEnv
from ..visualization.rerun_rollout import RolloutRerunLogger
from .runner import ACTPolicyRunner


def evaluate(checkpoint, *, stats_path=None, output_dir=None, num_episodes=10,
             max_steps=500, seed=1000, device="auto", rerun=True,
             viewer=False):
    if num_episodes <= 0 or max_steps <= 0:
        raise ValueError("num_episodes and max_steps must be positive")
    checkpoint = Path(checkpoint)
    output_dir = Path(output_dir or checkpoint.parent.parent / "evaluation")
    rollout_dir = output_dir / "rollouts"
    rollout_dir.mkdir(parents=True, exist_ok=True)
    runner = ACTPolicyRunner(checkpoint, stats_path, device=device)
    results = []
    with AIWorkerMujocoEnv(
            render_images=True, camera_names=runner.camera_names) as env:
        viewer_context = None
        if viewer:
            import mujoco.viewer
            viewer_context = mujoco.viewer.launch_passive(env.model, env.data)
            viewer_handle = viewer_context.__enter__()
        for episode_index in range(num_episodes):
            observation = env.reset(seed=seed + episode_index)
            runner.reset()
            actions = []
            path = rollout_dir / f"rollout_{episode_index:04d}.rrd"
            with RolloutRerunLogger(
                    path, env.camera_names, enabled=rerun) as logger:
                for frame in range(max_steps):
                    if viewer and not viewer_handle.is_running():
                        break
                    frame_start = time.perf_counter()
                    action, policy_info = runner.get_action(observation)
                    action = env.prepare_action(action)
                    logger.log(
                        frame, observation, action,
                        predicted_chunk=policy_info["predicted_chunk"])
                    actions.append(action)
                    observation = env.step(action)
                    if viewer:
                        viewer_handle.sync()
                        remaining = (1.0 / env.actual_control_hz) - (
                            time.perf_counter() - frame_start)
                        if remaining > 0:
                            time.sleep(remaining)
            actions = np.asarray(actions)
            deltas = np.diff(actions, axis=0) if len(actions) > 1 else np.zeros((0, 16))
            results.append({
                "success": bool(observation["task"]["success"]),
                "episode_length": len(actions),
                "final_task_error": observation["task"]["object_position_error"],
                "mean_action_magnitude": float(np.linalg.norm(actions, axis=1).mean()),
                "mean_action_delta": float(
                    np.linalg.norm(deltas, axis=1).mean()) if len(deltas) else 0.0,
            })
        if viewer_context is not None:
            viewer_context.__exit__(None, None, None)
        success_count = sum(item["success"] for item in results)
    evaluation = {
        "num_episodes": len(results),
        "success_count": success_count,
        "success_rate": success_count / max(1, len(results)),
        "mean_episode_length": float(np.mean([
            item["episode_length"] for item in results])),
        "mean_final_task_error": float(np.mean([
            item["final_task_error"] for item in results])),
        "mean_action_magnitude": float(np.mean([
            item["mean_action_magnitude"] for item in results])),
        "mean_action_delta": float(np.mean([
            item["mean_action_delta"] for item in results])),
        "episodes": results,
    }
    with (output_dir / "evaluation.json").open("w", encoding="utf-8") as stream:
        json.dump(evaluation, stream, indent=2)
    return evaluation


__all__ = ["evaluate"]
