"""Closed-loop MuJoCo evaluation and resumable aggregate metric output."""

import json
import math
import time
from collections import Counter
from pathlib import Path

import numpy as np

from ...config import SETTINGS
from ...control import whole_body
from ..data.schema import ARM_JOINTS
from ..simulation.environment import AIWorkerMujocoEnv
from ..visualization.rerun_rollout import RolloutRerunLogger
from .runner import ACTPolicyRunner
from .task_space import task_action_to_joint


def _json_safe_float(value):
    value = float(value)
    return value if math.isfinite(value) else None


def _mean(values):
    return float(np.mean(values)) if values else None


def _percentile(values, percentile):
    return float(np.percentile(values, percentile)) if values else None


def _wilson_interval(successes, total, z=1.959963984540054):
    if total <= 0:
        return [0.0, 0.0]
    proportion = successes / total
    denominator = 1.0 + z * z / total
    center = (proportion + z * z / (2.0 * total)) / denominator
    radius = (
        z
        * math.sqrt(
            proportion * (1.0 - proportion) / total + z * z / (4.0 * total * total)
        )
        / denominator
    )
    return [max(0.0, center - radius), min(1.0, center + radius)]


def _load_resumable_trials(config_path, trials_path, run_config):
    if not config_path.exists() and not trials_path.exists():
        config_path.parent.mkdir(parents=True, exist_ok=True)
        config_path.write_text(json.dumps(run_config, indent=2), encoding="utf-8")
        return []
    if not config_path.exists():
        raise ValueError(f"cannot resume trials without configuration: {config_path}")
    existing_config = json.loads(config_path.read_text(encoding="utf-8"))
    if existing_config != run_config:
        raise ValueError(
            f"evaluation settings do not match existing run: {config_path}"
        )
    if not trials_path.exists():
        return []
    trials = []
    with trials_path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                trials.append(json.loads(line))
            except json.JSONDecodeError as error:
                raise ValueError(
                    f"invalid trial JSON at {trials_path}:{line_number}"
                ) from error
    expected_indices = list(range(len(trials)))
    actual_indices = [trial.get("episode_index") for trial in trials]
    if actual_indices != expected_indices:
        raise ValueError(f"non-contiguous resumed trials in {trials_path}")
    return trials


def _summarize(results, *, max_steps, control_hz):
    successes = [item for item in results if item["success"]]
    completion_times = [item["completion_time_s"] for item in successes]
    penalized_times = [
        (item["completion_time_s"] if item["success"] else max_steps / control_hz)
        for item in results
    ]
    success_count = len(successes)
    variant_counts = Counter(item["object_variant"] for item in results)
    variant_successes = Counter(item["object_variant"] for item in successes)
    layout_counts = Counter(
        "swapped" if item["bin_colors_swapped"] else "default" for item in results
    )
    layout_successes = Counter(
        "swapped" if item["bin_colors_swapped"] else "default" for item in successes
    )
    ik_errors = [
        value for item in results for value in item.get("ik_position_errors_mm", [])
    ]
    minimum_distances = [
        item["minimum_collision_distance_m"]
        for item in results
        if item.get("minimum_collision_distance_m") is not None
    ]
    return {
        "num_episodes": len(results),
        "success_count": success_count,
        "success_rate": success_count / max(1, len(results)),
        "success_rate_ci95": _wilson_interval(success_count, len(results)),
        "mean_completion_time_s": _mean(completion_times),
        "median_completion_time_s": (
            float(np.median(completion_times)) if completion_times else None
        ),
        "completion_time_iqr_s": (
            [float(value) for value in np.percentile(completion_times, [25, 75])]
            if completion_times
            else None
        ),
        "mean_penalized_time_s": _mean(penalized_times),
        "mean_final_task_error_m": _mean(
            [item["final_task_error_m"] for item in results]
        ),
        "mean_action_delta": _mean([item["mean_action_delta"] for item in results]),
        "mean_policy_inference_ms": _mean(
            [item["mean_policy_inference_ms"] for item in results]
        ),
        "mean_ik_position_error_mm": _mean(ik_errors),
        "p95_ik_position_error_mm": _percentile(ik_errors, 95),
        "minimum_collision_distance_m": (
            min(minimum_distances) if minimum_distances else None
        ),
        "max_collision_constraint_violation": max(
            (item.get("max_collision_constraint_violation", 0.0) for item in results),
            default=0.0,
        ),
        "by_object_variant": {
            name: {
                "trials": count,
                "successes": variant_successes[name],
                "success_rate": variant_successes[name] / count,
            }
            for name, count in sorted(variant_counts.items())
        },
        "by_bin_layout": {
            name: {
                "trials": count,
                "successes": layout_successes[name],
                "success_rate": layout_successes[name] / count,
            }
            for name, count in sorted(layout_counts.items())
        },
    }


def evaluate(
    checkpoint,
    *,
    stats_path=None,
    output_dir=None,
    num_episodes=10,
    max_steps=500,
    seed=1000,
    device="auto",
    rerun=True,
    viewer=False,
    task_name="can_to_box",
    object_variants=None,
    representation="auto",
    proleptic_steps=0,
    temporal_decay=0.05,
    task_ik_speed_scale=None,
    stable_success_steps=10,
    progress=False,
):
    """Evaluate one checkpoint with deterministic, resumable episode seeds."""
    if num_episodes <= 0 or max_steps <= 0:
        raise ValueError("num_episodes and max_steps must be positive")
    if stable_success_steps <= 0:
        raise ValueError("stable_success_steps must be positive")
    checkpoint = Path(checkpoint).resolve()
    output_dir = Path(output_dir or checkpoint.parent.parent / "evaluation").resolve()
    rollout_dir = output_dir / "rollouts"
    rollout_dir.mkdir(parents=True, exist_ok=True)
    object_variants = (
        None
        if object_variants is None
        else tuple(dict.fromkeys(str(name) for name in object_variants))
    )
    if object_variants == ():
        raise ValueError("object_variants must not be empty")
    runner = ACTPolicyRunner(
        checkpoint,
        stats_path,
        device=device,
        representation=representation,
        proleptic_steps=proleptic_steps,
        temporal_decay=temporal_decay,
    )
    speed_scale = float(
        SETTINGS.number("imitation.policy.task_ik_speed_scale", positive=True)
        if task_ik_speed_scale is None
        else task_ik_speed_scale
    )
    if not math.isfinite(speed_scale) or speed_scale <= 0.0:
        raise ValueError("task IK speed scale must be finite and positive")

    run_config = {
        "checkpoint": str(checkpoint),
        "stats_path": None if stats_path is None else str(Path(stats_path).resolve()),
        "num_episodes": int(num_episodes),
        "max_steps": int(max_steps),
        "seed": int(seed),
        "task_name": str(task_name),
        "representation": runner.representation,
        "proleptic_steps": int(runner.proleptic_steps),
        "temporal_decay": float(temporal_decay),
        "task_ik_speed_scale": speed_scale,
        "stable_success_steps": int(stable_success_steps),
        "camera_names": list(runner.camera_names),
        "chunk_size": int(runner.config.chunk_size),
    }
    if object_variants is not None:
        run_config["object_variants"] = list(object_variants)
    config_path = output_dir / "evaluation_config.json"
    trials_path = output_dir / "trials.jsonl"
    results = _load_resumable_trials(config_path, trials_path, run_config)
    if len(results) > num_episodes:
        raise ValueError("resumed run contains more trials than requested")

    with AIWorkerMujocoEnv(
        render_images=True,
        camera_names=runner.camera_names,
        task_name=task_name,
        object_variants=object_variants,
        randomize_bin_colors=task_name == "can_color_sort",
    ) as env:
        solver = None
        if runner.representation == "task":
            # The environment enables task-bin collisions before the solver
            # catalogs collision pairs, matching Teleop initialization order.
            solver = whole_body.WholeBodyIK(
                env.model,
                {"r": "grasp_target_r", "l": "grasp_target_l"},
                ARM_JOINTS,
            )
        viewer_context = None
        if viewer:
            import mujoco.viewer

            viewer_context = mujoco.viewer.launch_passive(env.model, env.data)
            viewer_handle = viewer_context.__enter__()
        try:
            for episode_index in range(len(results), num_episodes):
                episode_seed = int(seed) + episode_index
                observation = env.reset(seed=episode_seed)
                runner.reset()
                if solver is not None:
                    solver.rebase(env.data)
                actions = []
                inference_times_ms = []
                ik_position_errors_mm = []
                minimum_collision_distance = math.inf
                max_collision_violation = 0.0
                success_streak = 0
                path = rollout_dir / f"rollout_{episode_index:04d}.rrd"
                with RolloutRerunLogger(
                    path, env.camera_names, enabled=rerun
                ) as logger:
                    for frame in range(max_steps):
                        if viewer and not viewer_handle.is_running():
                            break
                        frame_start = time.perf_counter()
                        action, policy_info = runner.get_action(observation)
                        inference_times_ms.append(
                            (time.perf_counter() - frame_start) * 1000.0
                        )
                        task_action = None
                        ik_metrics = None
                        if runner.representation == "task":
                            task_action = action.copy()
                            action, diagnostics = task_action_to_joint(
                                env, solver, task_action, speed_scale=speed_scale
                            )
                            ik_position_errors_mm.append(diagnostics.position_error_mm)
                            if math.isfinite(diagnostics.minimum_collision_distance_m):
                                minimum_collision_distance = min(
                                    minimum_collision_distance,
                                    diagnostics.minimum_collision_distance_m,
                                )
                            max_collision_violation = max(
                                max_collision_violation,
                                diagnostics.collision_constraint_violation,
                            )
                            ik_metrics = {
                                "position_error_mm": (diagnostics.position_error_mm),
                                "speed_scale": speed_scale,
                                "collision_constraint_violation": (
                                    diagnostics.collision_constraint_violation
                                ),
                            }
                            if math.isfinite(diagnostics.minimum_collision_distance_m):
                                ik_metrics["minimum_collision_distance_m"] = (
                                    diagnostics.minimum_collision_distance_m
                                )
                        action = env.prepare_action(action)
                        logger.log(
                            frame,
                            observation,
                            action,
                            predicted_chunk=policy_info["predicted_chunk"],
                            task_action=task_action,
                            representation=runner.representation,
                            temporal_metrics={
                                "proleptic_steps": policy_info["proleptic_steps"],
                                "target_timestep": policy_info["target_timestep"],
                                "ensemble_candidate_count": policy_info[
                                    "ensemble_candidate_count"
                                ],
                            },
                            ik_metrics=ik_metrics,
                        )
                        actions.append(action)
                        observation = env.step(action)
                        success_streak = (
                            success_streak + 1 if observation["task"]["success"] else 0
                        )
                        if viewer:
                            viewer_handle.sync()
                        if success_streak >= stable_success_steps:
                            break

                actions = np.asarray(actions)
                deltas = (
                    np.diff(actions, axis=0) if len(actions) > 1 else np.zeros((0, 16))
                )
                succeeded = success_streak >= stable_success_steps
                steps_executed = len(actions)
                trial = {
                    "episode_index": episode_index,
                    "seed": episode_seed,
                    "object_variant": observation["task"]["object_variant"],
                    "target_label": observation["task"]["target_label"],
                    "bin_colors_swapped": bool(env.task.bin_colors_swapped),
                    "bin_color_layout": env.task.bin_color_layout,
                    "success": succeeded,
                    "steps": steps_executed,
                    "completion_time_s": steps_executed / env.actual_control_hz,
                    "final_task_error_m": float(
                        observation["task"]["object_position_error"]
                    ),
                    "mean_action_delta": (
                        float(np.linalg.norm(deltas, axis=1).mean())
                        if len(deltas)
                        else 0.0
                    ),
                    "mean_policy_inference_ms": _mean(inference_times_ms),
                    "ik_position_errors_mm": ik_position_errors_mm,
                    "mean_ik_position_error_mm": _mean(ik_position_errors_mm),
                    "p95_ik_position_error_mm": _percentile(ik_position_errors_mm, 95),
                    "minimum_collision_distance_m": _json_safe_float(
                        minimum_collision_distance
                    ),
                    "max_collision_constraint_violation": float(
                        max_collision_violation
                    ),
                }
                with trials_path.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(trial) + "\n")
                results.append(trial)
                if progress:
                    print(
                        f"[{runner.representation} f={runner.proleptic_steps}] "
                        f"episode={episode_index + 1:03d}/{num_episodes} "
                        f"seed={episode_seed} "
                        f"can={trial['object_variant']} "
                        f"layout={'swap' if trial['bin_colors_swapped'] else 'default'} "
                        f"success={succeeded} "
                        f"time={trial['completion_time_s']:.2f}s",
                        flush=True,
                    )
        finally:
            if viewer_context is not None:
                viewer_context.__exit__(None, None, None)

        summary = _summarize(
            results, max_steps=max_steps, control_hz=env.actual_control_hz
        )
        evaluation = {
            **run_config,
            "control_hz": env.actual_control_hz,
            **summary,
            "episodes": results,
        }
    (output_dir / "evaluation.json").write_text(
        json.dumps(evaluation, indent=2), encoding="utf-8"
    )
    return evaluation


__all__ = ["evaluate"]
