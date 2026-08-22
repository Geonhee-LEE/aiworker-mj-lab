"""Replay a recorded action sequence in deterministic MuJoCo physics."""

import argparse
import json
import time
from pathlib import Path

from ffw_sh5_grasp.imitation.data.paths import resolve_episode_path


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", type=Path)
    parser.add_argument("--dataset-dir", type=Path)
    parser.add_argument("--episode-idx", type=int, default=0)
    parser.add_argument("--atol", type=float, default=5e-4)
    parser.add_argument("--viewer", action="store_true",
                        help="show the replayed robot in the MuJoCo viewer")
    args = parser.parse_args(argv)
    path = resolve_episode_path(
        args.episode, args.dataset_dir, args.episode_idx)
    from ffw_sh5_grasp.imitation.data.episode import load_episode
    from ffw_sh5_grasp.imitation.data.replay import replay_episode
    from ffw_sh5_grasp.imitation.simulation.environment import AIWorkerMujocoEnv

    episode = load_episode(path)
    task_name = episode.attrs.get(
        "scenario_name", episode.attrs.get("task_name", "can_to_box"))
    with AIWorkerMujocoEnv(
            render_images=False, task_name=task_name) as env:
        if args.viewer:
            import mujoco.viewer
            with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
                viewer.sync()

                def show_frame(_frame, simulation):
                    if not viewer.is_running():
                        return False
                    viewer.sync()
                    time.sleep(1.0 / simulation.actual_control_hz)
                    return True

                result = replay_episode(
                    env, episode, atol=args.atol, step_callback=show_frame)
                viewer.sync()
        else:
            result = replay_episode(env, episode, atol=args.atol)
    printable = {key: value for key, value in result.items() if key != "qpos"}
    print(json.dumps(printable, indent=2))
    if not result["reproduced"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
