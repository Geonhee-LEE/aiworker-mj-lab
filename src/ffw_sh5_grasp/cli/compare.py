"""Overlay expert and policy actions for a recorded episode in Rerun."""

import argparse
from pathlib import Path

from ffw_sh5_grasp.imitation.data.episode import load_episode
from ffw_sh5_grasp.imitation.runtime.runner import ACTPolicyRunner
from ffw_sh5_grasp.imitation.visualization.rerun_rollout import RolloutRerunLogger


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--episode", required=True, type=Path)
    parser.add_argument("--stats", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--device", default="auto")
    args = parser.parse_args(argv)
    episode = load_episode(args.episode)
    runner = ACTPolicyRunner(
        args.checkpoint, args.stats, device=args.device)
    output = args.output or args.episode.with_name(
        args.episode.stem + "_policy_compare.rrd")
    runner.reset()
    with RolloutRerunLogger(
            output, tuple(episode.images),
            application_id="aiworker_expert_policy_compare") as logger:
        for frame in range(episode.length):
            observation = {
                "qpos": episode.qpos[frame],
                "qvel": episode.qvel[frame],
                "images": {name: values[frame]
                           for name, values in episode.images.items()},
                "task": {"success": False, "object_position_error": 0.0},
            }
            action, info = runner.get_action(observation)
            logger.log(
                frame, observation, action,
                predicted_chunk=info["predicted_chunk"],
                expert_action=episode.action[frame])
    print(output)


if __name__ == "__main__":
    main()
