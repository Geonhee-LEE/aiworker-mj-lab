#!/usr/bin/env python3
"""Replay a recorded action sequence in deterministic MuJoCo physics."""

import argparse
import json
from pathlib import Path

from ffw_sh5_grasp.imitation.mujoco_env import AIWorkerMujocoEnv
from ffw_sh5_grasp.imitation.replay import replay_episode


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", type=Path)
    parser.add_argument("--dataset-dir", type=Path)
    parser.add_argument("--episode-idx", type=int, default=0)
    parser.add_argument("--atol", type=float, default=5e-4)
    args = parser.parse_args(argv)
    path = args.episode or (
        args.dataset_dir / f"episode_{args.episode_idx:06d}.hdf5")
    with AIWorkerMujocoEnv(render_images=False) as env:
        result = replay_episode(env, path, atol=args.atol)
    printable = {key: value for key, value in result.items() if key != "qpos"}
    print(json.dumps(printable, indent=2))
    if not result["reproduced"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
