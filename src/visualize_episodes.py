#!/usr/bin/env python3
"""Create a side-by-side RGB video from one recorded episode."""

import argparse
from pathlib import Path

import imageio.v2 as imageio
import numpy as np

from ffw_sh5_grasp.imitation.dataset import load_episode


def resolve_episode(dataset_dir, episode_idx):
    return Path(dataset_dir) / f"episode_{episode_idx:06d}.hdf5"


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", type=Path)
    parser.add_argument("--dataset-dir", type=Path)
    parser.add_argument("--episode-idx", type=int, default=0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    path = args.episode or resolve_episode(args.dataset_dir, args.episode_idx)
    episode = load_episode(path)
    output = args.output or path.with_suffix(".mp4")
    fps = float(episode.attrs.get("control_hz", 25.0))
    camera_names = list(episode.images)
    if not camera_names:
        raise ValueError("episode has no RGB cameras")
    with imageio.get_writer(output, fps=fps, codec="libx264", quality=8) as writer:
        for frame in range(episode.length):
            images = [episode.images[name][frame] for name in camera_names]
            target_height = max(image.shape[0] for image in images)
            padded = [np.pad(
                image, ((0, target_height - image.shape[0]), (0, 0), (0, 0)))
                for image in images]
            writer.append_data(np.concatenate(padded, axis=1))
    print(output)


if __name__ == "__main__":
    main()
