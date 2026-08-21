"""Create a side-by-side RGB video from one recorded episode."""

import argparse
from pathlib import Path

from ffw_sh5_grasp.imitation.data.paths import resolve_episode_path


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", type=Path)
    parser.add_argument("--dataset-dir", type=Path)
    parser.add_argument("--episode-idx", type=int, default=0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    path = resolve_episode_path(
        args.episode, args.dataset_dir, args.episode_idx)
    import imageio.v2 as imageio
    import numpy as np

    from ffw_sh5_grasp.imitation.data.episode import load_episode

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
