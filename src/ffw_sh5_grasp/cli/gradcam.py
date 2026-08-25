"""Generate action-targeted ACT Grad-CAM overlays."""

import argparse
from pathlib import Path

from ffw_sh5_grasp.imitation.visualization.gradcam import generate_gradcam


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Explain a continuous ACT action target with camera-wise Grad-CAM."
        )
    )
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--episode", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--representation", choices=("auto", "joint", "task"), default="auto"
    )
    parser.add_argument("--device", default="auto")
    parser.add_argument("--frames", nargs="+", type=int)
    parser.add_argument("--num-frames", type=int, default=5)
    parser.add_argument(
        "--target",
        choices=("chunk", "action"),
        default="chunk",
        help=(
            "chunk explains the complete normalized prediction; action "
            "explains one --chunk-step/--action-index scalar"
        ),
    )
    parser.add_argument("--chunk-step", type=int)
    parser.add_argument("--action-index", type=int)
    parser.add_argument("--target-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--alpha", type=float, default=0.45)
    args = parser.parse_args(argv)
    if not 0.0 <= args.alpha <= 1.0:
        parser.error("--alpha must be between 0 and 1")
    if args.target == "action" and (
        args.chunk_step is None or args.action_index is None
    ):
        parser.error("--target action requires --chunk-step and --action-index")
    outputs = generate_gradcam(
        args.checkpoint,
        args.episode,
        output_dir=args.output_dir,
        representation=args.representation,
        device=args.device,
        frames=args.frames,
        num_frames=args.num_frames,
        target=args.target,
        chunk_step=args.chunk_step,
        action_index=args.action_index,
        target_sign=args.target_sign,
        alpha=args.alpha,
    )
    for path in outputs:
        print(path)


if __name__ == "__main__":
    main()
