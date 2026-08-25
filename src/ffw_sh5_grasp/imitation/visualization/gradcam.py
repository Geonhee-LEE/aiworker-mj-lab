"""Action-targeted Grad-CAM for ACT image observations."""

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
from torch.nn import functional as F

from ..data.episode import load_episode
from ..runtime.runner import ACTPolicyRunner


@dataclass(frozen=True)
class GradCamFrame:
    """One frame of camera heatmaps and the action target they explain."""

    frame: int
    target_value: float
    camera_names: tuple[str, ...]
    images: np.ndarray
    heatmaps: np.ndarray
    heatmap_raw_max: np.ndarray
    gradient_abs_mean: np.ndarray
    predicted_chunk: np.ndarray


def _observation(episode, frame, camera_names):
    missing = set(camera_names) - set(episode.images)
    if missing:
        raise ValueError(f"episode is missing checkpoint cameras: {sorted(missing)}")
    observation = {
        "qpos": episode.qpos[frame],
        "images": {name: episode.images[name][frame] for name in camera_names},
    }
    if episode.ee_pose:
        observation["ee_pose"] = {
            side: values[frame] for side, values in episode.ee_pose.items()
        }
    return observation


def _target_score(
    actions, *, target, chunk_step, action_index, target_sign, action_weights=None
):
    """Turn a continuous ACT chunk into the scalar required by Grad-CAM."""
    if target == "chunk":
        selected = actions if chunk_step is None else actions[:, chunk_step]
        # Weight by the detached prediction itself. This asks which pixels
        # support the complete normalized action prediction without allowing
        # the weights to introduce second-order gradient terms.
        return (selected * selected.detach()).mean() * target_sign
    if target == "linear":
        if chunk_step is None or action_weights is None:
            raise ValueError("linear target requires chunk_step and action_weights")
        weights = torch.as_tensor(
            action_weights, device=actions.device, dtype=actions.dtype
        )
        if weights.shape != (actions.shape[-1],):
            raise ValueError("action_weights must match the policy action dimension")
        return (actions[0, chunk_step] * weights).sum() * target_sign
    if chunk_step is None or action_index is None:
        raise ValueError("action target requires both chunk_step and action_index")
    return actions[0, chunk_step, action_index] * target_sign


def compute_gradcam(
    runner,
    observation,
    *,
    target="chunk",
    chunk_step=None,
    action_index=None,
    target_sign=1.0,
    action_weights=None,
):
    """Compute a separate last-feature Grad-CAM map for every policy camera."""
    if target not in ("chunk", "action", "linear"):
        raise ValueError("Grad-CAM target must be 'chunk', 'action', or 'linear'")
    if chunk_step is not None and not 0 <= chunk_step < runner.config.chunk_size:
        raise ValueError(f"chunk_step must be in [0,{runner.config.chunk_size - 1}]")
    if action_index is not None and not 0 <= action_index < runner.config.action_dim:
        raise ValueError(f"action_index must be in [0,{runner.config.action_dim - 1}]")
    target_sign = float(target_sign)
    if target_sign not in (-1.0, 1.0):
        raise ValueError("target_sign must be -1 or 1")

    qpos, images = runner._inputs(observation)
    activation = None

    def capture(_module, _inputs, output):
        nonlocal activation
        activation = output
        activation.retain_grad()

    handle = runner.policy.image_projection.register_forward_hook(capture)
    runner.policy.zero_grad(set_to_none=True)
    try:
        output = runner.policy(qpos, images)
        normalized_actions = output["actions"]
        score = _target_score(
            normalized_actions,
            target=target,
            chunk_step=chunk_step,
            action_index=action_index,
            target_sign=target_sign,
            action_weights=action_weights,
        )
        score.backward()
    finally:
        handle.remove()

    if activation is None or activation.grad is None:
        raise RuntimeError("failed to capture ACT image feature gradients")
    gradients = activation.grad
    weights = gradients.mean(dim=(2, 3), keepdim=True)
    raw_heatmaps = torch.relu((weights * activation).sum(dim=1, keepdim=True))
    raw_heatmaps = F.interpolate(
        raw_heatmaps,
        size=images.shape[-2:],
        mode="bilinear",
        align_corners=False,
    )[:, 0]
    raw_maxima = raw_heatmaps.flatten(1).max(dim=1).values
    heatmaps = raw_heatmaps / raw_maxima.clamp_min(1e-12)[:, None, None]
    gradient_abs_mean = gradients.abs().mean(dim=(1, 2, 3))

    action_mean = torch.as_tensor(
        runner.stats.action_mean,
        device=normalized_actions.device,
        dtype=normalized_actions.dtype,
    )
    action_std = torch.as_tensor(
        runner.stats.action_std,
        device=normalized_actions.device,
        dtype=normalized_actions.dtype,
    )
    predicted = normalized_actions[0] * action_std + action_mean
    return (
        heatmaps.detach().cpu().numpy().astype(np.float32),
        predicted.detach().cpu().numpy().astype(np.float32),
        float(score.detach().cpu()),
        raw_maxima.detach().cpu().numpy().astype(np.float32),
        gradient_abs_mean.detach().cpu().numpy().astype(np.float32),
    )


def _save_figure(result, path, *, representation, target_description, alpha=0.45):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = len(result.camera_names)
    figure, axes = plt.subplots(rows, 3, figsize=(13.5, 4.1 * rows), squeeze=False)
    color_map = plt.get_cmap("turbo")
    for row, (name, image, heatmap) in enumerate(
        zip(result.camera_names, result.images, result.heatmaps)
    ):
        colored = color_map(heatmap)[..., :3]
        overlay = (1.0 - alpha) * image.astype(np.float32) / 255.0 + alpha * colored
        axes[row, 0].imshow(image)
        axes[row, 0].set_title(f"{name}: RGB")
        axes[row, 1].imshow(heatmap, cmap="turbo", vmin=0.0, vmax=1.0)
        axes[row, 1].set_title("Grad-CAM")
        axes[row, 2].imshow(np.clip(overlay, 0.0, 1.0))
        axes[row, 2].set_title("Overlay")
        for axis in axes[row]:
            axis.axis("off")
    figure.suptitle(
        f"ACT {representation} | episode frame {result.frame} | "
        f"{target_description} | score={result.target_value:.5f}",
        fontsize=12,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    figure.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(figure)


def generate_gradcam(
    checkpoint,
    episode_path,
    *,
    output_dir=None,
    representation="auto",
    device="auto",
    frames=None,
    num_frames=5,
    target="chunk",
    chunk_step=None,
    action_index=None,
    target_sign=1.0,
    alpha=0.45,
):
    """Generate PNG overlays plus lossless arrays for selected episode frames."""
    checkpoint = Path(checkpoint).resolve()
    episode_path = Path(episode_path).resolve()
    episode = load_episode(episode_path)
    runner = ACTPolicyRunner(checkpoint, device=device, representation=representation)
    if frames is None:
        if num_frames <= 0:
            raise ValueError("num_frames must be positive")
        frames = np.linspace(
            0,
            episode.length - 1,
            min(int(num_frames), episode.length),
            dtype=int,
        ).tolist()
    frames = tuple(dict.fromkeys(int(frame) for frame in frames))
    invalid = [frame for frame in frames if not 0 <= frame < episode.length]
    if invalid:
        raise ValueError(
            f"episode frames out of range [0,{episode.length - 1}]: {invalid}"
        )

    output_dir = Path(
        output_dir or (checkpoint.parent.parent / "gradcam" / episode_path.stem)
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    target_description = (
        "normalized action-chunk direction"
        if target == "chunk"
        else f"normalized action[{chunk_step},{action_index}] × {target_sign:+g}"
    )
    metadata = {
        "checkpoint": str(checkpoint),
        "episode": str(episode_path),
        "representation": runner.representation,
        "camera_names": list(runner.camera_names),
        "episode_length": episode.length,
        "frames": list(frames),
        "target": target,
        "chunk_step": chunk_step,
        "action_index": action_index,
        "target_sign": float(target_sign),
        "target_description": target_description,
        "feature_layer": "image_projection",
        "notes": (
            "ACT has no class logit; this Grad-CAM explains a continuous "
            "action target and is not proof of causal feature use."
        ),
    }
    outputs = []
    for frame in frames:
        observation = _observation(episode, frame, runner.camera_names)
        (
            heatmaps,
            predicted_chunk,
            target_value,
            heatmap_raw_max,
            gradient_abs_mean,
        ) = compute_gradcam(
            runner,
            observation,
            target=target,
            chunk_step=chunk_step,
            action_index=action_index,
            target_sign=target_sign,
        )
        images = np.stack([observation["images"][name] for name in runner.camera_names])
        result = GradCamFrame(
            frame=frame,
            target_value=target_value,
            camera_names=runner.camera_names,
            images=images,
            heatmaps=heatmaps,
            heatmap_raw_max=heatmap_raw_max,
            gradient_abs_mean=gradient_abs_mean,
            predicted_chunk=predicted_chunk,
        )
        stem = f"frame_{frame:04d}_{target}"
        png_path = output_dir / f"{stem}.png"
        array_path = output_dir / f"{stem}.npz"
        _save_figure(
            result,
            png_path,
            representation=runner.representation,
            target_description=target_description,
            alpha=alpha,
        )
        np.savez_compressed(
            array_path,
            heatmaps=heatmaps,
            heatmap_raw_max=heatmap_raw_max,
            gradient_abs_mean=gradient_abs_mean,
            predicted_chunk=predicted_chunk,
            target_value=np.asarray(target_value, dtype=np.float32),
            camera_names=np.asarray(runner.camera_names),
            frame=np.asarray(frame, dtype=np.int64),
        )
        outputs.extend((png_path, array_path))
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    outputs.append(output_dir / "metadata.json")
    return outputs


__all__ = ["GradCamFrame", "compute_gradcam", "generate_gradcam"]
