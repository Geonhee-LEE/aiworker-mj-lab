"""Create green-can D150 Joint/Task F=5/10/15/20 closed-loop GIFs."""

from __future__ import annotations

import argparse
import gc
import hashlib
import json
import tempfile
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont

from ffw_sh5_grasp.control import whole_body
from ffw_sh5_grasp.imitation.data.schema import ARM_JOINTS
from ffw_sh5_grasp.imitation.runtime.runner import ACTPolicyRunner
from ffw_sh5_grasp.imitation.runtime.task_space import task_action_to_joint
from ffw_sh5_grasp.imitation.simulation.environment import AIWorkerMujocoEnv

SEED = 195958
MAX_FRAME = 300
CONTROL_HZ = 25.0
F_VALUES = (5, 10, 15, 20)
CAMERA_NAMES = ("cam_high", "cam_right_wrist")
DISPLAY_CAMERA = "external_observer"
OBSERVER_WIDTH = 640
OBSERVER_HEIGHT = 576
OBSERVER_LOOKAT = (0.35, 0.0, 1.00)
OBSERVER_DISTANCE = 1.3
OBSERVER_AZIMUTH_DEG = 180.0
OBSERVER_ELEVATION_DEG = -15.0
STABLE_SUCCESS_STEPS = 10
TASK_IK_SPEED_SCALE = 3.0
POLICIES = (
    (
        "d150_joint",
        "D150 Joint",
        "joint",
        Path(
            "outputs/act_modular/can_color_sort_act_joint_aug150/"
            "checkpoints/policy_best.ckpt"
        ),
    ),
    (
        "d150_task",
        "D150 Task",
        "task",
        Path(
            "outputs/act_modular/can_color_sort_act_task_aug150/"
            "checkpoints/policy_best.ckpt"
        ),
    ),
)


def array_hash(*arrays):
    digest = hashlib.sha256()
    for value in arrays:
        array = np.ascontiguousarray(value)
        digest.update(str(array.dtype).encode())
        digest.update(str(array.shape).encode())
        digest.update(array.tobytes())
    return digest.hexdigest()


def font(size):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except OSError:
        return ImageFont.load_default()


def annotated_frame(image, *, label, f_value, frame, stable_success):
    image = Image.fromarray(np.asarray(image, dtype=np.uint8), mode="RGB")
    header_height = 34
    canvas = Image.new("RGB", (image.width, image.height + header_height), "black")
    canvas.paste(image, (0, header_height))
    draw = ImageDraw.Draw(canvas)
    future_s = f_value / CONTROL_HZ
    suffix = " | SUCCESS" if stable_success else ""
    draw.text(
        (8, 6),
        f"{label} | F={f_value} ({future_s:.1f}s) | frame {frame:03d}{suffix}",
        fill=(80, 255, 120) if stable_success else "white",
        font=font(16),
    )
    return np.asarray(canvas)


def rollout(
    repo,
    temporary_dir,
    output_dir,
    policy_key,
    label,
    representation,
    checkpoint_relative,
    f_value,
):
    checkpoint = (repo / checkpoint_relative).resolve()
    runner = ACTPolicyRunner(
        checkpoint,
        device="auto",
        representation=representation,
        proleptic_steps=f_value,
    )
    mmap_path = temporary_dir / f"{policy_key}_f_{f_value:02d}.npy"
    frames = np.lib.format.open_memmap(
        mmap_path,
        mode="w+",
        dtype=np.uint8,
        shape=(MAX_FRAME + 1, OBSERVER_HEIGHT, OBSERVER_WIDTH, 3),
    )
    gif_path = output_dir / f"{policy_key}_f_{f_value:02d}_external_observer.gif"
    success_streak = 0
    stable_success = False
    normal_success_step = None
    can_positions = []
    stable_history = []

    with AIWorkerMujocoEnv(
        render_images=True,
        camera_names=CAMERA_NAMES,
        task_name="can_color_sort",
        object_variants=("green",),
        randomize_bin_colors=True,
    ) as env:
        solver = None
        if representation == "task":
            solver = whole_body.WholeBodyIK(
                env.model,
                {"r": "grasp_target_r", "l": "grasp_target_l"},
                ARM_JOINTS,
            )
        observation = env.reset(seed=SEED)
        runner.reset()
        if solver is not None:
            solver.rebase(env.data)
        observer_renderer = mujoco.Renderer(
            env.model, height=OBSERVER_HEIGHT, width=OBSERVER_WIDTH
        )
        observer_camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(env.model, observer_camera)
        observer_camera.lookat[:] = OBSERVER_LOOKAT
        observer_camera.distance = OBSERVER_DISTANCE
        observer_camera.azimuth = OBSERVER_AZIMUTH_DEG
        observer_camera.elevation = OBSERVER_ELEVATION_DEG
        observer_option = mujoco.MjvOption()
        # The rounded head housing is a visual-only mesh in geom group 4.
        # MjvOption defaults groups 3..5 to hidden; the operator GUI's F4-style
        # view enables group 4, so the external observer must do the same.
        observer_option.geomgroup[4] = 1
        initial_state_hash = array_hash(
            observation["debug"]["full_qpos"],
            observation["debug"]["full_qvel"],
            observation["debug"]["task_object_pose"],
            observation["debug"]["target_position"],
        )
        if observation["task"]["object_variant"] != "green":
            raise ValueError("environment did not produce the requested green can")
        if observation["task"]["target_label"] != "blue":
            raise ValueError("green can no longer targets the blue bin")
        if not env.task.bin_colors_swapped:
            raise ValueError("seed no longer produces swapped bins")

        with imageio.get_writer(
            gif_path,
            mode="I",
            duration=round(1000.0 / CONTROL_HZ),
            loop=0,
            palettesize=256,
            subrectangles=True,
        ) as writer:
            for frame in range(MAX_FRAME + 1):
                observer_renderer.update_scene(
                    env.data,
                    camera=observer_camera,
                    scene_option=observer_option,
                )
                rgb = np.asarray(observer_renderer.render()).copy()
                frames[frame] = rgb
                stable_history.append(stable_success)
                can_positions.append(
                    np.asarray(observation["debug"]["task_object_pose"][:3]).copy()
                )
                writer.append_data(
                    annotated_frame(
                        rgb,
                        label=label,
                        f_value=f_value,
                        frame=frame,
                        stable_success=stable_success,
                    )
                )
                if frame == MAX_FRAME:
                    final_observation = observation
                    break
                action, _policy_info = runner.get_action(observation)
                if representation == "task":
                    action, _diagnostics = task_action_to_joint(
                        env,
                        solver,
                        action,
                        speed_scale=TASK_IK_SPEED_SCALE,
                    )
                observation = env.step(env.prepare_action(action))
                if observation["task"]["success"]:
                    success_streak += 1
                else:
                    success_streak = 0
                if not stable_success and success_streak >= STABLE_SUCCESS_STEPS:
                    stable_success = True
                    normal_success_step = frame + 1
        observer_renderer.close()

        final_can_position = np.asarray(
            final_observation["debug"]["task_object_pose"][:3]
        ).copy()
        target_position = np.asarray(
            final_observation["debug"]["target_position"]
        ).copy()
        summary = {
            "policy": policy_key,
            "label": label,
            "representation": representation,
            "checkpoint": str(checkpoint),
            "f_steps": f_value,
            "lookahead_s": f_value / env.actual_control_hz,
            "seed": SEED,
            "camera": DISPLAY_CAMERA,
            "control_hz": env.actual_control_hz,
            "normal_evaluation_success": normal_success_step is not None,
            "normal_success_step": normal_success_step,
            "final_instantaneous_success": bool(final_observation["task"]["success"]),
            "final_can_position": final_can_position.tolist(),
            "target_position": target_position.tolist(),
            "final_position_error_m": float(
                np.linalg.norm(final_can_position - target_position)
            ),
            "initial_state_sha256": initial_state_hash,
            "object_variant": final_observation["task"]["object_variant"],
            "target_label": final_observation["task"]["target_label"],
            "bin_colors_swapped": bool(env.task.bin_colors_swapped),
            "gif": str(gif_path.resolve()),
        }
    frames.flush()
    del frames, runner
    gc.collect()
    torch.cuda.empty_cache()
    return summary, mmap_path, np.asarray(can_positions), np.asarray(stable_history)


def comparison_frame(frame_arrays, summaries, frame):
    cell_width = 320
    cell_height = 288
    header_height = 32
    rows = len(POLICIES)
    columns = len(F_VALUES)
    canvas = Image.new(
        "RGB",
        (cell_width * columns, (cell_height + header_height) * rows),
        (20, 20, 20),
    )
    draw = ImageDraw.Draw(canvas)
    for row, (policy_key, label, _representation, _checkpoint) in enumerate(POLICIES):
        for column, f_value in enumerate(F_VALUES):
            key = (policy_key, f_value)
            source = Image.fromarray(frame_arrays[key][frame], mode="RGB")
            source = source.resize((cell_width, cell_height), Image.Resampling.LANCZOS)
            x = column * cell_width
            y = row * (cell_height + header_height)
            canvas.paste(source, (x, y + header_height))
            success_step = summaries[key]["normal_success_step"]
            succeeded = success_step is not None and frame >= success_step
            short_label = label.removeprefix("D150 ")
            text = f"{short_label} | F{f_value}"
            if succeeded:
                text += " | SUCCESS"
            else:
                text += f" | {f_value / CONTROL_HZ:.1f}s"
            draw.text(
                (x + 5, y + 5),
                text,
                fill=(80, 255, 120) if succeeded else "white",
                font=font(15),
            )
    return np.asarray(canvas)


def write_comparison(output_dir, temporary_paths, summaries):
    frame_arrays = {
        key: np.load(path, mmap_mode="r") for key, path in temporary_paths.items()
    }
    f_label = "_".join(f"{value:02d}" for value in F_VALUES)
    output_path = output_dir / f"d150_joint_task_f_{f_label}_external_observer.gif"
    with imageio.get_writer(
        output_path,
        mode="I",
        duration=round(1000.0 / CONTROL_HZ),
        loop=0,
        palettesize=256,
        subrectangles=True,
    ) as writer:
        for frame in range(MAX_FRAME + 1):
            writer.append_data(comparison_frame(frame_arrays, summaries, frame))
    return output_path


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(
            "outputs/analysis/closed_loop_gradcam_ee_y_seed195958_20260825_r2/"
            "d150_pte_gifs_f05_10_15_20_seed195958_green_close_front_hq_head_visible"
        ),
    )
    args = parser.parse_args(argv)
    repo = Path(__file__).resolve().parents[1]
    output_dir = (repo / args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    summaries = {}
    temporary_paths = {}
    with tempfile.TemporaryDirectory(prefix="d150_pte_gif_") as temporary:
        temporary_dir = Path(temporary)
        for policy_key, label, representation, checkpoint in POLICIES:
            for f_value in F_VALUES:
                print(f"ROLLOUT {label} F={f_value}", flush=True)
                summary, mmap_path, _can_positions, _stable_history = rollout(
                    repo,
                    temporary_dir,
                    output_dir,
                    policy_key,
                    label,
                    representation,
                    checkpoint,
                    f_value,
                )
                key = (policy_key, f_value)
                summaries[key] = summary
                temporary_paths[key] = mmap_path
        initial_hashes = {value["initial_state_sha256"] for value in summaries.values()}
        if len(initial_hashes) != 1:
            raise ValueError("F comparison did not use one initial physical state")
        comparison = write_comparison(output_dir, temporary_paths, summaries)

    payload = {
        "experiment": {
            "task": "can_color_sort",
            "seed": SEED,
            "object_variant": "green",
            "target_label": "blue",
            "bin_layout": "swapped; blue world +Y, red world -Y",
            "camera": DISPLAY_CAMERA,
            "reason_for_camera": (
                "third-person fixed view keeps the robot, table, can, and both "
                "boxes visible"
            ),
            "observer_camera": {
                "lookat": list(OBSERVER_LOOKAT),
                "distance": OBSERVER_DISTANCE,
                "azimuth_deg": OBSERVER_AZIMUTH_DEG,
                "elevation_deg": OBSERVER_ELEVATION_DEG,
                "width": OBSERVER_WIDTH,
                "height": OBSERVER_HEIGHT,
                "geom_group_4_head_visible": True,
            },
            "frames": [0, MAX_FRAME],
            "playback_fps": CONTROL_HZ,
            "f_values": list(F_VALUES),
            "lookahead_seconds": [value / CONTROL_HZ for value in F_VALUES],
            "same_initial_physical_state": True,
            "comparison_gif": str(comparison.resolve()),
        },
        "rollouts": [
            summaries[(policy_key, f_value)]
            for policy_key, _label, _representation, _checkpoint in POLICIES
            for f_value in F_VALUES
        ],
    }
    (output_dir / "summary.json").write_text(json.dumps(payload, indent=2) + "\n")
    print(output_dir, flush=True)


if __name__ == "__main__":
    main()
