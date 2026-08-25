import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.application.teleop import (  # noqa: E402
    ENV_TASK_NAMES,
    TeleopApp,
    _parse_args,
)
from ffw_sh5_grasp.config import SETTINGS  # noqa: E402
from ffw_sh5_grasp.imitation.simulation.environment import (
    AIWorkerMujocoEnv,  # noqa: E402
)


def _geom_ids(model, body_name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return np.flatnonzero(model.geom_bodyid == body_id)


def test_arm_only_env_reset_and_step():
    with AIWorkerMujocoEnv(render_images=False, seed=11) as env:
        first = env.reset(seed=11)
        first_can = env.initial_can_position.copy()
        target_xy = env.data.site_xpos[env.task.target_site, :2].copy()
        spawn_anchor = target_xy + env.task.spawn_anchor_offset
        assert np.linalg.norm(first_can[:2] - spawn_anchor) <= (
            env.task.spawn_jitter_radius + 1e-12
        )
        robot_home = first["debug"]["full_qpos"][: env.task.can_qpos].copy()
        assert first["qpos"].shape == (16,)
        assert set(first["ee_pose"]) == {"left", "right"}
        assert all(pose.shape == (7,) for pose in first["ee_pose"].values())
        assert first["images"] == {}
        assert np.allclose(
            first["debug"]["full_qpos"][env.head_qpos], env.head_fixed_position
        )
        assert np.allclose(first["qpos"][:7], env.left_arm_park_position)
        assert first["qpos"][7] == env.left_grasp_fixed
        assert np.all(env.model.geom_contype[env.target_bin_geom_ids] == 1)
        assert np.all(env.model.geom_conaffinity[env.target_bin_geom_ids] == 1)
        target_bin_body = mujoco.mj_name2id(
            env.model, mujoco.mjtObj.mjOBJ_BODY, "target_bin"
        )
        assert env.model.body_contype[target_bin_body] == 1
        assert env.model.body_conaffinity[target_bin_body] == 1
        assert np.all(env.model.geom_priority[env.target_bin_geom_ids] == 2)
        assert np.allclose(env.model.geom_margin[env.target_bin_geom_ids], 0.002)
        assert np.allclose(env.model.geom_solref[env.target_bin_geom_ids], [0.003, 1.0])
        for index in range(13, 21):
            body_id = mujoco.mj_name2id(
                env.model, mujoco.mjtObj.mjOBJ_BODY, f"finger_r_link{index}"
            )
            assert body_id not in env.model.exclude_signature
        left_site = mujoco.mj_name2id(
            env.model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l"
        )
        palm_normal = env.data.site_xmat[left_site].reshape(3, 3)[:, 0]
        assert np.dot(palm_normal, [0.0, 0.0, 1.0]) > 0.999
        assert not any(
            {int(contact.geom1), int(contact.geom2)}.intersection(
                env.target_bin_geom_ids
            )
            for contact in env.data.contact
        )
        requested = first["qpos"].copy()
        requested[:8] = 0.5
        prepared = env.prepare_action(requested)
        assert np.allclose(prepared[:7], env.left_arm_park_position)
        assert prepared[7] == env.left_grasp_fixed
        env.step(first["qpos"])
        reset = env.reset(seed=11)
        assert np.allclose(env.initial_can_position, first_can)
        assert np.allclose(reset["debug"]["full_qpos"][: env.task.can_qpos], robot_home)
        env.reset(seed=12)
        assert not np.allclose(env.initial_can_position, first_can)
        assert env.steps_per_control * env.model.opt.timestep == 1 / 25
        assert "whole_body_solver" not in vars(env)
        target = env.data.site_xpos[env.task.target_site].copy()
        env.data.qpos[env.task.can_qpos : env.task.can_qpos + 3] = target
        env.data.qvel[env.task.can_dof : env.task.can_dof + 6] = 0.0
        mujoco.mj_forward(env.model, env.data)
        assert env.task.metrics(env.data).success

        # A slight floor penetration must create a real can/bin contact, not
        # merely satisfy the task's geometric success test.
        env.data.qpos[env.task.can_qpos + 2] -= 0.002
        mujoco.mj_forward(env.model, env.data)
        can_geom = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "can_geom")
        floor_geom = mujoco.mj_name2id(
            env.model, mujoco.mjtObj.mjOBJ_GEOM, "target_bin_floor"
        )
        contact_pairs = {
            frozenset((int(contact.geom1), int(contact.geom2)))
            for contact in env.data.contact
        }
        assert frozenset((can_geom, floor_geom)) in contact_pairs

        # This reproducible right-arm pose places the thumb at the front wall.
        # It guards the body-level collision mask cache as well as geom masks:
        # with only geom_* patched, mj_geomDistance is negative here but MuJoCo
        # emits no robot/bin contact and the hand passes through the wall.
        env.data.qpos[env.state_adapter.arm_qpos["r"]] = [
            0.03791421,
            -0.02158768,
            0.45203724,
            -1.58742088,
            0.05192529,
            0.03414608,
            -0.57582228,
        ]
        mujoco.mj_forward(env.model, env.data)
        assert any(
            pair.intersection(env.target_bin_geom_ids)
            and any(
                (
                    mujoco.mj_id2name(
                        env.model,
                        mujoco.mjtObj.mjOBJ_BODY,
                        int(env.model.geom_bodyid[geom_id]),
                    )
                    or ""
                ).startswith("finger_r_")
                for geom_id in pair
            )
            for contact in env.data.contact
            for pair in ({int(contact.geom1), int(contact.geom2)},)
        )


def test_arm_only_env_can_attach_to_existing_model_and_data():
    model = mujoco.MjModel.from_xml_path(str(REPO_ROOT / "models/full_scene.xml"))
    data = mujoco.MjData(model)
    with AIWorkerMujocoEnv(model=model, data=data, render_images=False, seed=7) as env:
        assert env.model is model
        assert env.data is data
        assert env.get_qpos().shape == (16,)

    # Reattaching must be safe after collision exclusions were configured by
    # the first policy session.
    with AIWorkerMujocoEnv(model=model, data=data, render_images=False, seed=8) as env:
        observation = env.step(env.get_qpos())
        assert observation["qpos"].shape == (16,)


def test_default_teleop_matches_policy_pose_and_can_reset_is_object_only():
    app = TeleopApp.__new__(TeleopApp)
    app.act_policy_seed = 1000
    app._setup_sim()

    expected_left = np.asarray(SETTINGS.get("imitation.left_arm_park_position_rad"))
    assert np.allclose(app.data.qpos[app.arm_qpos_indices["l"]], expected_left)
    for name, expected in zip(
        ("head_joint1", "head_joint2"),
        SETTINGS.get("imitation.head_fixed_position_rad"),
    ):
        assert np.isclose(app.data.joint(name).qpos[0], expected)
    target_bin = app.model.body("target_bin").id
    target_geoms = np.flatnonzero(app.model.geom_bodyid == target_bin)
    assert np.all(app.model.geom_contype[target_geoms] == 1)
    assert np.all(app.model.geom_conaffinity[target_geoms] == 1)

    robot_before = app.data.qpos[: app.imitation_task.can_qpos].copy()
    can_before = app.data.qpos[
        app.imitation_task.can_qpos : app.imitation_task.can_qpos + 3
    ].copy()
    app.reset_can()
    assert np.array_equal(app.data.qpos[: app.imitation_task.can_qpos], robot_before)
    assert not np.array_equal(
        app.data.qpos[app.imitation_task.can_qpos : app.imitation_task.can_qpos + 3],
        can_before,
    )


def test_teleop_env_argument_randomizes_color_and_target_side_independently():
    args = _parse_args(
        [
            "--env",
            "1",
            "--policy-representation",
            "task",
            "--policy-rerun",
            "--policy-rerun-port",
            "9888",
            "--policy-rerun-hz",
            "10",
            "--policy-ik-speed-scale",
            "1.5",
            "--policy-pte-steps",
            "5",
        ]
    )
    assert ENV_TASK_NAMES[args.env] == "can_color_sort"
    assert args.policy_representation == "task"
    assert args.policy_rerun
    assert args.policy_rerun_port == 9888
    assert args.policy_rerun_hz == 10.0
    assert args.policy_ik_speed_scale == 1.5
    assert args.policy_pte_steps == 5

    app = TeleopApp.__new__(TeleopApp)
    app.act_policy_seed = 1000
    app.task_name = ENV_TASK_NAMES[args.env]
    app._setup_sim()

    task = app.imitation_task
    can_visual = mujoco.mj_name2id(
        app.model, mujoco.mjtObj.mjOBJ_GEOM, "can_side_visual"
    )
    variants = set()
    layouts = set()
    target_bodies = set()
    colors = {}
    for _ in range(128):
        app.reset_can()
        variants.add(task.object_variant)
        layouts.add(tuple(sorted(task.bin_color_layout.items())))
        material_id = int(app.model.geom_matid[can_visual])
        colors[task.object_variant] = tuple(app.model.mat_rgba[material_id])
        target_body = mujoco.mj_id2name(
            app.model,
            mujoco.mjtObj.mjOBJ_BODY,
            int(app.model.site_bodyid[task.target_site]),
        )
        target_bodies.add(target_body)
        assert target_body == task.bin_color_layout[task.target_label]

    assert variants == {"green", "red", "orange", "blue"}
    assert len(layouts) == 2
    assert target_bodies == {"target_bin", "target_bin_red"}
    assert len(set(colors.values())) == 4


def test_teleop_task_policy_pose_is_converted_by_arm_only_ik():
    app = TeleopApp.__new__(TeleopApp)
    app.act_policy_seed = 1000
    app.task_name = "can_color_sort"
    app._setup_sim()
    app.ik_err_mm = {"l": 0.0, "r": 0.0}

    with AIWorkerMujocoEnv(
        model=app.model,
        data=app.data,
        render_images=False,
        reset_on_init=False,
        task=app.imitation_task,
    ) as env:
        app.act_policy_env = env
        observation = env.get_observation()
        task_action = np.concatenate(
            (
                observation["ee_pose"]["right"],
                [0.75],
            )
        )
        task_action[0] += 0.01
        before = env.get_qpos()

        original_position_gain = app.whole_body_solver.position_gain
        original_orientation_gain = app.whole_body_solver.orientation_gain
        app.act_policy_ik_speed_scale = 1.0
        slow_action = app._task_policy_action_to_joint(task_action)
        app.act_policy_ik_speed_scale = 2.0
        joint_action = app._task_policy_action_to_joint(task_action)

        assert joint_action.shape == (16,)
        assert np.all(np.isfinite(joint_action))
        assert np.array_equal(joint_action[:8], before[:8])
        assert not np.array_equal(joint_action[8:15], before[8:15])
        slow_delta = np.linalg.norm(slow_action[8:15] - before[8:15])
        fast_delta = np.linalg.norm(joint_action[8:15] - before[8:15])
        assert fast_delta > 1.5 * slow_delta
        assert np.isclose(joint_action[15], 0.75)
        assert app.ik_err_mm["r"] > 0.0
        assert app.whole_body_solver.position_gain == original_position_gain
        assert app.whole_body_solver.orientation_gain == original_orientation_gain


if __name__ == "__main__":
    test_arm_only_env_reset_and_step()
    test_arm_only_env_can_attach_to_existing_model_and_data()
    test_default_teleop_matches_policy_pose_and_can_reset_is_object_only()
    test_teleop_env_argument_randomizes_color_and_target_side_independently()
    test_teleop_task_policy_pose_is_converted_by_arm_only_ik()
    print("PASS")
