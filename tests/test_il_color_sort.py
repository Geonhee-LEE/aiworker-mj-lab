import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.simulation.environment import (  # noqa: E402
    AIWorkerMujocoEnv,
)

EXPECTED_TARGETS = {
    "green": "blue",
    "red": "red",
    "orange": "red",
    "blue": "blue",
}


def _geom_ids(model, body_name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return np.flatnonzero(model.geom_bodyid == body_id)


def _place_can_at_site(env, site_name):
    site_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    env.data.qpos[env.task.can_qpos : env.task.can_qpos + 3] = env.data.site_xpos[
        site_id
    ]
    env.data.qvel[env.task.can_dof : env.task.can_dof + 6] = 0.0
    mujoco.mj_forward(env.model, env.data)


def test_legacy_scenario_keeps_red_bin_disabled():
    with AIWorkerMujocoEnv(render_images=False, seed=3, task_name="can_to_box") as env:
        red_geoms = _geom_ids(env.model, "target_bin_red")
        assert len(red_geoms) == 5
        assert np.all(env.model.geom_rgba[red_geoms, 3] == 0.0)
        assert np.all(env.model.geom_contype[red_geoms] == 0)
        assert np.all(env.model.geom_conaffinity[red_geoms] == 0)
        assert env.task.object_variant == "green"
        assert env.task.target_label == "blue"
        assert len(env.target_bin_geom_ids) == 5


def test_color_sort_randomizes_visual_only_and_uses_matching_target():
    with AIWorkerMujocoEnv(
        render_images=False, seed=0, task_name="can_color_sort"
    ) as env:
        can_geom = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "can_geom")
        physical_signature = (
            env.model.geom_size[can_geom].copy(),
            env.model.geom_friction[can_geom].copy(),
            float(env.model.body_mass[env.task.can_body]),
            env.model.body_inertia[env.task.can_body].copy(),
        )
        variants = {}
        spawn_site = env.task.spawn_site
        assert np.allclose(env.data.site_xpos[spawn_site], [0.4055, -0.05, 0.7866])
        for seed in range(32):
            observation = env.reset(seed=seed)
            variant = observation["task"]["object_variant"]
            variants.setdefault(variant, seed)
            assert observation["task"]["target_label"] == (EXPECTED_TARGETS[variant])
            assert (
                np.linalg.norm(
                    env.initial_can_position[:2] - env.data.site_xpos[spawn_site, :2]
                )
                <= env.task.spawn_jitter_radius + 1e-12
            )
        assert set(variants) == set(EXPECTED_TARGETS)

        assert np.array_equal(env.model.geom_size[can_geom], physical_signature[0])
        assert np.array_equal(env.model.geom_friction[can_geom], physical_signature[1])
        assert env.model.body_mass[env.task.can_body] == physical_signature[2]
        assert np.array_equal(
            env.model.body_inertia[env.task.can_body], physical_signature[3]
        )
        assert len(env.target_bin_geom_ids) == 10
        assert np.all(env.model.geom_contype[env.target_bin_geom_ids] == 1)
        assert np.all(env.model.geom_conaffinity[env.target_bin_geom_ids] == 1)

        table_geom = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "table")
        assert np.allclose(env.model.geom_size[table_geom, :2], [0.30, 0.35])
        assert np.isclose(env.model.geom_pos[table_geom, 1], -0.05)
        expected_positions = {
            "target_bin_red": [0.5355, 0.17, 0.7391],
            "target_bin": [0.5355, -0.27, 0.7391],
        }
        for body_name, expected_position in expected_positions.items():
            body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            assert np.allclose(env.model.body_pos[body_id], expected_position)
            bin_geoms = _geom_ids(env.model, body_name)
            wall_geoms = [
                geom_id
                for geom_id in bin_geoms
                if not (
                    mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id))
                    or ""
                ).endswith("floor")
            ]
            assert np.allclose(env.model.geom_size[wall_geoms, 2], 0.08 * 2.0 / 3.0)
            outer_edge = abs(
                env.model.body_pos[body_id, 1] - env.model.geom_pos[table_geom, 1]
            ) + max(
                abs(env.model.geom_pos[geom_id, 1]) + env.model.geom_size[geom_id, 1]
                for geom_id in bin_geoms
            )
            assert outer_edge < env.model.geom_size[table_geom, 1]
        blue_body = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "target_bin")
        red_body = mujoco.mj_name2id(
            env.model, mujoco.mjtObj.mjOBJ_BODY, "target_bin_red"
        )
        assert np.isclose(
            env.model.body_pos[red_body, 1] - env.model.body_pos[blue_body, 1], 0.44
        )
        assert np.isclose(
            0.5 * (env.model.body_pos[blue_body, 1] + env.model.body_pos[red_body, 1]),
            -0.05,
        )

        for variant, seed in variants.items():
            env.reset(seed=seed)
            correct_site = (
                "target_bin_center"
                if EXPECTED_TARGETS[variant] == "blue"
                else "target_bin_red_center"
            )
            wrong_site = (
                "target_bin_red_center"
                if EXPECTED_TARGETS[variant] == "blue"
                else "target_bin_center"
            )
            _place_can_at_site(env, wrong_site)
            assert not env.task.metrics(env.data).success
            _place_can_at_site(env, correct_site)
            assert env.task.metrics(env.data).success

            # The lowered floor and walls retain real collision contacts.
            env.data.qpos[env.task.can_qpos + 2] -= 0.002
            mujoco.mj_forward(env.model, env.data)
            can_geom = mujoco.mj_name2id(
                env.model, mujoco.mjtObj.mjOBJ_GEOM, "can_geom"
            )
            floor_geom = mujoco.mj_name2id(
                env.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                correct_site.replace("_center", "_floor"),
            )
            contact_pairs = {
                frozenset((int(contact.geom1), int(contact.geom2)))
                for contact in env.data.contact
            }
            assert frozenset((can_geom, floor_geom)) in contact_pairs

            metadata = env.task.episode_metadata()
            assert metadata["scenario_name"] == "can_color_sort"
            assert metadata["object_variant"] == variant
            assert metadata["target_label"] == EXPECTED_TARGETS[variant]
            assert metadata["object_target_mapping"] == EXPECTED_TARGETS
            assert np.allclose(
                metadata["bin_body_positions"]["target_bin"], [0.5355, -0.27, 0.7391]
            )
            assert np.allclose(
                metadata["bin_body_positions"]["target_bin_red"], [0.5355, 0.17, 0.7391]
            )


def test_color_sort_recorder_can_target_underrepresented_variants():
    with AIWorkerMujocoEnv(
        render_images=False,
        seed=0,
        task_name="can_color_sort",
        object_variants=("orange", "blue"),
        randomize_bin_colors=True,
    ) as env:
        variants = set()
        layouts = set()
        target_bodies = set()
        for seed in range(64):
            observation = env.reset(seed=seed)
            variants.add(observation["task"]["object_variant"])
            layouts.add(tuple(sorted(env.task.bin_color_layout.items())))
            target_bodies.add(
                mujoco.mj_id2name(
                    env.model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    int(env.model.site_bodyid[env.task.target_site]),
                )
            )

        assert variants == {"orange", "blue"}
        assert len(layouts) == 2
        assert target_bodies == {"target_bin", "target_bin_red"}


def test_color_sort_bin_walls_stop_a_fast_can():
    """A 2 m/s can must contact, rather than tunnel through, either outer wall."""
    cases = (
        ("target_bin_center", "target_bin", -1.0),
        ("target_bin_red_center", "target_bin_red", 1.0),
    )
    with AIWorkerMujocoEnv(
        render_images=False, seed=0, task_name="can_color_sort"
    ) as env:
        can_geom = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "can_geom")
        for site_name, bin_body_name, direction in cases:
            env.reset(seed=0)
            site_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
            wall_geoms = {
                int(geom_id)
                for geom_id in _geom_ids(env.model, bin_body_name)
                if not (
                    mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id))
                    or ""
                ).endswith("floor")
            }
            target = env.data.site_xpos[site_id].copy()
            env.data.qpos[env.task.can_qpos : env.task.can_qpos + 3] = target
            env.data.qpos[env.task.can_qpos + 3 : env.task.can_qpos + 7] = [
                1.0,
                0.0,
                0.0,
                0.0,
            ]
            env.data.qvel[env.task.can_dof : env.task.can_dof + 6] = 0.0
            env.data.qvel[env.task.can_dof + 1] = 2.0 * direction
            mujoco.mj_forward(env.model, env.data)
            maximum_lateral_travel = 0.0
            touched_wall = False
            for _ in range(250):
                mujoco.mj_step(env.model, env.data)
                maximum_lateral_travel = max(
                    maximum_lateral_travel,
                    abs(env.data.qpos[env.task.can_qpos + 1] - target[1]),
                )
                touched_wall = touched_wall or any(
                    can_geom in (int(contact.geom1), int(contact.geom2))
                    and bool(
                        wall_geoms.intersection(
                            (int(contact.geom1), int(contact.geom2))
                        )
                    )
                    for contact in env.data.contact
                )
            assert touched_wall
            # The wall's inner face is 7.5 cm from the box center. A can that
            # crossed it would travel much farther than its 3.3 cm radius.
            assert maximum_lateral_travel < 0.045


def test_color_sort_starts_right_arm_ten_centimeters_higher_without_bin_contact():
    with (
        AIWorkerMujocoEnv(
            render_images=False, seed=0, task_name="can_to_box"
        ) as legacy_env,
        AIWorkerMujocoEnv(
            render_images=False, seed=0, task_name="can_color_sort"
        ) as color_sort_env,
    ):
        legacy_pose = legacy_env.get_observation()["ee_pose"]["right"]
        color_sort_pose = color_sort_env.get_observation()["ee_pose"]["right"]

        assert np.allclose(
            color_sort_pose[:3] - legacy_pose[:3], [0.0, 0.0, 0.1], atol=1e-7
        )
        assert np.isclose(
            abs(np.dot(color_sort_pose[3:], legacy_pose[3:])), 1.0, atol=1e-9
        )

        right_qpos = color_sort_env.data.qpos[
            color_sort_env.state_adapter.arm_qpos["r"]
        ]
        right_ranges = color_sort_env.action_adapter.arm_ranges["r"]
        assert np.all(right_qpos >= right_ranges[:, 0])
        assert np.all(right_qpos <= right_ranges[:, 1])

        bin_geoms = set(color_sort_env.target_bin_geom_ids.tolist())
        assert not any(
            bin_geoms.intersection((int(contact.geom1), int(contact.geom2)))
            for contact in color_sort_env.data.contact
        )


if __name__ == "__main__":
    test_legacy_scenario_keeps_red_bin_disabled()
    test_color_sort_randomizes_visual_only_and_uses_matching_target()
    test_color_sort_recorder_can_target_underrepresented_variants()
    test_color_sort_bin_walls_stop_a_fast_can()
    test_color_sort_starts_right_arm_ten_centimeters_higher_without_bin_contact()
    print("PASS")
