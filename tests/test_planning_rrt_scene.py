"""실제 can-sort MuJoCo 장면에서 RRT-Connect 한 번의 seeded 질의를 검증한다.

Headless 단독 실행: ``python3 tests/test_planning_rrt_scene.py``
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.simulation.environment import (  # noqa: E402
    enable_task_collisions,
)
from ffw_sh5_grasp.paths import MODEL_PATH  # noqa: E402
from ffw_sh5_grasp.planning import (  # noqa: E402
    ArmCollisionChecker,
    EdgeChecker,
    RightArmSpace,
    plan_rrt_connect,
)

REQUIRE_CONTACT_GEOMS = (
    "target_bin_floor",
    "target_bin_red_floor",
    "can_geom",
    "table",
    "floor",
)
# 일반 teleop ``home`` 자세는 상자 승격 후 실제로 겹친다(P0에서 확인한 사실).
# 데모/시험에서는 상자가 승격된 상태에서 유효성을 직접 확인한 자세만 쓴다.
START_Q = np.array([0.0, -1.4, 0.0, -0.5, 0.0, 0.3, 0.0])
GOAL_Q = np.array([-0.3, -0.9, 0.0, -1.8, 0.0, 0.5, 0.0])


def _scene():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    enable_task_collisions(model, ("target_bin", "target_bin_red"))
    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, home_key)
    mujoco.mj_forward(model, data)
    space = RightArmSpace.from_model(model)
    checker = ArmCollisionChecker(
        model, space, padding_m=0.012, require_contact_geoms=REQUIRE_CONTACT_GEOMS
    )
    checker.set_snapshot(data)
    return space, checker


def test_start_and_goal_are_valid():
    space, checker = _scene()
    assert checker.is_valid(START_Q)
    assert checker.is_valid(GOAL_Q)


def test_seeded_query_succeeds_and_certifies():
    space, checker = _scene()
    edge_checker = EdgeChecker(space, checker.is_valid, resolution_rad=0.05)
    result = plan_rrt_connect(
        space, edge_checker, START_Q, GOAL_Q,
        rng=np.random.default_rng(0), step_size_rad=0.3, goal_bias=0.1,
        max_iterations=4000, time_budget_s=5.0,
    )
    assert result.success, result.reason
    assert result.elapsed_s < 5.0
    for point in result.path:
        assert checker.is_valid(point)
    for a, b in zip(result.path[:-1], result.path[1:]):
        assert edge_checker.is_valid_edge(a, b)
    print(
        f"iterations={result.iterations} nodes={result.node_counts} "
        f"checks={result.state_checks} elapsed={result.elapsed_s:.3f}s "
        f"waypoints={len(result.path)}"
    )


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()


def test_plan_then_execute_converges():
    """계획한 경로를 ``ArmTorqueController``로 재생하면 실제로 목표에 수렴한다.

    회귀 대상: 재생을 시작하기 전에 live ``data.qpos``를 계획의 시작
    configuration으로 맞추지 않으면(예: 여전히 ``home`` 키프레임에 남아 있으면)
    물리 시뮬레이션이 planner가 검증한 상태와 다른 — 상자와 겹칠 수도 있는 —
    자세에서 출발해 수렴하지 못한다.
    """
    import mujoco

    from ffw_sh5_grasp.control.arm import ArmTorqueController

    mj_model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    enable_task_collisions(mj_model, ("target_bin", "target_bin_red"))
    live_data = mujoco.MjData(mj_model)
    home_key = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(mj_model, live_data, home_key)
    mujoco.mj_forward(mj_model, live_data)

    space = RightArmSpace.from_model(mj_model)
    checker = ArmCollisionChecker(
        mj_model, space, padding_m=0.012, require_contact_geoms=REQUIRE_CONTACT_GEOMS
    )
    checker.set_snapshot(live_data)
    edge_checker = EdgeChecker(space, checker.is_valid, resolution_rad=0.05)

    result = plan_rrt_connect(
        space, edge_checker, START_Q, GOAL_Q,
        rng=np.random.default_rng(0), step_size_rad=0.3, goal_bias=0.1,
        max_iterations=4000, time_budget_s=5.0,
    )
    assert result.success

    # live 시뮬레이션 상태를 계획의 시작점으로 맞춘 뒤 재생한다.
    space.write(live_data.qpos, START_Q)
    mujoco.mj_forward(mj_model, live_data)

    controller = ArmTorqueController(mj_model, space.joint_names)
    for q_des in result.path[1:]:
        for _ in range(3000):
            controller.apply(live_data, q_des)
            mujoco.mj_step(mj_model, live_data)
            if np.max(np.abs(live_data.qpos[space.qpos_adrs] - q_des)) < 0.02:
                break

    final_error = np.max(np.abs(live_data.qpos[space.qpos_adrs] - GOAL_Q))
    assert final_error < 0.05, f"실행이 목표에 수렴하지 못했습니다: {final_error} rad"
