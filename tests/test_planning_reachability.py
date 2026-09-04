"""P7.0 reachability map(``planning.reachability``) 검증.

``ReachabilityMap.query``의 순수 로직(격자 보간)은 MuJoCo가 필요 없어 빠르게
돈다. ``build_reachability_map``은 실제 can-sort 장면에서 작은 격자로 한 번
실행해 도달 영역과 도달 밖 영역의 점수가 실제로 갈리는지 확인한다.

Headless 단독 실행: ``python3 tests/test_planning_reachability.py``
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
from ffw_sh5_grasp.kinematics.joint_space import JointSpaceKinematics  # noqa: E402
from ffw_sh5_grasp.paths import MODEL_PATH  # noqa: E402
from ffw_sh5_grasp.planning import ArmCollisionChecker, RightArmSpace  # noqa: E402
from ffw_sh5_grasp.planning.reachability import (  # noqa: E402
    ReachabilityMap,
    build_reachability_map,
)

REQUIRE_CONTACT_GEOMS = (
    "target_bin_floor",
    "target_bin_red_floor",
    "can_geom",
    "table",
    "floor",
)
TREE_SITE_NAME = "grasp_target_r"
# home 키프레임에서 base_x=base_y=base_yaw=0으로 확인됨(실측) — 이 장면의
# 월드 좌표가 곧 베이스 프레임 좌표다.
# 3000개 무작위 유효 표본 FK 1~99 백분위 실측 중앙 부근의 "쉬운" 도달 지점.
REACHABLE_OFFSET = np.array([0.0, -0.6, 1.2])
# 팔이 물리적으로 절대 못 닿는 먼 지점(로봇 뒤 지하).
UNREACHABLE_OFFSET = np.array([5.0, 5.0, -3.0])


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
    solver = JointSpaceKinematics(model, TREE_SITE_NAME, list(space.joint_names), tree=checker.tree)
    return space, checker, solver


def test_query_returns_exact_value_at_grid_point():
    grid = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    rmap = ReachabilityMap(grid_points=grid, success_rate=np.array([1.0, 0.0, 0.5]))
    assert rmap.query([0.0, 0.0, 0.0]) == 1.0
    assert rmap.query([1.0, 0.0, 0.0]) == 0.0


def test_query_interpolates_between_nearby_points():
    # 두 격자점 사이 중점에서는 두 값의 평균에 가까워야 한다(역거리 가중).
    grid = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    rmap = ReachabilityMap(grid_points=grid, success_rate=np.array([1.0, 0.0]))
    midpoint_score = rmap.query([1.0, 0.0, 0.0])
    assert abs(midpoint_score - 0.5) < 1e-9


def test_query_favors_closer_points():
    grid = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]])
    rmap = ReachabilityMap(grid_points=grid, success_rate=np.array([1.0, 0.0]))
    near_first = rmap.query([0.5, 0.0, 0.0])
    near_second = rmap.query([9.5, 0.0, 0.0])
    assert near_first > 0.5 > near_second


def test_build_reachability_map_distinguishes_reachable_from_unreachable():
    space, checker, solver = _scene()
    rng = np.random.default_rng(0)
    grid = np.array([REACHABLE_OFFSET, UNREACHABLE_OFFSET])

    rmap = build_reachability_map(solver, checker, space, rng, grid=grid, n_restarts=10)

    assert rmap.grid_points.shape == (2, 3)
    assert rmap.success_rate.shape == (2,)
    assert rmap.success_rate[0] == 1.0, "실측상 쉬운 도달 지점이 실패로 나옴"
    assert rmap.success_rate[1] == 0.0, "물리적으로 불가능한 지점이 성공으로 나옴"


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
