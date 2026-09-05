"""P7.1 베이스 자세 선택(``planning.base_pose``)과 얇은 주행 실행
(``planning.mobile_execution``) 검증.

``world_to_base_frame``·``select_base_pose``의 순수 로직(SE(2) 변환, 후보
순위 매기기)은 MuJoCo 없이 스텁으로 빠르게 돈다. ``BaseFootprintChecker``의
실제 충돌 판정은 작은 합성 MJCF로 확인한다(실제 장면엔 베이스 높이대
[0.27, 0.51]m에 겹치는 정적 장애물이 없어 참-충돌 사례를 만들 수 없다 —
table은 z∈[0.63, 0.73]로 그 위에 있다).

가장 중요한 회귀 방지 테스트는
``test_target_unreachable_from_far_base_becomes_reachable_after_repositioning``다
— 실제 can-sort 장면에서 (1) 먼 베이스 위치에서는 진짜 IK로 도달 불가능하고
(2) ``select_base_pose``가 고른 위치로 옮기면 같은 진짜 IK로 도달
가능해짐을 ``build_reachability_map``(P7.0, 새 IK 아님)을 그대로 재사용해
증명한다.

Headless 단독 실행: ``python3 tests/test_planning_base_pose.py``
"""

import math
import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.control.base import SwerveDrive  # noqa: E402
from ffw_sh5_grasp.imitation.simulation.environment import (  # noqa: E402
    enable_task_collisions,
)
from ffw_sh5_grasp.kinematics.joint_space import JointSpaceKinematics  # noqa: E402
from ffw_sh5_grasp.paths import MODEL_PATH  # noqa: E402
from ffw_sh5_grasp.planning import ArmCollisionChecker, RightArmSpace  # noqa: E402
from ffw_sh5_grasp.planning.base_pose import (  # noqa: E402
    BaseFootprintChecker,
    select_base_pose,
    world_to_base_frame,
)
from ffw_sh5_grasp.planning.mobile_execution import drive_base_to_pose  # noqa: E402
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
# home 키프레임에서 base_x=base_y=base_yaw=0으로 확인됨(test_planning_reachability.py와 동일 실측).
# 3000개 무작위 유효 표본 FK 1~99 백분위 실측 중앙 부근의 "쉬운" 도달 지점.
REACHABLE_WORLD_TARGET = np.array([0.0, -0.6, 1.2])
# 로봇 뒤 지하 3m — 어느 베이스 위치에서도 물리적으로 못 닿는 먼 지점.
FAR_BASE_POSE = np.array([3.0, 3.0, 0.0])

_WALL_MJCF = """
<mujoco>
  <worldbody>
    <geom name="wall" type="box" size="0.1 1 1" pos="1.0 0 0" contype="1" conaffinity="1"/>
    <body name="base_link" pos="0 0 0">
      <joint name="base_x" type="slide" axis="1 0 0"/>
      <joint name="base_y" type="slide" axis="0 1 0"/>
      <joint name="base_yaw" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.2 0.2 0.12" contype="1" conaffinity="1"/>
    </body>
  </worldbody>
</mujoco>
"""


def _scene_at_base_pose(base_pose):
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    enable_task_collisions(model, ("target_bin", "target_bin_red"))
    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, home_key)
    for name, value in zip(("base_x", "base_y", "base_yaw"), base_pose):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint_id]] = float(value)
    mujoco.mj_forward(model, data)
    space = RightArmSpace.from_model(model)
    checker = ArmCollisionChecker(
        model, space, padding_m=0.012, require_contact_geoms=REQUIRE_CONTACT_GEOMS
    )
    checker.set_snapshot(data)
    solver = JointSpaceKinematics(model, TREE_SITE_NAME, list(space.joint_names), tree=checker.tree)
    return model, data, space, checker, solver


class _StubReachabilityMap:
    """``select_base_pose``는 ``.query``만 있으면 되는 duck-typed 의존이다."""

    def __init__(self, score_fn):
        self._score_fn = score_fn
        self.queried = []

    def query(self, relative_xyz):
        relative_xyz = np.asarray(relative_xyz, dtype=float).copy()
        self.queried.append(relative_xyz)
        return self._score_fn(relative_xyz)


class _AlwaysValidFootprint:
    def is_valid(self, base_pose):
        return True


# --- world_to_base_frame: 순수 SE(2) 변환 로직 --------------------------------


def test_world_to_base_frame_identity_at_origin():
    target = (1.3, -0.4, 0.9)
    relative = world_to_base_frame(target, (0.0, 0.0, 0.0))
    np.testing.assert_allclose(relative, target)


def test_world_to_base_frame_pure_translation():
    relative = world_to_base_frame((1.0, 2.0, 5.0), (1.0, 2.0, 0.0))
    np.testing.assert_allclose(relative, [0.0, 0.0, 5.0])


def test_world_to_base_frame_rotation_matches_standard_so2():
    base_pose = (0.0, 0.0, math.pi / 2)
    target = (1.0, 0.0, 0.3)
    relative = world_to_base_frame(target, base_pose)
    # 표준 SO(2) 회전행렬 R(-yaw)를 독립적으로 다시 계산해 대조한다.
    yaw = base_pose[2]
    rotation = np.array(
        [[math.cos(-yaw), -math.sin(-yaw)], [math.sin(-yaw), math.cos(-yaw)]]
    )
    expected_xy = rotation @ np.array([target[0] - base_pose[0], target[1] - base_pose[1]])
    np.testing.assert_allclose(relative[:2], expected_xy, atol=1e-12)
    assert relative[2] == target[2]


def test_world_to_base_frame_z_passes_through_unaffected():
    for base_pose in [(0.0, 0.0, 0.0), (2.0, -1.0, math.pi / 3), (-5.0, 5.0, math.pi)]:
        relative = world_to_base_frame((0.4, 0.4, 1.7), base_pose)
        assert relative[2] == 1.7


# --- select_base_pose: 후보 생성·순위·거부 로직(스텁, MuJoCo 없음) -----------------


def test_select_base_pose_picks_highest_scoring_candidate():
    stub_map = _StubReachabilityMap(lambda rel: float(rel[0]))
    result = select_base_pose(
        stub_map,
        target_world_xyz=(0.0, 0.0, 0.0),
        footprint_checker=_AlwaysValidFootprint(),
        current_base_pose=(0.0, 0.0, 0.0),
        candidate_radii=[1.0, 2.0],
        candidate_angles=[0.0, math.pi / 2, math.pi],
        min_reachability=-10.0,
    )
    assert result.success
    best_relative_x = max(q[0] for q in stub_map.queried)
    got_relative = world_to_base_frame((0.0, 0.0, 0.0), result.base_pose)
    assert abs(got_relative[0] - best_relative_x) < 1e-9
    assert abs(result.reachability_score - best_relative_x) < 1e-9


def test_select_base_pose_rejects_colliding_candidates():
    stub_map = _StubReachabilityMap(lambda rel: 1.0)

    class _RejectNearOrigin:
        def is_valid(self, base_pose):
            return not (abs(base_pose[0]) < 0.05 and abs(base_pose[1]) < 0.05)

    result = select_base_pose(
        stub_map,
        target_world_xyz=(0.0, 0.0, 0.0),
        footprint_checker=_RejectNearOrigin(),
        current_base_pose=(5.0, 5.0, 0.0),
        candidate_radii=[0.0],
        candidate_angles=[0.0, math.pi / 2],
        min_reachability=0.0,
    )
    assert not result.success
    assert result.base_pose is None
    assert result.reason


def test_select_base_pose_breaks_ties_by_distance_to_current():
    stub_map = _StubReachabilityMap(lambda rel: 1.0)
    result = select_base_pose(
        stub_map,
        target_world_xyz=(0.0, 0.0, 0.0),
        footprint_checker=_AlwaysValidFootprint(),
        current_base_pose=(1.0, 0.0, 0.0),
        candidate_radii=[1.0, 5.0],
        candidate_angles=[0.0],
        min_reachability=0.0,
    )
    assert result.success
    assert abs(result.base_pose[0] - (-1.0)) < 1e-9


def test_select_base_pose_returns_failure_reason_when_no_candidate_qualifies():
    stub_map = _StubReachabilityMap(lambda rel: 0.0)
    result = select_base_pose(
        stub_map,
        target_world_xyz=(0.0, 0.0, 0.0),
        footprint_checker=_AlwaysValidFootprint(),
        current_base_pose=(0.0, 0.0, 0.0),
        candidate_radii=[1.0],
        candidate_angles=[0.0],
        min_reachability=0.5,
    )
    assert not result.success
    assert result.base_pose is None
    assert result.reason


# --- BaseFootprintChecker: 실제 충돌 판정(합성 MJCF) ---------------------------


def test_base_footprint_checker_detects_real_collision():
    model = mujoco.MjModel.from_xml_string(_WALL_MJCF)
    checker = BaseFootprintChecker(model, padding_m=0.0)
    assert checker.is_valid((0.0, 0.0, 0.0))
    assert not checker.is_valid((0.9, 0.0, 0.0))


def test_base_footprint_checker_padding_shrinks_clearance():
    model = mujoco.MjModel.from_xml_string(_WALL_MJCF)
    tight = BaseFootprintChecker(model, padding_m=0.0)
    padded = BaseFootprintChecker(model, padding_m=0.15)
    # 패딩 없이는 무충돌이지만, 여유(0.15m)를 더하면 같은 위치가 충돌로 바뀐다.
    assert tight.is_valid((0.65, 0.0, 0.0))
    assert not padded.is_valid((0.65, 0.0, 0.0))


def test_base_footprint_checker_valid_at_home_in_real_scene():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    checker = BaseFootprintChecker(model, padding_m=0.05)
    assert checker.is_valid((0.0, 0.0, 0.0))


# --- 핵심 회귀: 먼 베이스에서는 불가능, 재배치 후 실제 IK로 가능 -------------------


def test_target_unreachable_from_far_base_becomes_reachable_after_repositioning():
    rng = np.random.default_rng(0)

    _, _, far_space, far_checker, far_solver = _scene_at_base_pose(FAR_BASE_POSE)
    far_map = build_reachability_map(
        far_solver, far_checker, far_space, rng,
        grid=np.array([REACHABLE_WORLD_TARGET]), n_restarts=10,
    )
    assert far_map.success_rate[0] == 0.0, "먼 베이스 위치에서 타겟이 원래는 도달 불가능해야 함"

    origin_model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    footprint_checker = BaseFootprintChecker(origin_model, padding_m=0.05)
    reach_map = ReachabilityMap(
        grid_points=np.array([REACHABLE_WORLD_TARGET]), success_rate=np.array([1.0])
    )
    # radius=0.6, position_angle=-pi/2 → base=(0,0); yaw 후보에 0.0을 포함해
    # home 키프레임과 정확히 같은 (0,0,0)이 후보에 들어가도록 한다.
    result = select_base_pose(
        reach_map,
        REACHABLE_WORLD_TARGET,
        footprint_checker=footprint_checker,
        current_base_pose=FAR_BASE_POSE,
        candidate_radii=[0.6],
        candidate_angles=[-math.pi / 2, 0.0, math.pi / 2, math.pi],
        min_reachability=0.5,
    )
    assert result.success, result.reason
    # 격자점이 하나뿐인 지도라 모든 yaw 후보가 동점(1.0)이 될 수 있어
    # (x, y)만 확인한다 — 실제 도달 가능 여부는 아래에서 진짜 IK로 검증한다.
    np.testing.assert_allclose(result.base_pose[:2], [0.0, 0.0], atol=1e-9)

    _, _, near_space, near_checker, near_solver = _scene_at_base_pose(result.base_pose)
    near_map = build_reachability_map(
        near_solver, near_checker, near_space, rng,
        grid=np.array([REACHABLE_WORLD_TARGET]), n_restarts=10,
    )
    assert near_map.success_rate[0] == 1.0, "재배치 후에는 실제 IK로 도달 가능해야 함"


# --- drive_base_to_pose: 얇은 실행 헬퍼 스모크 테스트(실제 장면) -------------------


def test_drive_base_to_pose_reaches_small_target_displacement():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, home_key)
    mujoco.mj_forward(model, data)

    swerve_drive = SwerveDrive()
    target_pose = np.array([0.15, 0.0, 0.0])
    report = drive_base_to_pose(
        model, data, target_pose, swerve_drive,
        tolerance_m=0.03, tolerance_rad=0.05, max_steps=8000,
    )
    assert report.success, (
        f"주행 실패: 남은 위치오차={report.final_position_error_m:.4f}m, "
        f"yaw오차={report.final_yaw_error_rad:.4f}rad, steps={report.steps}"
    )


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
