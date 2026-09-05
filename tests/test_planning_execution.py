"""P3 실행 모듈(``follow_trajectory``) 검증: 실제 can-sort 장면 재생, 침투 없음,
최종 site 오차 ≤5mm(PRD P3 exit criterion). 순수 구조 시험은 합성 경로로 빠르게.

Headless 단독 실행: ``python3 tests/test_planning_execution.py``
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.control.arm import ArmTorqueController  # noqa: E402
from ffw_sh5_grasp.imitation.simulation.environment import (  # noqa: E402
    enable_task_collisions,
)
from ffw_sh5_grasp.paths import MODEL_PATH  # noqa: E402
from ffw_sh5_grasp.planning import (  # noqa: E402
    ArmCollisionChecker,
    EdgeChecker,
    RightArmSpace,
    follow_trajectory,
    plan_rrt_connect,
    time_parameterize,
)

REQUIRE_CONTACT_GEOMS = (
    "target_bin_floor",
    "target_bin_red_floor",
    "can_geom",
    "table",
    "floor",
)
TREE_SITE_NAME = "grasp_target_r"
# 일반 teleop ``home`` 자세는 상자 승격 후 실제로 겹친다(P0에서 확인한 사실).
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
    return model, data, space, checker


def test_follow_trajectory_reaches_goal_within_5mm():
    model, data, space, checker = _scene()
    edge_checker = EdgeChecker(space, checker.is_valid, resolution_rad=0.05)
    result = plan_rrt_connect(
        space, edge_checker, START_Q, GOAL_Q,
        rng=np.random.default_rng(0), step_size_rad=0.3, goal_bias=0.1,
        max_iterations=4000, time_budget_s=5.0,
    )
    assert result.success, f"계획 실패: {result.reason}"

    trajectory = time_parameterize(
        space, result.path,
        max_speed_rad_s=1.0, max_accel_rad_s2=2.0, control_period_s=float(model.opt.timestep),
    )

    space.write(data.qpos, trajectory.positions[0])
    mujoco.mj_forward(model, data)
    controller = ArmTorqueController(model, space.joint_names)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TREE_SITE_NAME)

    report = follow_trajectory(model, data, space, checker, controller, trajectory, site_id)

    assert report.invalid_sample_count == 0, (
        f"재생 중 침투 발생: {report.invalid_sample_count}개 표본, "
        f"인덱스 {report.invalid_sample_indices[:5]}..."
    )
    assert report.final_site_error_m <= 0.005, (
        f"최종 site 오차 {report.final_site_error_m * 1000:.2f}mm > 5mm"
    )
    assert report.total_samples == len(trajectory.positions)


def test_follow_trajectory_reaches_goal_via_torque_control_only():
    """제어기 토크만으로 실제 목표 근처에 도달하는지 — data.qpos를 직접 쓰지
    않는 진짜 폐루프 재생임을 결과로 간접 확인한다."""
    model, data, space, checker = _scene()
    edge_checker = EdgeChecker(space, checker.is_valid, resolution_rad=0.05)
    result = plan_rrt_connect(
        space, edge_checker, START_Q, GOAL_Q,
        rng=np.random.default_rng(1), step_size_rad=0.3, goal_bias=0.1,
        max_iterations=4000, time_budget_s=5.0,
    )
    assert result.success

    trajectory = time_parameterize(
        space, result.path,
        max_speed_rad_s=1.0, max_accel_rad_s2=2.0, control_period_s=float(model.opt.timestep),
    )
    space.write(data.qpos, trajectory.positions[0])
    mujoco.mj_forward(model, data)
    controller = ArmTorqueController(model, space.joint_names)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TREE_SITE_NAME)

    report = follow_trajectory(model, data, space, checker, controller, trajectory, site_id)

    # 토크 제어로만 도달했다면 최종 관절 오차가 작아야 한다(가짜로 qpos를
    # 직접 썼다면 오차가 정확히 0이 될 텐데, 실제 물리 재생은 완전한 0은
    # 아니면서도 충분히 작아야 한다 — PD+feedforward 정착 특성).
    assert 0.0 < report.max_joint_error_rad < 0.05


def test_execution_report_shape():
    """합성 최소 경로로 ExecutionReport 필드 타입·길이만 빠르게 확인."""
    model, data, space, checker = _scene()
    nearby_goal = START_Q + np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert checker.is_valid(nearby_goal)
    trajectory = time_parameterize(
        space, np.stack([START_Q, nearby_goal]),
        max_speed_rad_s=1.0, max_accel_rad_s2=2.0, control_period_s=float(model.opt.timestep),
    )
    space.write(data.qpos, trajectory.positions[0])
    mujoco.mj_forward(model, data)
    controller = ArmTorqueController(model, space.joint_names)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TREE_SITE_NAME)

    report = follow_trajectory(
        model, data, space, checker, controller, trajectory, site_id, settle_time_s=0.2
    )

    assert isinstance(report.final_site_error_m, float)
    assert isinstance(report.max_joint_error_rad, float)
    assert isinstance(report.invalid_sample_count, int)
    assert isinstance(report.invalid_sample_indices, tuple)
    assert report.total_samples == len(trajectory.positions)


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
