"""오른팔 RRT-Connect 모션 플래닝을 실제 can-sort 장면에서 직접 실행하는 데모.

실행 (저장소 루트에서):

    PYTHONPATH=src MUJOCO_GL=osmesa python3 scripts/demo_plan_right_arm.py

옵션:
    --seed N          목표 configuration 샘플링 seed (기본 0)
    --start "q0 .. q6" 시작 configuration 직접 지정 (기본: 미리 확인한 유효 자세)
    --goal "q0 .. q6"  목표 configuration 직접 지정 (기본: seed로 무작위 유효 표본)
    --execute          계획한 경로를 MuJoCo 물리로 재생하고 추종 오차를 보고
    --viewer            --execute와 함께 실시간 뷰어 창을 띄워 눈으로 확인
"""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from ffw_sh5_grasp.control.arm import ArmTorqueController
from ffw_sh5_grasp.imitation.simulation.environment import enable_task_collisions
from ffw_sh5_grasp.paths import MODEL_PATH
from ffw_sh5_grasp.planning import (
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
# 상자를 승격한 상태에서 실제로 유효함을 확인한 기본 자세다. 일반 teleop
# ``home`` 키프레임은 상자 승격 후 겹치므로 기본값으로 쓰지 않는다.
DEFAULT_START = np.array([0.0, -1.4, 0.0, -0.5, 0.0, 0.3, 0.0])


def _build_scene():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    enable_task_collisions(model, ("target_bin", "target_bin_red"))
    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, home_key)
    mujoco.mj_forward(model, data)
    return model, data


def _sample_valid_goal(space, checker, rng, *, max_attempts=200):
    for _ in range(max_attempts):
        q = space.sample(rng)
        if checker.is_valid(q):
            return q
    raise RuntimeError(f"{max_attempts}회 시도했지만 유효한 목표를 찾지 못했습니다")


def _parse_q(text):
    values = [float(token) for token in text.split()]
    if len(values) != 7:
        raise argparse.ArgumentTypeError(f"관절 7개 값이 필요합니다, 받음 {len(values)}개")
    return np.asarray(values, dtype=float)


def _execute(model, data, space, path, *, use_viewer, converge_tol_rad=0.02, max_wait_s=3.0):
    """웨이포인트마다 실제로 수렴할 때까지 기다렸다가 다음으로 넘어간다.

    관절 공간 거리로 재생 시간을 미리 계산하는 대신 수렴 게이팅을 쓰는 이유는,
    ``ArmTorqueController``의 토크 제한 하에서 큰 다관절 동시 이동이 planner의
    ``max_joint_speed_rad_s`` 가정보다 훨씬 느리게 추종될 수 있기 때문이다
    (미리 정한 시간표대로 밀어붙이면 오차가 누적된다). 정식 시간 파라미터화는
    P2에서 다룬다 — 이 함수는 데모/디버그 재생용이다.
    """
    controller = ArmTorqueController(model, space.joint_names)
    dt = float(model.opt.timestep)
    max_steps_per_waypoint = max(1, int(round(max_wait_s / dt)))
    viewer_ctx = mujoco.viewer.launch_passive(model, data) if use_viewer else None

    try:
        for q_des in path[1:]:
            for _ in range(max_steps_per_waypoint):
                controller.apply(data, q_des)
                mujoco.mj_step(model, data)
                if viewer_ctx is not None:
                    viewer_ctx.sync()
                    time.sleep(dt)
                if np.max(np.abs(data.qpos[space.qpos_adrs] - q_des)) < converge_tol_rad:
                    break
    finally:
        if viewer_ctx is not None:
            viewer_ctx.close()

    final_q = data.qpos[space.qpos_adrs]
    error = np.abs(final_q - path[-1])
    return float(np.max(error))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--start", type=_parse_q, default=None)
    parser.add_argument("--goal", type=_parse_q, default=None)
    parser.add_argument("--padding-m", type=float, default=0.012)
    parser.add_argument("--step-size-rad", type=float, default=0.3)
    parser.add_argument("--goal-bias", type=float, default=0.1)
    parser.add_argument("--max-iterations", type=int, default=4000)
    parser.add_argument("--time-budget-s", type=float, default=5.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args(argv)

    model, data = _build_scene()
    space = RightArmSpace.from_model(model)
    checker = ArmCollisionChecker(
        model, space, padding_m=args.padding_m, require_contact_geoms=REQUIRE_CONTACT_GEOMS
    )
    checker.set_snapshot(data)
    edge_checker = EdgeChecker(space, checker.is_valid, resolution_rad=0.05)

    rng = np.random.default_rng(args.seed)
    start = args.start if args.start is not None else DEFAULT_START
    if not checker.is_valid(start):
        print(f"시작 자세가 무효합니다: {checker.report(start)}")
        return 1
    goal = args.goal if args.goal is not None else _sample_valid_goal(space, checker, rng)
    if not checker.is_valid(goal):
        print(f"목표 자세가 무효합니다: {checker.report(goal)}")
        return 1

    print(f"start = {np.round(start, 3).tolist()}")
    print(f"goal  = {np.round(goal, 3).tolist()}")
    print(f"straight line valid = {edge_checker.is_valid_edge(start, goal)}")

    result = plan_rrt_connect(
        space, edge_checker, start, goal,
        rng=rng, step_size_rad=args.step_size_rad, goal_bias=args.goal_bias,
        max_iterations=args.max_iterations, time_budget_s=args.time_budget_s,
    )
    print(
        f"success={result.success} reason={result.reason} "
        f"iterations={result.iterations} nodes={result.node_counts} "
        f"checks={result.state_checks} elapsed={result.elapsed_s:.3f}s"
    )
    if not result.success:
        return 1
    print(f"path waypoints = {len(result.path)}")

    if args.execute:
        # live data는 아직 ``home`` 키프레임 그대로다 — 실행을 시작하기 전에
        # 실제 시뮬레이션 상태를 계획의 시작점(``start``)으로 맞춰야 한다.
        # (이 동기화 없이 재생하면 재생이 planner가 검증한 시작 자세가 아닌
        # 다른 — 상자와 겹칠 수도 있는 — 자세에서 출발하게 된다.)
        space.write(data.qpos, start)
        mujoco.mj_forward(model, data)
        max_error = _execute(model, data, space, result.path, use_viewer=args.viewer)
        print(f"실행 완료. 최종 관절 오차(최대) = {max_error:.4f} rad")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
