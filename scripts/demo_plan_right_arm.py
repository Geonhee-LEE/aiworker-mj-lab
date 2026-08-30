"""오른팔 RRT-Connect 모션 플래닝을 실제 can-sort 장면에서 직접 실행하는 데모.

실행 (저장소 루트에서):

    PYTHONPATH=src MUJOCO_GL=osmesa python3 scripts/demo_plan_right_arm.py

옵션:
    --seed N           목표 configuration 샘플링 seed (기본 0)
    --start "q0 .. q6"  시작 configuration 직접 지정 (기본: 미리 확인한 유효 자세)
    --goal "q0 .. q6"   목표 configuration 직접 지정 (기본: seed로 무작위 유효 표본)
    --execute           계획한 경로를 MuJoCo 물리로 재생하고 추종 오차를 보고
    --viewer            실시간 뷰어 창을 띄워 눈으로 확인
    --show-tree         RRT-Connect가 탐색한 두 트리를 뷰어에 그린다(--viewer 자동 활성화)
    --loop N            목표에 도착할 때마다 새 무작위 목표를 다시 계획·재생한다.
                        N<=0이면 뷰어를 닫거나 Ctrl-C할 때까지 계속 반복(기본 1회)
    --no-obstacle       탐색 영역에 추가한 장애물(빨간 기둥)을 빼고 비교

기본적으로 테이블 위에 오른팔이 실제로 피해 가야 하는 장애물(빨간 기둥,
``planning_obstacle``)을 하나 추가한다. 저장소의 ``models/full_scene.xml``은
건드리지 않는다 — 데모를 실행할 때만 ``mujoco.MjSpec``으로 임시 지오메트리를
붙이고 컴파일한다.

``--viewer``를 쓸 때는 ``MUJOCO_GL``을 설정하지 않는다(``osmesa``/``egl``은
오프스크린 백엔드라 창형 GLFW 뷰어와 충돌해 ``OpenGL error ... mjr_makeContext``가
난다). 셸에 이미 export되어 있다면 ``env -u MUJOCO_GL``로 지우고 실행한다.
"""

import argparse
import os
import sys
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

OBSTACLE_PREFIX = "planning_obstacle_"
# (x, y, z, radius) 구체 3개. "테이블 위"가 아니라 오른팔이 무작위 관절
# 표본에서 실제로 손을 뻗는 영역(=RightArmSpace.sample이 만드는 분포) 안에
# 놓았다 — 처음엔 테이블 위 고정 기둥 하나였는데, 팔이 실제로 훑는 도달
# 영역이 테이블 근처가 아니라 훨씬 넓고 다른 자리(y가 훨씬 더 음수인 쪽)에
# 몰려 있어서 거의 안 걸렸다. 5000개 무작위 유효 표본의 손끝 FK 위치
# 분포(중앙값 근방)에서 후보 중심을 뽑았다. 반지름 6cm에서도
# ``DEFAULT_START``는 계속 유효하다 — 실측: 무작위 유효 표본 비율은 장애물
# 없을 때(58%) 대비 52%로 소폭 감소, 직선 경로 차단율은 목표 50개 중
# 27개(54%). 반지름을 더 키우면(7cm+) 시작 자세도 무효가 되기 시작한다.
OBSTACLE_SPHERES = (
    (-0.05, -0.91, 1.61, 0.06),
    (0.55, -0.81, 1.19, 0.06),
    (-0.01, -0.80, 1.17, 0.06),
)
OBSTACLE_NAMES = tuple(f"{OBSTACLE_PREFIX}{i}" for i in range(len(OBSTACLE_SPHERES)))
REQUIRE_CONTACT_GEOMS = (
    "target_bin_floor",
    "target_bin_red_floor",
    "can_geom",
    "table",
    "floor",
) + OBSTACLE_NAMES
# 상자를 승격한 상태에서 실제로 유효함을 확인한 기본 자세다. 일반 teleop
# ``home`` 키프레임은 상자 승격 후 겹치므로 기본값으로 쓰지 않는다.
DEFAULT_START = np.array([0.0, -1.4, 0.0, -0.5, 0.0, 0.3, 0.0])
# 시각화에서 각 관절 configuration을 하나의 3D 점으로 투영할 site다.
TREE_SITE_NAME = "grasp_target_r"
# #2a9e4a / #4f8ff2 / #d94fa0 — 3색 모두 OKLab Delta E 기반 CVD(색각 이상)
# 검사를 통과한 조합이다(protan/deutan 최소 10.3, 정상 시각 최소 24.9,
# 모두 8/15 문턱 이상). 이전 초록·주황 조합은 protanopia에서 Delta E 2.8로
# 사실상 구분이 안 됐다 — Q-space 시각화 페이지에서 검증하며 발견했다.
START_TREE_RGBA = np.array([0.165, 0.620, 0.290, 0.9], dtype=np.float32)
GOAL_TREE_RGBA = np.array([0.310, 0.561, 0.949, 0.9], dtype=np.float32)
PATH_RGBA = np.array([0.851, 0.310, 0.627, 0.95], dtype=np.float32)


def _build_scene(*, with_obstacle=True):
    spec = mujoco.MjSpec.from_file(str(MODEL_PATH))
    if with_obstacle:
        for name, (x, y, z, radius) in zip(OBSTACLE_NAMES, OBSTACLE_SPHERES):
            spec.worldbody.add_geom(
                name=name,
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                pos=[x, y, z],
                size=[radius, 0.0, 0.0],
                rgba=[0.9, 0.2, 0.1, 0.85],
            )
    model = spec.compile()
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


def _site_positions(checker, space, configs, site_id):
    """관절 configuration 배열 ``(N, 7)``을 각각 ``site_id`` world 위치로 투영한다.

    베이스·리프트·왼팔·손가락 등 계획 대상이 아닌 자유도는 충돌 검사기가
    스냅샷으로 들고 있는 배경 상태(``checker.snapshot_qpos``)를 그대로 쓴다.
    """
    background = checker.snapshot_qpos
    positions = np.empty((len(configs), 3))
    for index, q in enumerate(configs):
        qpos = background.copy()
        space.write(qpos, q)
        site = checker.tree.forward_site(qpos, site_id, space.joint_ids)
        positions[index] = site.position
    return positions


def _draw_trees(viewer, trees, *, node_size=0.006, edge_width=1.5):
    """``trees``(positions, parents, rgba) 목록을 뷰어의 user scene에 그린다."""
    scn = viewer.user_scn
    scn.ngeom = 0
    for positions, parents, rgba in trees:
        for position in positions:
            if scn.ngeom >= scn.maxgeom:
                break
            mujoco.mjv_initGeom(
                scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([node_size, 0.0, 0.0]), position, np.eye(3).flatten(), rgba,
            )
            scn.ngeom += 1
        for child_index, parent_index in enumerate(parents):
            if parent_index < 0 or scn.ngeom >= scn.maxgeom:
                continue
            mujoco.mjv_initGeom(
                scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_LINE,
                np.zeros(3), np.zeros(3), np.eye(3).flatten(), rgba,
            )
            mujoco.mjv_connector(
                scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_LINE, edge_width,
                positions[parent_index], positions[child_index],
            )
            scn.ngeom += 1


def _clear_scene(viewer):
    viewer.user_scn.ngeom = 0
    viewer.sync()


def _show_tree(viewer, checker, space, result, *, pause_s):
    site_id = mujoco.mj_name2id(checker.model, mujoco.mjtObj.mjOBJ_SITE, TREE_SITE_NAME)
    start_positions = _site_positions(checker, space, result.start_tree.nodes, site_id)
    goal_positions = _site_positions(checker, space, result.goal_tree.nodes, site_id)
    _draw_trees(
        viewer,
        [
            (start_positions, result.start_tree.parents, START_TREE_RGBA),
            (goal_positions, result.goal_tree.parents, GOAL_TREE_RGBA),
        ],
    )
    viewer.sync()
    deadline = time.perf_counter() + pause_s
    while viewer.is_running() and time.perf_counter() < deadline:
        time.sleep(0.02)
    _clear_scene(viewer)


def _draw_path(viewer, checker, space, path):
    """최종 선택 경로를 순서대로 잇는 waypoint 마커+선을 그린다.

    경로는 트리가 아니라 단순 사슬이므로, 각 waypoint의 "부모"를 바로
    앞 waypoint로 두면 ``_draw_trees``를 그대로 재사용할 수 있다.
    """
    site_id = mujoco.mj_name2id(checker.model, mujoco.mjtObj.mjOBJ_SITE, TREE_SITE_NAME)
    positions = _site_positions(checker, space, path, site_id)
    parents = np.arange(-1, len(positions) - 1)
    _draw_trees(viewer, [(positions, parents, PATH_RGBA)], node_size=0.009, edge_width=2.5)
    viewer.sync()


def _step_waypoint(model, data, controller, space, q_des, max_steps, converge_tol_rad, on_frame):
    """한 waypoint로 수렴할 때까지(또는 max_steps까지) 물리를 진행한다."""
    for _ in range(max_steps):
        controller.apply(data, q_des)
        mujoco.mj_step(model, data)
        if on_frame is not None and not on_frame():
            return False  # 뷰어 창이 닫혔다 — 재생 중단
        if np.max(np.abs(data.qpos[space.qpos_adrs] - q_des)) < converge_tol_rad:
            break
    return True


def _execute(model, data, space, path, *, viewer, converge_tol_rad=0.02, max_wait_s=3.0):
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

    if viewer is None:
        for q_des in path[1:]:
            _step_waypoint(model, data, controller, space, q_des, max_steps_per_waypoint,
                            converge_tol_rad, on_frame=None)
    else:
        # 물리는 dt(보통 1 kHz)마다 진행하되, 화면 갱신은 ~60 Hz로만 한다.
        # 매 물리 스텝마다 뷰어를 sync()하면 Wayland/GLFW 오버헤드 때문에
        # 15-waypoint 경로 하나가 실제로 1분 넘게 걸릴 수 있다(실측).
        render_every = max(1, int(round(1.0 / (60.0 * dt))))
        frame_counter = 0

        def on_frame():
            nonlocal frame_counter
            frame_counter += 1
            time.sleep(dt)
            if frame_counter % render_every == 0:
                viewer.sync()
            return viewer.is_running()

        for q_des in path[1:]:
            if not _step_waypoint(model, data, controller, space, q_des,
                                   max_steps_per_waypoint, converge_tol_rad, on_frame):
                break

    final_q = data.qpos[space.qpos_adrs]
    error = np.abs(final_q - path[-1])
    return float(np.max(error))


def _run_cycle(cycle, model, data, space, checker, edge_checker, start, goal, rng, args, viewer):
    """계획 한 번 + (선택) 트리 표시 + (선택) 실행. 성공한 목표 configuration을 반환한다."""
    print(f"--- cycle {cycle}: start={np.round(start, 2).tolist()} goal={np.round(goal, 2).tolist()} ---")
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
    if viewer is not None and (args.show_tree or not result.success):
        _show_tree(viewer, checker, space, result, pause_s=args.tree_pause_s)
    if not result.success:
        return None
    print(f"path waypoints = {len(result.path)}")

    if args.execute:
        # live data를 이 cycle의 시작점으로 맞춘 뒤 재생한다. (이 동기화 없이
        # 재생하면 물리 시뮬레이션이 planner가 검증한 시작 자세가 아닌 다른
        # — 상자와 겹칠 수도 있는 — 자세에서 출발한다. 실제로 겪은 버그.)
        space.write(data.qpos, start)
        mujoco.mj_forward(model, data)
        if viewer is not None:
            # 계획한 경로(주황)를 그려두고, 팔이 실제로 움직이는 동안에도
            # 지우지 않는다 — "이 경로를 따라가는 중"이라는 걸 눈으로
            # 비교할 수 있게. ``_execute``의 프레임 콜백은 user_scn을
            # 건드리지 않으므로 여기서 그린 것이 실행 내내 그대로 남는다.
            _draw_path(viewer, checker, space, result.path)
        max_error = _execute(model, data, space, result.path, viewer=viewer)
        print(f"실행 완료. 최종 관절 오차(최대) = {max_error:.4f} rad")
        if viewer is not None:
            _clear_scene(viewer)

    return result.path[-1]


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
    parser.add_argument("--show-tree", action="store_true", help="--viewer를 자동으로 켠다")
    parser.add_argument("--tree-pause-s", type=float, default=2.5)
    parser.add_argument(
        "--loop", type=int, default=1,
        help="목표 도착마다 새 무작위 목표로 반복. 0 이하면 무한 반복(뷰어를 닫거나 Ctrl-C)",
    )
    parser.add_argument("--no-obstacle", action="store_true", help="추가 장애물(빨간 기둥) 없이 실행")
    args = parser.parse_args(argv)
    use_viewer = args.viewer or args.show_tree
    with_obstacle = not args.no_obstacle

    model, data = _build_scene(with_obstacle=with_obstacle)
    space = RightArmSpace.from_model(model)
    require_contact_geoms = REQUIRE_CONTACT_GEOMS if with_obstacle else tuple(
        name for name in REQUIRE_CONTACT_GEOMS if name not in OBSTACLE_NAMES
    )
    checker = ArmCollisionChecker(
        model, space, padding_m=args.padding_m, require_contact_geoms=require_contact_geoms
    )
    checker.set_snapshot(data)
    edge_checker = EdgeChecker(space, checker.is_valid, resolution_rad=0.05)

    rng = np.random.default_rng(args.seed)
    start = args.start if args.start is not None else DEFAULT_START
    if not checker.is_valid(start):
        print(f"시작 자세가 무효합니다: {checker.report(start)}")
        return 1

    def run_all(viewer):
        current_start = start
        cycle_count = 0
        while args.loop <= 0 or cycle_count < args.loop:
            cycle_count += 1
            # --goal은 첫 cycle에만 적용한다. 반복(--loop)의 나머지 cycle은
            # 항상 다음 rng 표본으로 새 무작위 목표를 고른다.
            if cycle_count == 1 and args.goal is not None:
                goal = args.goal
            else:
                goal = _sample_valid_goal(space, checker, rng)
            if not checker.is_valid(goal):
                print(f"목표 자세가 무효합니다: {checker.report(goal)}")
                return 1
            reached = _run_cycle(
                cycle_count, model, data, space, checker, edge_checker,
                current_start, goal, rng, args, viewer,
            )
            if reached is None:
                return 1
            current_start = reached
            if viewer is not None and not viewer.is_running():
                break
        return 0

    if not use_viewer:
        return run_all(None)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        rc = run_all(viewer)
        if viewer.is_running():
            viewer.sync()

    # ``with`` 블록이 뷰어를 정상 종료했어도 GLFW 렌더 스레드가 일부 드라이버
    # 조합에서 Python 인터프리터 종료 순서와 어긋나 뒤늦게 세그폴트를 낼 수
    # 있다(실측). 필요한 출력은 이미 다 찍었으므로 정상 종료 절차(atexit, GC
    # __del__)를 건너뛰고 바로 프로세스를 끝내 그 경로를 피한다.
    sys.stdout.flush()
    os._exit(rc)


if __name__ == "__main__":
    raise SystemExit(main())
