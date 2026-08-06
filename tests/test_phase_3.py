"""Phase 3 ``arm_hand`` 장면과 6자유도 IK 검증.

Part 0은 공개된 월드 정렬 기하 Jacobian을 중앙 유한차분과 비교하고, 정규화된
quaternion의 이중 덮개 처리도 확인한다.

Part 1은 정기구학으로 도달 가능한 자세 100개를 무작위 생성해 IK를 검사한다. 임의의
영 자세가 아니라 테이블 근처 준비 자세인 ``HOME_Q``에서 시작해 각 관절을 최대
``IK_TEST_SPREAD`` rad만큼 움직인다. 여기서 도달 가능 작업 공간은 전체 관절 범위의
비현실적인 자세가 아니라 실제 테이블 물체에 접근할 때 사용하는 영역을 뜻한다.
각 목표는 test-only ``solve_offline_pose_multistart``로 풀며 홈 초기값이 수렴하지 않으면 몇 개의 무작위
초기값으로 재시도한다. 목표의 95% 이상이 위치 오차 5 mm, 자세 오차 5도 미만으로
수렴해야 한다.

Part 2는 텔레옵 없이 스크립트로 홈, 캔 위 사전 파지, 3 cm/s 직선 접근, Phase 2
파지, 10 cm 들기를 10번 실행한다. 성공률은 7/10 이상이어야 한다. 자율 인지나 계획
기능이 아니라 팔·손·IK 통합을 검증하는 회귀 harness다.

Headless 실행: ``python3 tests/test_phase_3.py``
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "arm_hand.xml"

from ffw_sh5_grasp.control import arm as arm_control  # noqa: E402
from ffw_sh5_grasp.control import grasp  # noqa: E402
import ik  # noqa: E402
import kinematics  # noqa: E402
from offline_pose_ik import solve_offline_pose_multistart  # noqa: E402

ARM_JOINTS = [f"arm_r_joint{i}" for i in range(1, 8)]

# 캔에서 뒤로 10 cm, 위로 20 cm 떨어져 테이블과 충분한 여유가 있는 준비 자세다.
# ``grasp_target`` site를 수정한 뒤 다시 계산했다. 이전에는 ``hand_only.xml`` 캔의
# 월드 좌표를 손바닥 로컬 오프셋으로 잘못 사용해 수정 전의 HOME_Q, q_pregrasp와
# q_grasp가 모두 손의 잘못된 물리 지점을 목표로 삼았다.
HOME_Q = np.array([-0.225, -0.394, 0.682, -2.613, -0.704, 0.843, -1.218])
IK_TEST_SPREAD = 0.2  # 단위 시험의 도달 가능 작업 공간을 정하는 관절별 범위(rad)

N_IK_SAMPLES = 100
POS_TOL = 0.005  # 위치 허용 오차 5 mm
ORI_TOL_DEG = 5.0
IK_SUCCESS_RATE_TARGET = 0.95

N_PICK_TRIALS = 10
PICK_SUCCESS_RATE_TARGET = 0.7
APPROACH_SPEED = 0.03  # 접근 속도(m/s)
PRE_GRASP_OFFSET = np.array([0.0, 0.0, 0.10])  # 캔 바로 위에서 시작해 아래로 접근한다.
# ``grasp_target`` site가 캔의 월드 좌표를 손바닥 로컬 오프셋으로 잘못 사용했을 때는
# 잘못된 기하 때문에 중지 MCP 관절이 테이블에 스쳤다. site 자체를 고친 현재 값은 0이다.
# 향후 캔이나 테이블 배치에서 여유 문제가 다시 생기면 캔을 띄우지 말고 이 값을 높여야
# 한다. 캔은 실제 freejoint가 있어 생성 높이와 무관하게 테이블 위로 자유 낙하한다.
GRASP_TARGET_OFFSET = np.array([0.0, 0.0, 0.0])
RAMP_TIME = 1.0
SETTLE_TIME = 1.0
LIFT_HEIGHT = 0.10
LIFT_SPEED = 0.02
POST_LIFT_HOLD = 3.0
MIN_NET_LIFT = 0.08
CAN_NOISE = 0.005


def run_fk_jacobian_test(solver):
    """파싱한 트리의 FK/Jacobian이 유한차분 및 엔진 자세와 일치하는지 검사한다."""
    state = solver.forward_kinematics(HOME_Q)
    epsilon = 1e-6
    numerical = np.zeros_like(state.jacobian)
    for index in range(len(HOME_Q)):
        q_plus, q_minus = HOME_Q.copy(), HOME_Q.copy()
        q_plus[index] += epsilon
        q_minus[index] -= epsilon
        plus = solver.forward_kinematics(q_plus)
        minus = solver.forward_kinematics(q_minus)
        numerical[:3, index] = (plus.position - minus.position) / (2.0 * epsilon)
        numerical[3:, index] = kinematics.shortest_orientation_error(
            plus.quaternion, minus.quaternion) / (2.0 * epsilon)

    max_error = float(np.max(np.abs(state.jacobian - numerical)))
    quaternion_unit = abs(np.linalg.norm(state.quaternion) - 1.0) < 1e-12
    double_cover_error = np.linalg.norm(
        kinematics.shortest_orientation_error(state.quaternion, -state.quaternion))
    reference = mujoco.MjData(solver.model)
    reference.qpos[:] = solver.tree.qpos0
    reference.qpos[solver.qpos_adrs] = HOME_Q
    mujoco.mj_forward(solver.model, reference)
    engine_position = np.asarray(reference.site_xpos[solver.site_id]).copy()
    engine_quaternion = np.zeros(4)
    mujoco.mju_mat2Quat(
        engine_quaternion, reference.site_xmat[solver.site_id])
    engine_error = max(
        float(np.max(np.abs(state.position - engine_position))),
        float(np.linalg.norm(kinematics.shortest_orientation_error(
            state.quaternion, engine_quaternion))),
    )
    tree_ok = (
        solver.tree.site_by_name[solver.site_name].id == solver.site_id
        and all(name in solver.tree.joint_by_name for name in solver.joint_names)
        and not hasattr(solver, "data")
    )
    ok = (max_error < 1e-5 and engine_error < 1e-10 and tree_ok
          and quaternion_unit and double_cover_error < 1e-12)
    print(f"FK/Jacobian test: max_fd_error={max_error:.2e} quaternion_unit={quaternion_unit} "
          f"engine_pose_error={engine_error:.2e} parsed_tree={tree_ok} "
          f"double_cover={double_cover_error:.1e}: {'OK' if ok else 'FAIL'}")
    return ok


def run_ik_unit_test(model, solver, rng):
    """도달 가능한 무작위 pose에서 단일 팔 IK 수렴률과 오차 분포를 검사한다."""
    joint_ranges = np.array([model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                              for n in ARM_JOINTS])
    scratch = mujoco.MjData(model)

    successes = 0
    pos_errs, ori_errs = [], []
    for _ in range(N_IK_SAMPLES):
        q_target = np.clip(HOME_Q + rng.uniform(-IK_TEST_SPREAD, IK_TEST_SPREAD, size=7),
                            joint_ranges[:, 0], joint_ranges[:, 1])
        for qadr, val in zip(solver.qpos_adrs, q_target):
            scratch.qpos[qadr] = val
        mujoco.mj_forward(model, scratch)
        target_pos = scratch.site_xpos[solver.site_id].copy()
        target_quat = np.zeros(4)
        mujoco.mju_mat2Quat(target_quat, scratch.site_xmat[solver.site_id])

        _, pos_err, ori_err, converged = solve_offline_pose_multistart(
            solver, HOME_Q, target_pos, target_quat, rng,
            success_pos_tol=POS_TOL, success_ori_tol=np.radians(ORI_TOL_DEG))
        pos_errs.append(pos_err)
        ori_errs.append(np.degrees(ori_err))
        if converged:
            successes += 1

    rate = successes / N_IK_SAMPLES
    print(f"IK unit test: {successes}/{N_IK_SAMPLES} converged ({rate*100:.0f}%), "
          f"target >= {IK_SUCCESS_RATE_TARGET*100:.0f}%")
    print(f"  pos_err: median={np.median(pos_errs)*1000:.3f}mm max={np.max(pos_errs)*1000:.3f}mm")
    print(f"  ori_err: median={np.median(ori_errs):.3f}deg max={np.max(ori_errs):.3f}deg")
    return rate >= IK_SUCCESS_RATE_TARGET


def _read_arm_q(model, data):
    """오른팔 7개 관절의 현재 qpos를 solver 순서로 반환한다."""
    return np.array([data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]]
                      for n in ARM_JOINTS])


def _hold(model, data, controller, q_des, duration, dt, grasp_frac=None, thumb_frac=None):
    """``duration``초 동안 매 스텝 팔을 토크 제어해 ``q_des``로 구동한다.

    motor 액추에이터는 이전 position 액추에이터와 달리 오래된 ctrl 값을 유지하는
    기능이 없으므로 매 스텝 새 토크가 필요하다. 선택적으로 일정 비율의
    ``grasp.apply_grasp``도 적용한다.
    """
    n = int(duration / dt)
    for _ in range(n):
        controller.apply(data, q_des)
        if grasp_frac is not None:
            grasp.apply_grasp(model, data, grasp=grasp_frac, thumb=thumb_frac)
        mujoco.mj_step(model, data)


def _move(model, data, controller, q_from, q_to, duration, dt, grasp_frac=None, thumb_frac=None):
    """``_hold``와 같지만 ``duration``초 동안 q_des를 q_from에서 q_to로 선형 보간한다."""
    n = int(duration / dt)
    for i in range(n):
        frac = i / n
        controller.apply(data, q_from + frac * (q_to - q_from))
        if grasp_frac is not None:
            grasp.apply_grasp(model, data, grasp=grasp_frac, thumb=thumb_frac)
        mujoco.mj_step(model, data)


def run_pick_trial(model, data, solver, controller, rng):
    """무작위 캔에 접근·파지·들기를 실행하고 단일 pick 성공 정보를 반환한다."""
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    can_qadr = model.jnt_qposadr[can_jid]
    can_pos0 = data.qpos[can_qadr : can_qadr + 3].copy()
    can_pos0[:3] += rng.uniform(-CAN_NOISE, CAN_NOISE, size=3)
    data.qpos[can_qadr : can_qadr + 3] = can_pos0
    mujoco.mj_forward(model, data)

    target_quat = np.array([0.5, 0.5, 0.5, 0.5])  # hand_only.xml에서 검증한 파지 자세다.
    dt = model.opt.timestep

    # 접근 축에서 뒤로 물러난 사전 파지와 최종 파지 자세의 IK를 푼다. grasp_target_pos는
    # 손이 테이블과 간격을 두도록 실제 캔 중심보다 GRASP_TARGET_OFFSET만큼 위를 향한다.
    # 잡음과 들기 측정에 쓰는 can_pos0 자체는 바꾸지 않는다.
    grasp_target_pos = can_pos0 + GRASP_TARGET_OFFSET
    pregrasp_pos = grasp_target_pos + PRE_GRASP_OFFSET
    q_pregrasp, perr, oerr, ok1 = solve_offline_pose_multistart(
        solver, HOME_Q, pregrasp_pos, target_quat, rng)
    q_grasp, perr2, oerr2, ok2 = solve_offline_pose_multistart(
        solver, q_pregrasp, grasp_target_pos, target_quat, rng)
    if not (ok1 and ok2):
        return {"success": False, "reason": "ik_failed", "net_lift": 0.0}

    q_home = _read_arm_q(model, data)

    # 1) 중력·코리올리 전향 보상과 PD 토크 제어로 팔을 사전 파지 자세로 옮긴다.
    # 기존 position 액추에이터에서 약 15~20 mm의 site 잔류 오차를 확인한 뒤 이 방식으로
    # 교체했다.
    _move(model, data, controller, q_home, q_pregrasp, 3.0, dt, grasp_frac=0.0, thumb_frac=0.0)
    _hold(model, data, controller, q_pregrasp, 1.0, dt, grasp_frac=0.0, thumb_frac=0.0)

    # 2) 사전 파지에서 파지 자세까지 APPROACH_SPEED로 직선에 가깝게 접근한다. 같은 목표
    # 자세를 공유하고 가까이 있는 두 IK 해를 관절 공간에서 보간한다.
    approach_dist = np.linalg.norm(PRE_GRASP_OFFSET)
    approach_time = approach_dist / APPROACH_SPEED
    _move(model, data, controller, q_pregrasp, q_grasp, approach_time, dt, grasp_frac=0.0, thumb_frac=0.0)
    _hold(model, data, controller, q_grasp, 1.0, dt, grasp_frac=0.0, thumb_frac=0.0)

    # 3) Phase 2 파지 순서대로 서서히 닫고 정착시키며 팔은 q_grasp를 유지한다.
    n = int(RAMP_TIME / dt)
    for i in range(n):
        frac = i / n
        controller.apply(data, q_grasp)
        grasp.apply_grasp(model, data, grasp=frac, thumb=frac)
        mujoco.mj_step(model, data)
    _hold(model, data, controller, q_grasp, SETTLE_TIME, dt, grasp_frac=1.0, thumb_frac=1.0)

    grasped = grasp.is_grasped(model, data)
    can_z_before_lift = data.qpos[can_qadr + 2]

    # 4) IK 목표 자체를 LIFT_HEIGHT만큼 올려 다시 풀고 새 자세로 서보 제어한다.
    lift_target_pos = grasp_target_pos + np.array([0, 0, LIFT_HEIGHT])
    q_lift, _, _, _ = solve_offline_pose_multistart(
        solver, q_grasp, lift_target_pos, target_quat, rng)
    lift_time = LIFT_HEIGHT / LIFT_SPEED
    _move(model, data, controller, q_grasp, q_lift, lift_time, dt, grasp_frac=1.0, thumb_frac=1.0)
    _hold(model, data, controller, q_lift, POST_LIFT_HOLD, dt, grasp_frac=1.0, thumb_frac=1.0)

    net_lift = data.qpos[can_qadr + 2] - can_z_before_lift
    return {
        "success": grasped and net_lift >= MIN_NET_LIFT,
        "reason": "ok",
        "net_lift": net_lift,
    }


def run_pick_test(model, solver, controller, rng):
    """단일 팔 pick trial을 반복해 Phase 3 최소 성공률을 판정한다."""
    data = mujoco.MjData(model)
    results = []
    for trial in range(N_PICK_TRIALS):
        r = run_pick_trial(model, data, solver, controller, rng)
        results.append(r)
        print(f"  pick trial {trial}: success={r['success']} net_lift={r['net_lift']*100:.2f}cm reason={r['reason']}")
    n_success = sum(r["success"] for r in results)
    rate = n_success / N_PICK_TRIALS
    print(f"Pick test: {n_success}/{N_PICK_TRIALS} ({rate*100:.0f}%), target >= {PICK_SUCCESS_RATE_TARGET*100:.0f}%")
    return rate >= PICK_SUCCESS_RATE_TARGET


def main():
    """자체 FK/Jacobian, 단일 팔 IK와 물리 pick Phase 3 gate를 실행한다."""
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    solver = ik.InverseKinematics(model, "grasp_target", ARM_JOINTS)
    controller = arm_control.ArmTorqueController(model, ARM_JOINTS)
    rng = np.random.default_rng(0)

    fk_ok = run_fk_jacobian_test(solver)
    ik_ok = run_ik_unit_test(model, solver, rng)
    pick_ok = run_pick_test(model, solver, controller, rng)

    ok = fk_ok and ik_ok and pick_ok
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
