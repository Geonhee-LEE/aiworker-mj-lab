"""Phase 4 전체 로봇 고정 베이스 장면 회귀 시험.

``models/full_scene.xml``은 Phase 1~3에서 검증한 오른팔·손 물리를 전체 FFW-SH5에
그대로 넣는다. capsule 충돌, 파지 시너지, IK site 오프셋과 HOME_Q를 유지하고, 베이스는
고정하며 리프트와 머리는 실제 액추에이터로 남긴다. 양팔은 Phase 3과 같은 motor 및
전향 토크 제어를 사용하고 양손 capsule 충돌은 대칭이다. 모바일 베이스와 리프트로
달라진 arm_base 높이는 테이블과 캔 배치에 반영해 기존 검증값을 오른팔에 그대로
사용한다. 이 가정은 아래 Part 1에서 직접 검증한다.

Part 1은 ``home`` 키프레임에서 양팔에 전향 보상과 PD 토크를 적용하고 리프트, 머리와
손가락은 각 position 액추에이터로 5초간 유지한다. Phase 0과 같은 최대 가속도 기준으로
발산하지 않고, 양팔 ``grasp_target`` site가 키프레임 위치에서 2 mm 미만으로 움직이는지
확인한다.

Part 2는 ``test_phase_3.py``와 같은 홈, 사전 파지, 접근, 파지, 들기 순서를 오른손과
캔에 실행한다. 왼팔, 리프트와 머리는 매 스텝 키프레임 자세로 유지해 나머지 몸체가
검증된 오른손 경로를 방해하지 않는지 확인한다. 성공률 기준은 Phase 3과 같은 7/10이다.

왼손 파지 시너지는 대칭 기하이며 별도 캔으로 독립 회귀 시험하지 않는다. Part 1의
유지 검사는 발산과 site 이동을 확인하지만 Part 2는 실제 검증 범위에 맞게 오른손만
시험한다.

Headless 실행: ``python3 tests/test_phase_4.py``
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "full_scene.xml"

from ffw_sh5_grasp.control import arm as arm_control  # noqa: E402
from ffw_sh5_grasp.control import grasp  # noqa: E402
from ffw_sh5_grasp.kinematics import JointSpaceKinematics  # noqa: E402
from offline_pose_ik import solve_offline_pose_multistart  # noqa: E402

ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]

# 관련 ffw-sh5-mujoco 저장소의 ``Controller.reset()`` 휴지 자세와 일치한다. 4번
# 팔꿈치 관절만 -90도이고 나머지는 0도다. 이전 ``arm_hand.xml``에서 가져온 캔 접근
# 자세와 달리 특정 캔 기하에 묶이지 않은 일반적인 IK 다중 시작 초기값이다. 실제 사전
# 파지와 파지 자세 탐색은 test-only multistart 재시도가 담당한다.
HOME_Q_R = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
HOME_Q_L = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
LIFT_HOME = -0.39
THUMB_PRESHAPE_R = {"finger_r_joint1": 0.131, "finger_r_joint2": -1.309}
THUMB_PRESHAPE_L = {"finger_l_joint1": 0.131, "finger_l_joint2": 1.309}

HOLD_DURATION = 5.0
HOLD_QACC_LIMIT = 1e5
HOLD_SITE_DRIFT_LIMIT = 0.002  # 사이트 허용 표류량 2 mm

N_IK_SAMPLES = 100
IK_TEST_SPREAD = 0.2
POS_TOL = 0.005
ORI_TOL_DEG = 5.0
IK_SUCCESS_RATE_TARGET = 0.95

N_PICK_TRIALS = 10
PICK_SUCCESS_RATE_TARGET = 0.7
APPROACH_SPEED = 0.03
PRE_GRASP_OFFSET = np.array([0.0, 0.0, 0.10])
GRASP_TARGET_OFFSET = np.array([0.0, 0.0, 0.0])
RAMP_TIME = 1.0
SETTLE_TIME = 1.0
LIFT_HEIGHT = 0.10
LIFT_SPEED = 0.02
POST_LIFT_HOLD = 3.0
MIN_NET_LIFT = 0.08
CAN_NOISE = 0.005


def _reset_home(model, data):
    """전신 모델을 home 키프레임으로 되돌리고 파생 상태를 계산한다."""
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)


def _hold_whole_body(model, data, ctrl_r, ctrl_l, q_r, q_l, grasp_r=0.0, thumb_r=0.0):
    """매 스텝 오른팔은 q_r, 왼팔은 q_l로 전향 보상과 PD 토크 제어한다.

    리프트, 머리와 손가락은 각 position 액추에이터를 사용한다. 이 액추에이터는 ctrl을
    한 번 설정하면 되지만 시험 대상인 손 파지는 ``grasp.apply_grasp``를 다시 적용한다.
    """
    ctrl_r.apply(data, q_r)
    ctrl_l.apply(data, q_l)
    grasp.apply_grasp(model, data, grasp=grasp_r, thumb=thumb_r, side="r")


def run_hold_test(model):
    """양팔 홈 자세 유지 중 가속도와 손 site drift가 허용 범위인지 검사한다."""
    data = mujoco.MjData(model)
    _reset_home(model, data)
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)

    site_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r")
    site_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l")
    p0_r = data.site_xpos[site_r].copy()
    p0_l = data.site_xpos[site_l].copy()

    dt = model.opt.timestep
    n = int(HOLD_DURATION / dt)
    max_qacc = 0.0
    for _ in range(n):
        _hold_whole_body(model, data, ctrl_r, ctrl_l, HOME_Q_R, HOME_Q_L,
                          grasp_r=0.0, thumb_r=0.0)
        mujoco.mj_step(model, data)
        max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))

    drift_r = float(np.linalg.norm(data.site_xpos[site_r] - p0_r))
    drift_l = float(np.linalg.norm(data.site_xpos[site_l] - p0_l))
    print(f"Hold test: max|qacc|={max_qacc:.3f} (limit {HOLD_QACC_LIMIT:.0e}), "
          f"site_r drift={drift_r*1000:.3f}mm site_l drift={drift_l*1000:.3f}mm "
          f"(limit {HOLD_SITE_DRIFT_LIMIT*1000:.0f}mm)")
    ok = (max_qacc < HOLD_QACC_LIMIT and drift_r < HOLD_SITE_DRIFT_LIMIT
          and drift_l < HOLD_SITE_DRIFT_LIMIT)
    return ok


def run_ik_unit_test(model, solver, rng):
    """전신 모델 문맥에서 오른팔 IK의 무작위 pose 수렴률을 검사한다."""
    joint_ranges = np.array([model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                              for n in ARM_R])
    scratch = mujoco.MjData(model)
    _reset_home(model, scratch)  # lift_joint와 상위 문맥 관절의 초기값도 함께 넣는다.

    successes = 0
    pos_errs, ori_errs = [], []
    for _ in range(N_IK_SAMPLES):
        q_target = np.clip(HOME_Q_R + rng.uniform(-IK_TEST_SPREAD, IK_TEST_SPREAD, size=7),
                            joint_ranges[:, 0], joint_ranges[:, 1])
        for qadr, val in zip(solver.qpos_adrs, q_target):
            scratch.qpos[qadr] = val
        mujoco.mj_forward(model, scratch)
        target_pos = scratch.site_xpos[solver.site_id].copy()
        target_quat = np.zeros(4)
        mujoco.mju_mat2Quat(target_quat, scratch.site_xmat[solver.site_id])

        _, pos_err, ori_err, converged = solve_offline_pose_multistart(
            solver, HOME_Q_R, target_pos, target_quat, rng,
            success_pos_tol=POS_TOL, success_ori_tol=np.radians(ORI_TOL_DEG),
            context_qpos=scratch.qpos)
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


def _read_arm_q(model, data, joint_names):
    """지정 관절 이름 순서로 현재 팔 qpos 벡터를 읽는다."""
    return np.array([data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]]
                      for n in joint_names])


def _hold(model, data, ctrl_r, ctrl_l, q_r_des, duration, dt, grasp_frac=None, thumb_frac=None):
    """양팔 목표를 유지하며 선택적 오른손 파지 명령과 물리 step을 적용한다."""
    n = int(duration / dt)
    for _ in range(n):
        ctrl_r.apply(data, q_r_des)
        ctrl_l.apply(data, HOME_Q_L)
        if grasp_frac is not None:
            grasp.apply_grasp(model, data, grasp=grasp_frac, thumb=thumb_frac, side="r")
        mujoco.mj_step(model, data)


def _move(model, data, ctrl_r, ctrl_l, q_from, q_to, duration, dt, grasp_frac=None, thumb_frac=None):
    """오른팔 목표를 보간하고 왼팔을 유지하며 선택적 파지와 물리 step을 적용한다."""
    n = int(duration / dt)
    for i in range(n):
        frac = i / n
        ctrl_r.apply(data, q_from + frac * (q_to - q_from))
        ctrl_l.apply(data, HOME_Q_L)
        if grasp_frac is not None:
            grasp.apply_grasp(model, data, grasp=grasp_frac, thumb=thumb_frac, side="r")
        mujoco.mj_step(model, data)


def run_pick_trial(model, data, solver, ctrl_r, ctrl_l, rng):
    """전신 장면에서 오른팔 접근·파지·들기 한 회를 실행해 결과를 반환한다."""
    _reset_home(model, data)
    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    can_qadr = model.jnt_qposadr[can_jid]
    can_pos0 = data.qpos[can_qadr:can_qadr + 3].copy()
    can_pos0[:3] += rng.uniform(-CAN_NOISE, CAN_NOISE, size=3)
    data.qpos[can_qadr:can_qadr + 3] = can_pos0
    mujoco.mj_forward(model, data)

    target_quat = np.array([0.5, 0.5, 0.5, 0.5])
    dt = model.opt.timestep

    grasp_target_pos = can_pos0 + GRASP_TARGET_OFFSET
    pregrasp_pos = grasp_target_pos + PRE_GRASP_OFFSET
    ctx = data.qpos.copy()  # lift_joint 같은 상위 관절은 ik.py의 context_qpos 설명을 따른다.
    q_pregrasp, perr, oerr, ok1 = solve_offline_pose_multistart(
        solver, HOME_Q_R, pregrasp_pos, target_quat, rng, context_qpos=ctx)
    q_grasp, perr2, oerr2, ok2 = solve_offline_pose_multistart(
        solver, q_pregrasp, grasp_target_pos, target_quat, rng, context_qpos=ctx)
    if not (ok1 and ok2):
        return {"success": False, "reason": "ik_failed", "net_lift": 0.0}

    q_home = _read_arm_q(model, data, ARM_R)

    _move(model, data, ctrl_r, ctrl_l, q_home, q_pregrasp, 3.0, dt, grasp_frac=0.0, thumb_frac=0.0)
    _hold(model, data, ctrl_r, ctrl_l, q_pregrasp, 1.0, dt, grasp_frac=0.0, thumb_frac=0.0)

    approach_dist = np.linalg.norm(PRE_GRASP_OFFSET)
    approach_time = approach_dist / APPROACH_SPEED
    _move(model, data, ctrl_r, ctrl_l, q_pregrasp, q_grasp, approach_time, dt, grasp_frac=0.0, thumb_frac=0.0)
    _hold(model, data, ctrl_r, ctrl_l, q_grasp, 1.0, dt, grasp_frac=0.0, thumb_frac=0.0)

    n = int(RAMP_TIME / dt)
    for i in range(n):
        frac = i / n
        ctrl_r.apply(data, q_grasp)
        ctrl_l.apply(data, HOME_Q_L)
        grasp.apply_grasp(model, data, grasp=frac, thumb=frac, side="r")
        mujoco.mj_step(model, data)
    _hold(model, data, ctrl_r, ctrl_l, q_grasp, SETTLE_TIME, dt, grasp_frac=1.0, thumb_frac=1.0)

    grasped = grasp.is_grasped(model, data, side="r")
    can_z_before_lift = data.qpos[can_qadr + 2]

    lift_target_pos = grasp_target_pos + np.array([0, 0, LIFT_HEIGHT])
    q_lift, _, _, _ = solve_offline_pose_multistart(
        solver, q_grasp, lift_target_pos, target_quat, rng, context_qpos=ctx)
    lift_time = LIFT_HEIGHT / LIFT_SPEED
    _move(model, data, ctrl_r, ctrl_l, q_grasp, q_lift, lift_time, dt, grasp_frac=1.0, thumb_frac=1.0)
    _hold(model, data, ctrl_r, ctrl_l, q_lift, POST_LIFT_HOLD, dt, grasp_frac=1.0, thumb_frac=1.0)

    net_lift = data.qpos[can_qadr + 2] - can_z_before_lift
    return {
        "success": grasped and net_lift >= MIN_NET_LIFT,
        "reason": "ok",
        "net_lift": net_lift,
    }


def run_pick_test(model, solver, ctrl_r, ctrl_l, rng):
    """전신 pick trial을 반복해 Phase 4 성공률 기준을 판정한다."""
    data = mujoco.MjData(model)
    results = []
    for trial in range(N_PICK_TRIALS):
        r = run_pick_trial(model, data, solver, ctrl_r, ctrl_l, rng)
        results.append(r)
        print(f"  pick trial {trial}: success={r['success']} net_lift={r['net_lift']*100:.2f}cm reason={r['reason']}")
    n_success = sum(r["success"] for r in results)
    rate = n_success / N_PICK_TRIALS
    print(f"Pick test: {n_success}/{N_PICK_TRIALS} ({rate*100:.0f}%), target >= {PICK_SUCCESS_RATE_TARGET*100:.0f}%")
    return rate >= PICK_SUCCESS_RATE_TARGET


def main():
    """양팔 유지·IK·물리 pick을 묶은 Phase 4 통합 gate를 실행한다."""
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    solver = JointSpaceKinematics(model, "grasp_target_r", ARM_R)
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
    rng = np.random.default_rng(0)

    hold_ok = run_hold_test(model)
    ik_ok = run_ik_unit_test(model, solver, rng)
    pick_ok = run_pick_test(model, solver, ctrl_r, ctrl_l, rng)

    ok = hold_ok and ik_ok and pick_ok
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
