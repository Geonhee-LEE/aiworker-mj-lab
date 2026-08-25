"""Phase 2 고정 손 파지와 들기 시험으로 프로젝트의 핵심 검증이다.

각 시험에서 캔의 x/y/z를 ±5 mm 움직인 뒤 ``pregrasp`` 키프레임으로 초기화한다.
``grasp``와 ``thumb``을 계단 입력이 아닌 변화율 제한으로 0에서 1까지 올리고 접촉력
기반 파지를 확인한다. 그 다음 mocap 기준과 용접된 손을 2 cm/s로 10 cm 올려 5초간
유지한다. 정착과 유연성에 따른 처짐을 고려해 캔이 8 cm 이상 올라가고 마지막 유지
중 손 좌표계 미끄러짐이 1 cm 미만이면 성공이다.

Headless 실행: ``python3 tests/test_phase_2.py``
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "hand_only.xml"

from ffw_sh5_grasp.control import grasp  # noqa: E402

N_TRIALS = 10
NOISE = 0.005  # 배치 오차 ±5 mm
RAMP_TIME = 1.0  # grasp와 thumb을 0에서 1로 올리는 시간(초)
SETTLE_TIME = 1.0  # 들어 올리기 전 닫힌 파지를 유지하는 시간(초)
LIFT_HEIGHT = 0.10  # 들기 높이(m)
LIFT_SPEED = 0.02  # 들기 속도(m/s, 2 cm/s)
POST_LIFT_HOLD = 5.0  # 들기 후 유지 시간(초)
MIN_NET_LIFT = 0.08  # 하중에 따른 처짐을 고려한 실제 들기 최소 거리(m)
MAX_SLIP = 0.01  # 들기 후 유지 중 손 좌표계에서 측정한 최대 미끄러짐(m)

SUCCESS_RATE_TARGET = 0.8  # 목표 성공 횟수 8/10


def hand_frame_offset(model, data, can_qadr):
    """들기 이동을 제외해 미끄러짐만 측정하도록 움직이는 손 기준 캔 위치를 반환한다."""
    hand_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hx5_r_base")
    return data.qpos[can_qadr : can_qadr + 3] - data.xpos[hand_bid]


def run_trial(model, data, rng):
    """무작위 캔 위치에서 손을 닫고 들어 올려 파지 성공·미끄러짐 지표를 반환한다."""
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "pregrasp")
    mujoco.mj_resetDataKeyframe(model, data, key_id)

    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    qadr = model.jnt_qposadr[can_jid]
    data.qpos[qadr : qadr + 3] += rng.uniform(-NOISE, NOISE, size=3)
    mujoco.mj_forward(model, data)

    mocap_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand_mocap")
    mocap_id = model.body_mocapid[mocap_bid]
    mocap_start = data.mocap_pos[mocap_id].copy()

    dt = model.opt.timestep

    # 1) grasp와 thumb을 서서히 닫는다.
    t = 0.0
    while t < RAMP_TIME:
        frac = t / RAMP_TIME
        grasp.apply_grasp(model, data, grasp=frac, thumb=frac, side="r")
        mujoco.mj_step(model, data)
        t += dt

    # 2) 닫힌 상태로 정착시킨다.
    t = 0.0
    while t < SETTLE_TIME:
        grasp.apply_grasp(model, data, grasp=1.0, thumb=1.0, side="r")
        mujoco.mj_step(model, data)
        t += dt

    grasped_before_lift = grasp.is_grasped(model, data, side="r")
    can_z_before_lift = data.qpos[qadr + 2]

    # 3) mocap 기준을 LIFT_SPEED로 올린다. 용접 제약이 hx5_r_base와 잡은 물체를 함께
    # 끌어 올린다. 동적 body의 qpos 덮어쓰기가 아니라 mocap 목표 갱신이다.
    lift_duration = LIFT_HEIGHT / LIFT_SPEED
    t = 0.0
    while t < lift_duration:
        data.mocap_pos[mocap_id] = mocap_start + np.array(
            [0, 0, min(t, lift_duration) * LIFT_SPEED]
        )
        grasp.apply_grasp(model, data, grasp=1.0, thumb=1.0, side="r")
        mujoco.mj_step(model, data)
        t += dt
    data.mocap_pos[mocap_id] = mocap_start + np.array([0, 0, LIFT_HEIGHT])

    # 4) 최대 높이를 유지하며 손 좌표계에서 미끄러짐을 추적한다.
    offsets = []
    t = 0.0
    while t < POST_LIFT_HOLD:
        grasp.apply_grasp(model, data, grasp=1.0, thumb=1.0, side="r")
        mujoco.mj_step(model, data)
        offsets.append(hand_frame_offset(model, data, qadr).copy())
        t += dt

    can_z_after_hold = data.qpos[qadr + 2]
    net_lift = can_z_after_hold - can_z_before_lift
    offsets = np.array(offsets)
    slip = np.linalg.norm(offsets[-1] - offsets[0])

    return {
        "grasped_before_lift": grasped_before_lift,
        "net_lift": net_lift,
        "slip": slip,
        "success": grasped_before_lift
        and net_lift >= MIN_NET_LIFT
        and slip <= MAX_SLIP,
    }


def main():
    """반복 파지·들기 성공률이 Phase 2 기준을 만족하는지 검사한다."""
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    rng = np.random.default_rng(0)

    results = []
    for trial in range(N_TRIALS):
        r = run_trial(model, data, rng)
        results.append(r)
        print(
            f"trial {trial}: grasped={r['grasped_before_lift']} "
            f"net_lift={r['net_lift'] * 100:.2f}cm slip={r['slip'] * 1000:.2f}mm "
            f"success={r['success']}"
        )

    n_success = sum(r["success"] for r in results)
    rate = n_success / N_TRIALS
    print(
        f"\nSuccess rate: {n_success}/{N_TRIALS} ({rate * 100:.0f}%), target >= {SUCCESS_RATE_TARGET * 100:.0f}%"
    )

    ok = rate >= SUCCESS_RATE_TARGET
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
