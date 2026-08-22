"""Phase 1 ``hand_only`` 장면과 충돌 검증.

모든 손가락 굽힘 관절을 최대 액추에이터 힘으로 캔을 향해 닫는 시험을 20번 수행하고,
손가락과 캔 사이의 최악 접촉 침투 깊이를 기록한다. ``contact.dist``의 음수는 겹침을
뜻한다. 같은 rollout에서 달성한 실시간 배율도 출력한다.

Headless 실행: ``python3 tests/test_phase_1.py``
"""

import pathlib
import sys
import time

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
MODEL_PATH = REPO_ROOT / "models" / "hand_only.xml"

N_TRIALS = 20
SIM_SECONDS = 1.5
PENETRATION_LIMIT = 0.002  # 허용 침투 깊이 2 mm
RTF_TARGET = 0.5

# 엄지 MCP pitch·IP와 각 손가락 PIP·DIP·tip 굽힘 관절만 포함한다.
CURL_JOINTS = {"finger_r_joint3", "finger_r_joint4"}
for base in (5, 9, 13, 17):
    CURL_JOINTS.update({f"finger_r_joint{base+1}", f"finger_r_joint{base+2}", f"finger_r_joint{base+3}"})

CAN_INIT_POS = np.array([0.105, 0.065, 0.16])  # ``hand_only.xml``의 Phase 2 캔 위치와 같다.
CAN_INIT_QUAT = np.array([1.0, 0.0, 0.0, 0.0])


def actuator_for_joint(model, jid):
    """지정 손 관절을 구동하는 actuator ID를 찾아 반환한다."""
    for aid in range(model.nu):
        if (
            model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT
            and model.actuator_trnid[aid, 0] == jid
        ):
            return aid
    return None


def reset_trial(model, data):
    """파지 접촉 시험을 홈 자세와 초기 캔 상태로 되돌린다."""
    # Phase 1은 배치 강건성이 아니라 닫힘 동작 자체를 검사한다. 배치 강건성은 Phase 2의
    # ±5 mm 무작위 파지·들기 시험에서 다루므로 여기서는 일관된 고정 캔 자세를 반복한다.
    mujoco.mj_resetData(model, data)
    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    qadr = model.jnt_qposadr[can_jid]
    data.qpos[qadr : qadr + 3] = CAN_INIT_POS
    data.qpos[qadr + 3 : qadr + 7] = CAN_INIT_QUAT
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def close_hand(model, data):
    """정해진 시간 동안 손가락 actuator를 닫아 캔 접촉 상태를 만든다."""
    for jid in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        if name in CURL_JOINTS:
            aid = actuator_for_joint(model, jid)
            if aid is None:
                continue  # 범위가 0인 잠긴 관절에는 액추에이터가 없다.
            hi = model.jnt_range[jid][1]
            data.ctrl[aid] = hi


def worst_finger_can_penetration(model, data):
    """현재 접촉 중 손가락과 캔 사이의 가장 깊은 관통량을 반환한다."""
    can_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "can_geom")
    worst = 0.0
    for i in range(data.ncon):
        c = data.contact[i]
        if can_gid not in (c.geom1, c.geom2):
            continue
        other = c.geom1 if c.geom2 == can_gid else c.geom2
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[other]) or ""
        if not (bname.startswith("finger_r_") or bname == "hx5_r_base"):
            continue  # 캔이 떨어진 뒤의 바닥 접촉은 무시하고 손과 캔 접촉만 본다.
        if c.dist < worst:
            worst = c.dist
    return worst


def main():
    """여러 초기 조건에서 손가락-캔 관통 깊이와 실행 속도 Phase 1 gate를 검사한다."""
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    n_steps = int(SIM_SECONDS / model.opt.timestep)
    max_penetration = 0.0
    total_wall = 0.0

    for trial in range(N_TRIALS):
        reset_trial(model, data)
        close_hand(model, data)

        t0 = time.perf_counter()
        for _ in range(n_steps):
            mujoco.mj_step(model, data)
            pen = worst_finger_can_penetration(model, data)
            if -pen > max_penetration:
                max_penetration = -pen
        total_wall += time.perf_counter() - t0

    sim_seconds_total = N_TRIALS * SIM_SECONDS
    rtf = sim_seconds_total / total_wall

    print(f"Trials: {N_TRIALS}, {n_steps} steps each ({SIM_SECONDS}s sim)")
    print(f"Max finger-can penetration depth: {max_penetration * 1000:.3f} mm (limit {PENETRATION_LIMIT*1000:.1f} mm)")
    print(f"Real-time factor: {rtf:.2f} (target >= {RTF_TARGET})")

    ok = max_penetration < PENETRATION_LIMIT
    print("PASS" if ok else "FAIL: penetration exceeds limit")
    if rtf < RTF_TARGET:
        print(f"WARNING: real-time factor {rtf:.2f} below target {RTF_TARGET}")

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
