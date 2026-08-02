"""Phase 5 모바일 베이스 이동 검증.

``models/full_scene.xml``의 base_link에 기울어지지 않으면서 병진·회전할 수 있는
base_x/base_y/base_yaw 평면 관절 3개를 두고, 세 바퀴에는 실제 조향·구동 관절을
사용한다. 이 단계의 범위는 주행이며 기존 Phase 4 캔 파지 과제는 그대로 유지한다.
같은 모델에서 Phase 4가 다시 통과하는 것도 회귀 기준에 포함된다.

초기 설계의 가상 관절 직접 속도 액추에이터를 실제 바퀴와 지면 마찰 추진으로
변경했다. base_x/base_y/base_yaw는 직접 구동하지 않고 바퀴 조향·구동 관절과 바닥
접촉의 반작용으로만 움직인다. 바퀴별 조향각과 구동 속도 기구학은
``control/base.py``의 ``SwerveDrive``에 있다. 공식 약 ±2π 범위와 별도로 주입한 좁은
범위 solver는 ``test_whole_body.py``에서 검사한다.

Part 1은 MuJoCo 없이 ``BaseTeleop`` 평활화 수학과 ROBOTIS 방식 ``SwerveDrive``를
독립 검사한다. 순수 전진, 회전, 횡이동과 180도 반전 상태 머신을 알려진 바퀴 장착
기하와 비교한다.

Part 2는 qpos가 아닌 ``SwerveDrive``의 ctrl 출력으로 실제 시뮬레이션 바퀴를 구동한다.
입력 없는 정지 상태가 흐르지 않는지, 바퀴가 거의 미끄러지지 않고 굴러 베이스를 실제로
움직이는지, 테이블을 향해 주행할 때 팔·테이블 접촉으로 멈추고 관통하지 않는지 확인한다.
이는 키프레임 토큰 수 오류와 바퀴·바닥 간격이 정확히 0이어서 접촉 두 개가 사라지고
약 99% 미끄러짐이 생겼던 결함의 직접 회귀 시험이기도 하다.

Headless 실행: ``python3 tests/test_phase_5.py``
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "full_scene.xml"

from ffw_sh5_grasp.control import arm as arm_control  # noqa: E402
from ffw_sh5_grasp.control import base as base_teleop  # noqa: E402
from ffw_sh5_grasp.control import grasp  # noqa: E402

ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
HOME_Q_R = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
HOME_Q_L = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
WHEELS = ("left_wheel", "right_wheel", "rear_wheel")

QACC_LIMIT = 1e5
IDLE_DRIFT_LIMIT = 0.002  # 정지 상태 허용 표류량 2 mm


def run_unit_tests():
    ok = True

    bt = base_teleop.BaseTeleop()
    vx = vy = vyaw = 0.0
    for _ in range(2000):
        vx, vy, vyaw = bt.update({"w": True}, 0.01, yaw=0.0)
    speed = float(np.hypot(vx, vy))
    ok_a = abs(speed - base_teleop.K_SPEED) < 0.02 and vy == 0.0 and vyaw == 0.0
    print(f"  (a) BaseTeleop forward hold settles at speed={speed:.4f} "
          f"(target {base_teleop.K_SPEED}): {'OK' if ok_a else 'FAIL'}")
    ok &= ok_a

    responsive = base_teleop.BaseTeleop()
    for _ in range(round(0.6 / 0.01)):
        combined = responsive.update_body({"w": True, "left": True}, 0.01)
    combined_ok = combined.vx > 0.90 * base_teleop.K_SPEED and combined.wz > 1.4
    release_time = None
    for step in range(1, round(1.2 / 0.01) + 1):
        released = responsive.update_body({}, 0.01)
        if released.is_zero():
            release_time = step * 0.01
            break
    response_ok = combined_ok and release_time is not None and release_time < 1.0
    print(f"  (a2) combined+response: vx={combined.vx:.3f} wz={combined.wz:.3f} "
          f"release_zero={release_time}s: {'OK' if response_ok else 'FAIL'}")
    ok &= response_ok

    # 순수 전진에서는 모든 바퀴가 정면을 향하고 같은 속도로 구동되어야 한다.
    sd = base_teleop.SwerveDrive()
    for _ in range(300):
        cmds = sd.update({"w": True}, 0.01, yaw=0.0)
    steers = [abs(cmds[w][0]) for w in WHEELS]
    speeds = [cmds[w][1] for w in WHEELS]
    ok_b = all(s < 0.01 for s in steers) and max(speeds) - min(speeds) < 0.01 and speeds[0] > 0
    print(f"  (b) SwerveDrive forward: steer angles={[round(s,4) for s in steers]} "
          f"drive speeds={[round(s,3) for s in speeds]}: {'OK' if ok_b else 'FAIL'}")
    ok &= ok_b

    # 제자리 회전에서는 중심 바로 뒤의 뒷바퀴가 ±90도를 향하고 좌우 바퀴가 대칭으로
    # 원점 주위 회전에 맞게 정렬되어야 한다.
    sd2 = base_teleop.SwerveDrive()
    for _ in range(300):
        cmds2 = sd2.update({"left": True}, 0.01, yaw=0.0)
    rear_steer = cmds2["rear_wheel"][0]
    left_steer, right_steer = cmds2["left_wheel"][0], cmds2["right_wheel"][0]
    ok_c = (abs(abs(rear_steer) - np.pi / 2) < 0.02
            and abs(left_steer + right_steer) < 0.02  # 0을 기준으로 대칭이다.
            and abs(left_steer) > 0.01)
    print(f"  (c) SwerveDrive in-place yaw: rear={np.degrees(rear_steer):.1f}deg "
          f"left={np.degrees(left_steer):.1f}deg right={np.degrees(right_steer):.1f}deg: "
          f"{'OK' if ok_c else 'FAIL'}")
    ok &= ok_c

    # 순수 횡이동에서는 모든 바퀴가 전진 방향에 수직인 ±90도를 향해야 한다.
    sd3 = base_teleop.SwerveDrive()
    for _ in range(300):
        cmds3 = sd3.update({"a": True}, 0.01, yaw=0.0)
    strafe_steers = [abs(abs(cmds3[w][0]) - np.pi / 2) for w in WHEELS]
    ok_d = all(s < 0.02 for s in strafe_steers)
    print(f"  (d) SwerveDrive strafe: steer angles={[round(np.degrees(cmds3[w][0]),1) for w in WHEELS]}: "
          f"{'OK' if ok_d else 'FAIL'}")
    ok &= ok_d

    sd4 = base_teleop.SwerveDrive()
    feedback_steer = {wheel: 0.0 for wheel in WHEELS}
    moving_wheels = {wheel: 5.0 for wheel in WHEELS}
    sd4.update_twist(base_teleop.BodyTwist(0.5, 0.0, 0.0), 0.01,
                     feedback_steer, moving_wheels)
    reverse_cmd = sd4.update_twist(base_teleop.BodyTwist(-0.5, 0.0, 0.0), 0.01,
                                   feedback_steer, moving_wheels)
    ok_e = (
        sd4.reversal_phase["left_wheel"] == base_teleop.ReversalPhase.DECELERATING
        and sd4.wheel_speed_scale["left_wheel"] < 1.0
        and reverse_cmd["left_wheel"][1] > 0.0
    )
    print(f"  (e) SwerveDrive 180deg reversal: phase={sd4.reversal_phase['left_wheel'].name} "
          f"scale={sd4.wheel_speed_scale['left_wheel']:.2f} "
          f"wheel_cmd={reverse_cmd['left_wheel'][1]:.3f}: {'OK' if ok_e else 'FAIL'}")
    ok &= ok_e

    stopped = base_teleop.SwerveDrive()
    stopped_reverse = stopped.update_twist(
        base_teleop.BodyTwist(-0.5, 0.0, 0.0), 0.01,
        feedback_steer, {wheel: 0.0 for wheel in WHEELS})
    stopped_ok = (all(stopped.reversal_phase[w] == base_teleop.ReversalPhase.NORMAL for w in WHEELS)
                  and all(stopped_reverse[w][1] < 0.0 for w in WHEELS))

    stalled = base_teleop.SwerveDrive()
    for _ in range(30):
        stalled_cmd = stalled.update_twist(
            base_teleop.BodyTwist(0.0, 0.5, 0.0), 0.01,
            feedback_steer, {wheel: 0.0 for wheel in WHEELS})
    command_progress_ok = all(stalled_cmd[w][0] > 1.45 for w in WHEELS)
    ok_f = stopped_ok and command_progress_ok
    print(f"  (f) stopped reversal + lagging-feedback steering command: "
          f"direct_reverse={stopped_ok} steer_cmd="
          f"{[round(stalled_cmd[w][0], 2) for w in WHEELS]}: {'OK' if ok_f else 'FAIL'}")
    ok &= ok_f

    return ok


def _reset_home(model, data):
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)


def _make_rig(model):
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
    steer_aids = {w: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{w}_steer") for w in WHEELS}
    drive_aids = {w: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{w}_drive") for w in WHEELS}
    steer_qadrs = {
        w: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{w}_steer_joint")]
        for w in WHEELS
    }
    drive_dofs = {w: model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{w}_drive_joint")]
                  for w in WHEELS}
    base_yaw_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_yaw")]
    base_x_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_x")]
    base_y_qadr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_y")]
    base_x_dof = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_x")]
    base_y_dof = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_y")]
    base_yaw_dof = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_yaw")]
    return dict(ctrl_r=ctrl_r, ctrl_l=ctrl_l, steer_aids=steer_aids, drive_aids=drive_aids,
                steer_qadrs=steer_qadrs,
                drive_dofs=drive_dofs, base_yaw_qadr=base_yaw_qadr, base_x_qadr=base_x_qadr,
                base_y_qadr=base_y_qadr, base_x_dof=base_x_dof, base_y_dof=base_y_dof,
                base_yaw_dof=base_yaw_dof)


def _step(model, data, rig, drive, keys, frame_dt):
    yaw = data.qpos[rig["base_yaw_qadr"]]
    steering_positions = {w: float(data.qpos[qadr]) for w, qadr in rig["steer_qadrs"].items()}
    wheel_velocities = {w: float(data.qvel[dof]) for w, dof in rig["drive_dofs"].items()}
    cmds = drive.update(keys, frame_dt, yaw, steering_positions, wheel_velocities)
    dt = model.opt.timestep
    max_qacc = 0.0
    for _ in range(max(1, round(frame_dt / dt))):
        rig["ctrl_r"].apply(data, HOME_Q_R)
        rig["ctrl_l"].apply(data, HOME_Q_L)
        for wheel, (angle, speed) in cmds.items():
            data.ctrl[rig["steer_aids"][wheel]] = angle
            data.ctrl[rig["drive_aids"][wheel]] = speed
        grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="r")
        grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="l")
        mujoco.mj_step(model, data)
        max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))
    return max_qacc


def _step_twist(model, data, rig, drive, twist, frame_dt):
    steering_positions = {w: float(data.qpos[qadr]) for w, qadr in rig["steer_qadrs"].items()}
    wheel_velocities = {w: float(data.qvel[dof]) for w, dof in rig["drive_dofs"].items()}
    cmds = drive.update_twist(twist, frame_dt, steering_positions, wheel_velocities)
    max_qacc = 0.0
    for _ in range(max(1, round(frame_dt / model.opt.timestep))):
        rig["ctrl_r"].apply(data, HOME_Q_R)
        rig["ctrl_l"].apply(data, HOME_Q_L)
        for wheel, (angle, speed) in cmds.items():
            data.ctrl[rig["steer_aids"][wheel]] = angle
            data.ctrl[rig["drive_aids"][wheel]] = speed
        grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="r")
        grasp.apply_grasp(model, data, grasp=0.0, thumb=0.0, side="l")
        mujoco.mj_step(model, data)
        max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))
    return max_qacc


def _run_twist_trial(model, twist, duration):
    data = mujoco.MjData(model)
    _reset_home(model, data)
    rig = _make_rig(model)
    drive = base_teleop.SwerveDrive()
    initial = np.array([
        data.qpos[rig["base_x_qadr"]], data.qpos[rig["base_y_qadr"]],
        data.qpos[rig["base_yaw_qadr"]]])
    max_qacc = 0.0
    for _ in range(round(duration / 0.04)):
        max_qacc = max(max_qacc, _step_twist(model, data, rig, drive, twist, 0.04))
    final = np.array([
        data.qpos[rig["base_x_qadr"]], data.qpos[rig["base_y_qadr"]],
        data.qpos[rig["base_yaw_qadr"]]])
    return final - initial, max_qacc, data, rig, drive


def run_omnidirectional_regression(model):
    """실제 횡이동, 회전, 결합 twist, 반전과 내부 충돌을 검사한다."""
    audit = mujoco.MjData(model)
    _reset_home(model, audit)
    rig = _make_rig(model)
    for wheel in WHEELS:
        audit.qpos[rig["steer_qadrs"][wheel]] = np.pi / 2
    mujoco.mj_forward(model, audit)
    wheel_geoms = {mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, f"{wheel}_collision") for wheel in WHEELS}
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    internal_contacts = [contact for contact in audit.contact
                         if (contact.geom1 in wheel_geoms or contact.geom2 in wheel_geoms)
                         and floor not in (contact.geom1, contact.geom2)]

    strafe, strafe_acc, *_ = _run_twist_trial(
        model, base_teleop.BodyTwist(0.0, 0.5, 0.0), 2.0)
    strafe_ok = (strafe[1] > 0.55 and abs(strafe[0]) < 0.12
                 and abs(strafe[2]) < 0.15 and strafe_acc < QACC_LIMIT)

    yaw, yaw_acc, *_ = _run_twist_trial(
        model, base_teleop.BodyTwist(0.0, 0.0, 1.0), 2.0)
    yaw_ok = (yaw[2] > 0.8 and np.linalg.norm(yaw[:2]) < 0.10 and yaw_acc < QACC_LIMIT)

    combined, combined_acc, *_ = _run_twist_trial(
        model, base_teleop.BodyTwist(-0.4, 0.0, 0.6), 2.0)
    combined_ok = (np.linalg.norm(combined[:2]) > 0.35 and combined[2] > 0.45
                   and combined_acc < QACC_LIMIT)

    data = mujoco.MjData(model)
    _reset_home(model, data)
    rig = _make_rig(model)
    drive = base_teleop.SwerveDrive()
    max_acc = 0.0
    for _ in range(round(1.2 / 0.04)):
        max_acc = max(max_acc, _step_twist(
            model, data, rig, drive, base_teleop.BodyTwist(0.0, 0.45, 0.0), 0.04))
    positive_y = float(data.qpos[rig["base_y_qadr"]])
    for _ in range(round(1.2 / 0.04)):
        max_acc = max(max_acc, _step_twist(
            model, data, rig, drive, base_teleop.BodyTwist(0.0, -0.45, 0.0), 0.04))
    reversed_y = float(data.qpos[rig["base_y_qadr"]])
    reversal_ok = positive_y > 0.25 and reversed_y < positive_y - 0.18 and max_acc < QACC_LIMIT

    ok = (not internal_contacts and strafe_ok and yaw_ok and combined_ok and reversal_ok)
    print(f"  Omnidirectional: internal_contacts={len(internal_contacts)} "
          f"strafe=({strafe[0]:+.3f},{strafe[1]:+.3f},{np.degrees(strafe[2]):+.1f}deg) "
          f"yaw=({yaw[0]:+.3f},{yaw[1]:+.3f},{np.degrees(yaw[2]):+.1f}deg) "
          f"combined_dist={np.linalg.norm(combined[:2]):.3f}m/"
          f"{np.degrees(combined[2]):.1f}deg reverse_y={positive_y:.3f}->{reversed_y:.3f}: "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def run_idle_regression(model):
    """주행 키를 전혀 누르지 않은 상태가 흐르지 않는지 검사한다.

    누락된 키프레임 토큰과 수치적으로 불안정한 0 바퀴·바닥 간격 때문에 세 바퀴 중
    두 개가 ``data.contact``에서 사라졌던 두 결함을 직접 회귀 검사한다. 두 결함 모두
    이 정지 유지 시험에서 예상치 못한 이동으로 처음 드러났다.
    """
    data = mujoco.MjData(model)
    _reset_home(model, data)
    rig = _make_rig(model)
    drive = base_teleop.SwerveDrive()

    site_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r")
    site_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l")
    p0_r, p0_l = data.site_xpos[site_r].copy(), data.site_xpos[site_l].copy()

    max_qacc = 0.0
    for _ in range(int(5.0 / 0.04)):
        max_qacc = max(max_qacc, _step(model, data, rig, drive, {}, 0.04))

    drift_r = float(np.linalg.norm(data.site_xpos[site_r] - p0_r))
    drift_l = float(np.linalg.norm(data.site_xpos[site_l] - p0_l))
    base_drift = float(np.linalg.norm(data.qpos[rig["base_x_qadr"]:rig["base_x_qadr"] + 2]))
    print(f"  Idle hold (5s, no drive keys): max|qacc|={max_qacc:.3f} (limit {QACC_LIMIT:.0e}), "
          f"site_r drift={drift_r*1000:.3f}mm site_l drift={drift_l*1000:.3f}mm "
          f"base drift={base_drift*1000:.4f}mm (limit {IDLE_DRIFT_LIMIT*1000:.0f}mm)")
    return (max_qacc < QACC_LIMIT and drift_r < IDLE_DRIFT_LIMIT and drift_l < IDLE_DRIFT_LIMIT
            and base_drift < IDLE_DRIFT_LIMIT)


def run_drive_test(model):
    """후진 키를 3초 누르고 2초 놓아 실제 바퀴 구름으로 이동하는지 검사한다.

    후진은 장애물이 없는 테이블 반대 방향이다. 전진 중 테이블과 만나는 경우는 별도
    ``run_collision_test``에서 다룬다. 베이스가 실제로 움직이는 것뿐 아니라 바퀴 둘레
    속도와 베이스 속도가 작은 오차 안에서 일치해 미끄러지지 않는지도 확인한다. 남은
    가상 액추에이터가 아니라 실제 마찰로 추진되는지 직접 검증한다.
    """
    data = mujoco.MjData(model)
    _reset_home(model, data)
    rig = _make_rig(model)
    drive = base_teleop.SwerveDrive()

    max_qacc = 0.0
    x0 = data.qpos[rig["base_x_qadr"]]
    for _ in range(int(3.0 / 0.04)):
        max_qacc = max(max_qacc, _step(model, data, rig, drive, {"s": True}, 0.04))
    x_driven = data.qpos[rig["base_x_qadr"]]
    base_vx = data.qvel[rig["base_x_dof"]]
    wheel_qvel = data.qvel[rig["drive_dofs"]["left_wheel"]]
    rolling_speed = abs(wheel_qvel) * base_teleop.WHEEL_RADIUS
    slip = abs(rolling_speed - abs(base_vx)) / max(rolling_speed, 1e-6)

    for _ in range(int(2.0 / 0.04)):
        max_qacc = max(max_qacc, _step(model, data, rig, drive, {}, 0.04))
    speed_released = float(np.linalg.norm(data.qvel[rig["base_x_dof"]:rig["base_x_dof"] + 2]))

    distance = x0 - x_driven  # 양수면 명령대로 후진한 것이다.
    print(f"  Drive test: max|qacc|={max_qacc:.3f} (limit {QACC_LIMIT:.0e}), "
          f"distance after 3s='s'={distance*1000:.1f}mm, base speed while driven={abs(base_vx):.3f}m/s, "
          f"wheel rolling speed={rolling_speed:.3f}m/s (slip={slip*100:.1f}%), "
          f"speed 2s after release={speed_released:.4f}m/s")
    ok = (max_qacc < QACC_LIMIT and distance > 0.15 and abs(base_vx) > 0.1
          and slip < 0.15 and speed_released < 0.01)
    return ok


def run_collision_test(model):
    """테이블 방향 전진 키를 6초 눌러 접촉으로 정지하고 관통하지 않는지 검사한다.

    장애물이 없다면 1 m 이상 이동할 시간이지만, 뻗은 팔·손과 테이블의 접촉이 그보다
    훨씬 앞에서 베이스를 막아야 한다.
    """
    data = mujoco.MjData(model)
    _reset_home(model, data)
    rig = _make_rig(model)
    drive = base_teleop.SwerveDrive()

    max_qacc = 0.0
    for _ in range(int(6.0 / 0.04)):
        max_qacc = max(max_qacc, _step(model, data, rig, drive, {"w": True}, 0.04))

    x_final = data.qpos[rig["base_x_qadr"]]
    print(f"  Collision test: max|qacc|={max_qacc:.3f} (limit {QACC_LIMIT:.0e}), "
          f"base_x after 6s driving toward the table={x_final*1000:.1f}mm "
          f"(unobstructed would be ~1000mm+)")
    return max_qacc < QACC_LIMIT and 0.0 < x_final < 1.0


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    print("Part 1: BaseTeleop + SwerveDrive unit tests")
    unit_ok = run_unit_tests()

    print("Part 2a: idle hold regression (no drive keys)")
    idle_ok = run_idle_regression(model)

    print("Part 2b: drive + release (unobstructed direction, checks real rolling/no-slip)")
    drive_ok = run_drive_test(model)

    print("Part 2c: drive into the table (collision should stop it, not tunnel through)")
    collision_ok = run_collision_test(model)

    print("Part 2d: physical omnidirectional/reversal/self-collision regression")
    omni_ok = run_omnidirectional_regression(model)

    ok = unit_ok and idle_ok and drive_ok and collision_ok and omni_ok
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
