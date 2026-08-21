"""Phase 4의 스크립트 기반 파지·들기 동작을 GIF로 만드는 개발 도구.

Phase 테스트 자체는 아니다. ``models/full_scene.xml``에서 접촉력과 접촉점을 표시하고
카메라가 오른손을 따라가게 한다. 대화형 ``application/teleop.py``는 사용자가 직접
슬라이더를 조작하지만, 이 스크립트는 ``test_phase_4.py``와 같은 검증 동작을
headless로 실행해 키보드나 마우스 조작자 없이 데모를 만든다.

사용법: ``python3 tests/record_demo.py [out.gif]``
"""

import pathlib
import sys

import mujoco
import numpy as np
from PIL import Image

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "full_scene.xml"

from ffw_sh5_grasp.control import arm as arm_control  # noqa: E402
from ffw_sh5_grasp.control import grasp  # noqa: E402
from ffw_sh5_grasp.kinematics import JointSpaceKinematics  # noqa: E402
from offline_pose_ik import solve_offline_pose_multistart  # noqa: E402

ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
# ``models/full_scene.xml``의 ``home`` 키프레임과 일치한다.
HOME_Q_R = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])
HOME_Q_L = np.array([0.0, 0.0, 0.0, -1.5707963267948966, 0.0, 0.0, 0.0])

PRE_GRASP_OFFSET = np.array([0.0, 0.0, 0.10])
RAMP_TIME = 1.0
SETTLE_TIME = 1.0
LIFT_HEIGHT = 0.10
LIFT_SPEED = 0.02
POST_LIFT_HOLD = 2.0
APPROACH_SPEED = 0.03

FRAME_EVERY_S = 0.15
GIF_FRAME_MS = 90  # 약 11 fps로 재생한다.


def _read_arm_q(model, data, joint_names):
    """관절 이름 순서에 맞춰 현재 팔 qpos를 NumPy 벡터로 반환한다."""
    return np.array([data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]]
                      for n in joint_names])


class FrameGrabber:
    """MuJoCo offscreen 프레임을 일정 간격으로 모아 GIF로 저장한다."""

    def __init__(self, model):
        """오른손 추적 카메라, 접촉 표시 옵션과 빈 프레임 버퍼를 초기화한다."""
        self.renderer = mujoco.Renderer(model, height=360, width=480)
        self.cam = mujoco.MjvCamera()
        self.cam.lookat[:] = [0.5055, 0.0, 0.85]
        self.cam.distance = 0.55
        self.cam.azimuth = 100
        self.cam.elevation = -18
        self.opt = mujoco.MjvOption()
        self.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        self.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        self.frames = []
        self._next_t = 0.0

    def maybe_capture(self, data):
        """시뮬레이션 시각이 다음 촬영 시각에 도달했을 때 한 프레임을 저장한다."""
        if data.time < self._next_t:
            return
        self._next_t += FRAME_EVERY_S
        self.renderer.update_scene(data, camera=self.cam, scene_option=self.opt)
        self.frames.append(Image.fromarray(self.renderer.render()))

    def save(self, path):
        """누적 프레임을 128색으로 양자화해 반복 재생 GIF로 기록한다."""
        # 평평한 로봇 shading, 격자 바닥과 적은 색으로 구성되어 적응형 palette 양자화를
        # 적용해도 시각적 손실이 거의 없다. PIL 기본 프레임별 full-color GIF보다 용량을
        # 몇 배 줄일 수 있다.
        quantized = [f.quantize(colors=128, method=Image.MEDIANCUT) for f in self.frames]
        quantized[0].save(path, save_all=True, append_images=quantized[1:],
                           duration=GIF_FRAME_MS, loop=0, optimize=False)
        print(f"wrote {len(self.frames)} frames to {path}")


def _move(model, data, ctrl_r, ctrl_l, grabber, q_from, q_to, duration, dt, grasp_frac=None, thumb_frac=None):
    """오른팔 목표를 선형 보간하며 물리 진행·선택적 파지·촬영을 수행한다."""
    n = int(duration / dt)
    for i in range(n):
        frac = i / n
        ctrl_r.apply(data, q_from + frac * (q_to - q_from))
        ctrl_l.apply(data, HOME_Q_L)
        if grasp_frac is not None:
            grasp.apply_grasp(model, data, grasp=grasp_frac, thumb=thumb_frac, side="r")
        mujoco.mj_step(model, data)
        grabber.maybe_capture(data)


def _hold(model, data, ctrl_r, ctrl_l, grabber, q_des, duration, dt, grasp_frac=None, thumb_frac=None):
    """오른팔 목표를 유지하며 물리 진행·선택적 파지·촬영을 수행한다."""
    n = int(duration / dt)
    for _ in range(n):
        ctrl_r.apply(data, q_des)
        ctrl_l.apply(data, HOME_Q_L)
        if grasp_frac is not None:
            grasp.apply_grasp(model, data, grasp=grasp_frac, thumb=thumb_frac, side="r")
        mujoco.mj_step(model, data)
        grabber.maybe_capture(data)


def main():
    """접근·파지·들기 동작을 자동 실행해 문서용 GIF 데모를 생성한다."""
    out_path = sys.argv[1] if len(sys.argv) > 1 else str(REPO_ROOT / "docs" / "assets" / "demo.gif")
    pathlib.Path(out_path).parent.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    solver = JointSpaceKinematics(model, "grasp_target_r", ARM_R)
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
    grabber = FrameGrabber(model)
    rng = np.random.default_rng(0)
    dt = model.opt.timestep

    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    can_qadr = model.jnt_qposadr[can_jid]
    can_pos0 = data.qpos[can_qadr:can_qadr + 3].copy()
    target_quat = np.array([0.5, 0.5, 0.5, 0.5])

    ctx = data.qpos.copy()
    pregrasp_pos = can_pos0 + PRE_GRASP_OFFSET
    q_pregrasp, _, _, ok1 = solve_offline_pose_multistart(
        solver, HOME_Q_R, pregrasp_pos, target_quat, rng, context_qpos=ctx)
    q_grasp, _, _, ok2 = solve_offline_pose_multistart(
        solver, q_pregrasp, can_pos0, target_quat, rng, context_qpos=ctx)
    assert ok1 and ok2, "IK failed to set up demo -- check models/full_scene.xml"

    q_home = _read_arm_q(model, data, ARM_R)
    grabber.maybe_capture(data)

    print("move -> pre-grasp")
    _move(model, data, ctrl_r, ctrl_l, grabber, q_home, q_pregrasp, 3.0, dt, grasp_frac=0.0, thumb_frac=0.0)
    _hold(model, data, ctrl_r, ctrl_l, grabber, q_pregrasp, 0.5, dt, grasp_frac=0.0, thumb_frac=0.0)

    print("approach")
    approach_time = np.linalg.norm(PRE_GRASP_OFFSET) / APPROACH_SPEED
    _move(model, data, ctrl_r, ctrl_l, grabber, q_pregrasp, q_grasp, approach_time, dt, grasp_frac=0.0, thumb_frac=0.0)
    _hold(model, data, ctrl_r, ctrl_l, grabber, q_grasp, 0.5, dt, grasp_frac=0.0, thumb_frac=0.0)

    print("grasp")
    n = int(RAMP_TIME / dt)
    for i in range(n):
        frac = i / n
        ctrl_r.apply(data, q_grasp)
        ctrl_l.apply(data, HOME_Q_L)
        grasp.apply_grasp(model, data, grasp=frac, thumb=frac, side="r")
        mujoco.mj_step(model, data)
        grabber.maybe_capture(data)
    _hold(model, data, ctrl_r, ctrl_l, grabber, q_grasp, SETTLE_TIME, dt, grasp_frac=1.0, thumb_frac=1.0)

    print("lift")
    lift_target_pos = can_pos0 + np.array([0, 0, LIFT_HEIGHT])
    q_lift, _, _, _ = solve_offline_pose_multistart(
        solver, q_grasp, lift_target_pos, target_quat, rng, context_qpos=ctx)
    lift_time = LIFT_HEIGHT / LIFT_SPEED
    _move(model, data, ctrl_r, ctrl_l, grabber, q_grasp, q_lift, lift_time, dt, grasp_frac=1.0, thumb_frac=1.0)
    _hold(model, data, ctrl_r, ctrl_l, grabber, q_lift, POST_LIFT_HOLD, dt, grasp_frac=1.0, thumb_frac=1.0)

    net_lift = data.qpos[can_qadr + 2] - can_pos0[2]
    print(f"net_lift={net_lift*100:.2f}cm grasped={grasp.is_grasped(model, data, side='r')}")
    grabber.save(out_path)


if __name__ == "__main__":
    main()
