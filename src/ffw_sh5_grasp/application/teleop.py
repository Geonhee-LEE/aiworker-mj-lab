"""``models/full_scene.xml``을 실행하는 단일 창 텔레오퍼레이션 앱.

참조 영상의 인터페이스를 3D 화면 위에서 이동 가능한 ImGui 도구 창으로 재현한다.
홈 기준 X/Y/Z/Roll/Pitch/Yaw 말단 자세 슬라이더가 ROS 없는 전신 IK(베이스, 리프트,
양팔), 팔 토크 제어, 파지·엄지 시너지, 관절 모니터와 HUD를 구동한다.

렌더링은 같은 GLFW 창에서 MuJoCo 저수준 API와 ImGui를 함께 사용한다. ImGui OpenGL
렌더러가 호환되는 GLX 문맥을 받도록 GLFW는 X11 백엔드를 사용해야 한다. UI, 제어,
물리와 렌더링 단계는 모두 한 스레드에서 실행된다.

RPY 슬라이더는 각 손의 시작 자세를 기준으로 한 로컬 오프셋을 나타낸다. 전신 자세
목표는 월드 좌표계에 고정되며, 수동 주행 중에는 차체와 함께 이동한다.

**전신 제어와 모바일 베이스**: 말단 목표는 시작 월드 자세에 고정된 홈 기준 UI
값이다. `control/whole_body.py`는 베이스 x/y/yaw, 리프트와 양팔 7자유도를 하나의
경계 제한 가중 미분 IK 문제로 푼다. 베이스 속도는 차체 좌표계 `BodyTwist`로 변환한
뒤 `base.SwerveDrive`로 전달한다. 실제 조향·구동 액추에이터만 명령하므로 베이스의
모든 이동은 바퀴와 지면의 마찰로 발생한다. 키보드 차체 속도는 누르는 동안 우선하며
동일한 스워브 경로를 사용한다. ROS/MoveIt 의존성이나 로봇 qpos 덮어쓰기는 없다.

실행: `python3 src/teleop_app.py`
마우스: 왼쪽 드래그 회전, 오른쪽 드래그 이동, 스크롤 확대·축소
키보드: 위/아래는 베이스 전진·후진, 왼쪽/오른쪽은 베이스 회전, 대괄호는 좌우 이동,
Q/E는 리프트 하강·상승, R은 캔 무작위 재배치, G는 접촉력 표시, V는 충돌 형상과
CBF 표시, C는 카메라 프리셋 전환이다. R/G/V/C 기능은 화면 도구 창의 버튼으로도
실행할 수 있다.

"""

import argparse
import math
from pathlib import Path
import sys
import time

import glfw
# 호환되는 GLX 문맥을 선택하려면 ``glfw.init()``보다 먼저 호출해야 한다.
glfw.init_hint(glfw.PLATFORM, glfw.PLATFORM_X11)

import mujoco
import numpy as np

from ffw_sh5_grasp.control import arm, base  # noqa: E402
from ffw_sh5_grasp.control import grasp  # noqa: E402
from ffw_sh5_grasp.control import whole_body  # noqa: E402
from ffw_sh5_grasp.config import CONFIG_ENV_VAR, SETTINGS  # noqa: E402
from ffw_sh5_grasp.paths import MODEL_PATH  # noqa: E402
from ffw_sh5_grasp.visualization import render, ui  # noqa: E402
from . import targets  # noqa: E402

# 양팔 관절 이름 목록 (IK solver / 토크 제어기에 그대로 넘겨진다).
ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
SIDES = ("r", "l")
ARM_JOINTS = {"r": ARM_R, "l": ARM_L}
WHEELS = base.WHEELS
# ``models/full_scene.xml``의 ``home`` 키프레임과 일치한다. 이 자세는 관련
# ffw-sh5-mujoco 저장소의 휴지 자세와 같으며, 팔꿈치인 4번 관절만 -90도이고
# 나머지 관절은 모두 0도다.
HOME_Q_R = np.asarray(SETTINGS.get("application.home_arm_position_rad"), dtype=float)
HOME_Q_L = HOME_Q_R.copy()
LIFT_RANGE = tuple(float(value) for value in SETTINGS.get("application.lift_range_m"))
VIRTUAL_OBJECT_HOME_POS = np.asarray(
    SETTINGS.get("application.virtual_object_home_position_m"), dtype=float)
HOME_KEYFRAME = SETTINGS.get("application.home_keyframe")
# 패널의 "Joint position monitor"에 진행률 막대로 표시할 관절 전체 목록.
MONITOR_JOINTS = (
    [f"arm_r_joint{i}" for i in range(1, 8)] + [f"arm_l_joint{i}" for i in range(1, 8)]
    + [f"finger_r_joint{i}" for i in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)]
    + [f"finger_l_joint{i}" for i in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)]
    + ["lift_joint", "head_joint1", "head_joint2"]
)

WINDOW_W = SETTINGS.integer("application.window.width", minimum=1)
WINDOW_H = SETTINGS.integer("application.window.height", minimum=1)
LOOP_HZ = SETTINGS.number("application.loop_hz", positive=True)
# 목표 변화율 제한은 원시 슬라이더 값의 급격한 변화와 무관하게 실제 IK 목표가
# 렌더링 한 프레임에서 이동할 수 있는 거리를 제한한다. 25 Hz에서 프레임당 0.03 m는
# 0.75 m/s, 프레임당 8도는 200 deg/s에 해당해 빠르지만 추종 가능한 범위다.
MAX_POS_STEP_PER_FRAME = SETTINGS.number(
    "application.max_position_step_per_frame_m", positive=True)
MAX_RPY_STEP_PER_FRAME_DEG = SETTINGS.number(
    "application.max_rotation_step_per_frame_deg", positive=True)
CAN_RESET_NOISE = SETTINGS.number("application.can_reset_noise_m", minimum=0.0)
LIFT_JOG_SPEED = SETTINGS.number("application.lift_jog_speed_m_s", positive=True)
GRASP_COMMAND_RATE = SETTINGS.number(
    "application.grasp_command_rate_per_s", positive=True)
MANUAL_STOP_LINEAR_SPEED = SETTINGS.number(
    "application.manual_stop_linear_speed_m_s", minimum=0.0)
MANUAL_STOP_ANGULAR_SPEED = SETTINGS.number(
    "application.manual_stop_angular_speed_rad_s", minimum=0.0)
if LIFT_RANGE[0] >= LIFT_RANGE[1]:
    raise ValueError("application.lift_range_m은 [최솟값, 최댓값] 순서여야 합니다.")


def _named_id(model, object_type, name):
    """필수 MuJoCo 객체 이름을 ID로 바꾸고 누락 시 문맥이 있는 오류를 낸다."""
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise ValueError(f"MuJoCo object not found: {name!r}")
    return object_id


def _joint_address(model, name, addresses):
    """관절 이름을 qpos 또는 dof 주소 배열의 정수 인덱스로 변환한다."""
    joint_id = _named_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return int(addresses[joint_id])


def _reset_can_random(model, data, rng):
    """초기 키프레임 리셋 외에 이 파일에서 허용하는 유일한 qpos 쓰기다.

    자유 물체의 생성 자세를 재설정하는 명시적 예외이며, 로봇 자체의 기구학 상태를
    강제로 덮어쓰는 동작은 아니다.
    """
    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    qadr = model.jnt_qposadr[can_jid]
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, HOME_KEYFRAME)
    home_can_pos = model.key_qpos[key_id][qadr:qadr + 3].copy()
    data.qpos[qadr:qadr + 3] = home_can_pos + rng.uniform(
        -CAN_RESET_NOISE, CAN_RESET_NOISE, size=3)
    data.qpos[qadr + 3:qadr + 7] = [1, 0, 0, 0]
    dof = model.jnt_dofadr[can_jid]
    data.qvel[dof:dof + 6] = 0.0


class KeyEdge:
    """키를 누른 순간에만 한 번 참을 반환하는 엣지 입력 검사기."""

    def __init__(self):
        """이전 프레임에 눌려 있던 키 집합을 빈 상태로 초기화한다."""
        self._prev = set()

    def pressed(self, window, key):
        """지정 GLFW 키가 이번 프레임에 새로 눌렸을 때만 ``True``를 반환한다."""
        down = glfw.get_key(window, key) == glfw.PRESS
        was_down = key in self._prev
        if down:
            self._prev.add(key)
        else:
            self._prev.discard(key)
        return down and not was_down


class TeleopApp:
    """MuJoCo 주 창과 분리형 도구 창을 쓰는 텔레옵 앱. `run()`의 메인 루프가 매 프레임 아래 단계를
    순서대로 실행한다: 마우스 카메라 -> 엣지 키(R/G/V/C) -> 연속 키(주행/리프트) ->
    UI 패널 -> 물리 스텝 -> 렌더링. 상태는 전부 인스턴스 속성(self.*)에 있다."""

    def __init__(self):
        """시뮬레이션·렌더링·루프 상태를 순서대로 구성해 실행 가능한 앱을 만든다."""
        self._setup_sim()
        render.setup_render(self, WINDOW_W, WINDOW_H)
        self._setup_loop_state()

    # 초기화

    def _setup_sim(self):
        """모델 로드, 홈 자세 리셋, IK 솔버/토크 제어기, actuator/joint id 조회,
        슬라이더 목표값(targets) 초기화까지 -- 렌더링/윈도우와 무관한 부분 전부."""
        model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        data = mujoco.MjData(model)
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, HOME_KEYFRAME)
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        self.model = model
        self.data = data

        # 전신 IK solver와 팔 토크 제어기를 만든다. 기존 ik.py는 Phase 3/4 독립 회귀용이다.
        self.arm_controllers = {
            side: arm.ArmTorqueController(model, joint_names)
            for side, joint_names in ARM_JOINTS.items()
        }
        self.whole_body_enabled = True
        self.whole_body_solver = whole_body.WholeBodyIK(
            model,
            {"r": "grasp_target_r", "l": "grasp_target_l"},
            ARM_JOINTS,
        )
        # FK(관절각 직접 제어) 모드에서 패널 슬라이더의 최소/최대값으로 쓸, 각 팔
        # 관절 범위를 도 단위로 미리 계산한다.
        self.arm_joint_ranges_deg = {
            side: [
                tuple(
                    math.degrees(value)
                    for value in model.jnt_range[
                        _named_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                    ]
                )
                for name in joints
            ]
            for side, joints in ARM_JOINTS.items()
        }
        self.lift_aid = _named_id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "lift_joint"
        )
        self.base_x_qadr = _joint_address(model, "base_x", model.jnt_qposadr)
        self.base_y_qadr = _joint_address(model, "base_y", model.jnt_qposadr)
        self.base_yaw_qadr = _joint_address(model, "base_yaw", model.jnt_qposadr)
        self.base_x_dof = _joint_address(model, "base_x", model.jnt_dofadr)
        self.base_y_dof = _joint_address(model, "base_y", model.jnt_dofadr)
        self.base_yaw_dof = _joint_address(model, "base_yaw", model.jnt_dofadr)
        # 바퀴마다 실제 조향 위치 액추에이터와 구동 속도 액추에이터를 사용한다.
        # 자세한 변환은 ``control/base.py``의 ``SwerveDrive``가 담당한다. 베이스 이동은
        # base_x/base_y/base_yaw 직접 구동이 아니라 바퀴와 지면의 마찰 결과다. 해당
        # 관절은 base_link가 넘어지지 않게 남아 있지만 직접 구동되지는 않는다.
        self.wheel_steer_aids = {
            wheel: _named_id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{wheel}_steer"
            )
            for wheel in WHEELS
        }
        self.wheel_drive_aids = {
            wheel: _named_id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{wheel}_drive"
            )
            for wheel in WHEELS
        }
        self.wheel_steer_qadrs = {
            wheel: _joint_address(
                model, f"{wheel}_steer_joint", model.jnt_qposadr
            )
            for wheel in WHEELS
        }
        self.wheel_drive_dofs = {
            wheel: _joint_address(
                model, f"{wheel}_drive_joint", model.jnt_dofadr
            )
            for wheel in WHEELS
        }
        # 모바일 베이스: SwerveDrive가 키 입력을 바퀴별 조향각+구동속도로 변환.
        self.base_drive = base.SwerveDrive()
        monitor_joint_ids = {
            name: _named_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in MONITOR_JOINTS
        }
        self.monitor_qposadr = {
            name: int(model.jnt_qposadr[joint_id])
            for name, joint_id in monitor_joint_ids.items()
        }
        self.monitor_ranges = {
            name: model.jnt_range[joint_id]
            for name, joint_id in monitor_joint_ids.items()
        }
        self.rng = np.random.default_rng()

        self.ik_target_mocap_ids = {
            side: model.body_mocapid[
                _named_id(
                    model, mujoco.mjtObj.mjOBJ_BODY, f"ik_target_{side}"
                )
            ]
            for side in SIDES
        }
        self.virtual_object_mocap_id = model.body_mocapid[
            _named_id(
                model, mujoco.mjtObj.mjOBJ_BODY, "virtual_object_marker"
            )
        ]
        self.virtual_object_marker_geom_id = _named_id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "virtual_object_marker_geom")
        self.virtual_object_marker_site_id = _named_id(
            model, mujoco.mjtObj.mjOBJ_SITE, "virtual_object_marker_site")
        self.virtual_object_marker_rgba = {
            "geom": model.geom_rgba[self.virtual_object_marker_geom_id].copy(),
            "site": model.site_rgba[self.virtual_object_marker_site_id].copy(),
        }
        self.can_jid = _named_id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "can_free"
        )
        self.can_geom_id = _named_id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "can_geom"
        )
        self._disable_legacy_box_asset()
        # 각 손의 XYZ/RPY 슬라이더가 기준으로 삼는 자세다.
        targets.set_home_references(self)

        # 슬라이더가 직접 쓰는 "목표값" 딕셔너리 -- 물리 루프는 이 값만 읽고, 이 값을
        # GUI 슬라이더만 이 값을 쓴다. 데이터는 한 방향으로만 흐른다.
        self.targets = {
            **{
                f"{field}_{side}": [0.0, 0.0, 0.0]
                for side in SIDES
                for field in ("pos", "rpy")
            },
            **{
                f"{field}_{side}": 0.0
                for side in SIDES
                for field in ("grasp", "thumb")
            },
            "virtual_object_pos": VIRTUAL_OBJECT_HOME_POS.tolist(),
            "virtual_object_rpy": [0.0, 0.0, 0.0],
            "lift": float(
                data.qpos[_joint_address(model, "lift_joint", model.jnt_qposadr)]
            ),
        }
        # 입력하거나 드래그한 목표가 급변하지 않도록 전신 IK는 평활화된 사본을 추종한다.
        self.smoothed_pos = {
            side: np.array(self.targets[f"pos_{side}"]) for side in SIDES
        }
        self.smoothed_rpy = {
            side: np.array(self.targets[f"rpy_{side}"]) for side in SIDES
        }
        self.lift_cmd = self.targets["lift"]
        self.whole_body_base_twist = base.BodyTwist()
        self.commanded_base_twist = base.BodyTwist()
        self._manual_override_active = False
        self._manual_reference_base_pose = np.array([
            data.qpos[self.base_x_qadr], data.qpos[self.base_y_qadr],
            data.qpos[self.base_yaw_qadr],
        ], dtype=float)
        targets.sync_ik_mocaps_from_targets(self)
        self.contact_viz = False
        self.collision_viz = False
        self.collision_active_pairs = ()
        self.collision_min_distance = math.inf
        self.collision_constraint_violation = 0.0
        self.camera_preset = 0
        self.grab_state = dict.fromkeys(SIDES)
        self.cyclo_controller = "movel"
        self.cyclo_move_time = SETTINGS.number(
            "application.cyclo_move_time_s", positive=True)
        self.cyclo_grasp_captured = False
        self.cyclo_capture_offsets = None
        self.cyclo_status = "ready"
        # ``ui.draw_panel``은 순환 import를 피하려고 상수를 import하지 않고 ``app``에서
        # 아래 두 값을 직접 읽는다.
        self.lift_range = LIFT_RANGE
        self.reset_can()

    def _setup_loop_state(self):
        """메인 루프에서만 쓰는 상태(IK 웜스타트 값, 타이밍, 입력 헬퍼)."""
        self.q_des = {"r": HOME_Q_R.copy(), "l": HOME_Q_L.copy()}
        # 손별 제어 모드: "ik"(EE 포즈 슬라이더 -> whole-body solver) 또는
        # "fk"(관절각 슬라이더를 그대로 토크 제어기 목표로 사용, IK 자체를 건너뜀).
        # 리프트를 움직이는 동안 IK가 매 프레임에서만 어깨 높이를 다시 읽어들여서
        # 생기는 출렁임(리프트가 프레임 사이에도 계속 움직이는데 IK는 프레임당
        # 한 번만 풀림)을 피하고 싶을 때 FK로 전환해 관절각을 고정해두면, 팔 전체가
        # 리프트에 강체로 붙어 그대로 오르내리기만 해서 흔들림이 아예 없다.
        self.arm_mode = {"r": "ik", "l": "ik"}
        self.fk_q_deg = {
            side: np.degrees(q_des).tolist()
            for side, q_des in self.q_des.items()
        }
        self.frame_dt = 1.0 / LOOP_HZ
        self.steps_per_frame = max(1, round(self.frame_dt / self.model.opt.timestep))
        self.freq_ema = LOOP_HZ
        self.wall_start = time.perf_counter()
        self.last_mouse = list(glfw.get_cursor_pos(self.window))
        self.keys = KeyEdge()
        self.ik_err_mm = {"l": 0.0, "r": 0.0}
        self.gizmo_mouse_active = False

    # R/G/V/C 동작 -- 키보드(_handle_edge_keys)와 패널 버튼
    # (ui.draw_panel) 양쪽에서 똑같이 호출하는 공용 메서드.

    def reset_can(self):
        """캔을 홈 주변의 작은 무작위 위치와 단위 자세로 재배치한다."""
        _reset_can_random(self.model, self.data, self.rng)

    def reset_active_object(self):
        """캔과 파지·가상 물체 상태를 함께 초기화해 새 작업을 시작할 준비를 한다."""
        self.reset_can()
        self.grab_state = {"r": None, "l": None}
        self.cyclo_grasp_captured = False
        self.cyclo_capture_offsets = None
        self.whole_body_solver.set_rigid_grasp(self.data, False)
        self.cyclo_controller = "movel"
        self.cyclo_status = "ready"

    def _disable_legacy_box_asset(self):
        """캔 전용 흐름을 실행하는 동안 XML에 남은 이전 상자 물체를 비활성화한다."""
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "box_free")
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
        if jid == -1 or gid == -1:
            return
        qadr = self.model.jnt_qposadr[jid]
        dof = self.model.jnt_dofadr[jid]
        self.data.qpos[qadr:qadr + 3] = [2.0, 2.0, 0.1]
        self.data.qpos[qadr + 3:qadr + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[dof:dof + 6] = 0.0
        self.model.geom_contype[gid] = 0
        self.model.geom_conaffinity[gid] = 0
        self.model.geom_rgba[gid][3] = 0.0

    def cycle_camera(self):
        """두 개의 사전 정의 카메라 프리셋을 번갈아 선택하고 렌더 카메라에 적용한다."""
        self.camera_preset = 1 - self.camera_preset
        render.set_camera_preset(self.cam, self.camera_preset)

    def toggle_collision_visualization(self):
        """충돌 거리와 최근접점을 나타내는 진단 오버레이 표시 여부를 전환한다."""
        self.collision_viz = not self.collision_viz

    def set_arm_mode(self, side, mode):
        """이 손을 "ik"(EE 포즈 슬라이더) <-> "fk"(관절각 슬라이더)로 전환한다.
        전환 순간 팔이 튀지 않도록 방향에 따라 다르게 동기화한다:

        - ik -> fk: 지금 토크 제어기가 추종 중이던 관절각(q_des)을 그대로 FK
          슬라이더 값으로 복사 -- 전환 직후 첫 스텝의 목표 관절각이 전환 직전과
          정확히 같아서 포즈 점프가 없다.
        - fk -> ik: 반대로 EE 포즈 슬라이더 쪽이 전환 전 값(어쩌면 한참 전에 마지막
          으로 IK 모드였을 때의 낡은 목표)을 그대로 들고 있으므로, 그걸 쓰는 대신
          "지금 실제 site가 있는 월드 포즈"를 홈 기준 XYZ offset/RPY로 역산해 targets/
          smoothed_pos/smoothed_rpy에 다시 채워 넣는다 -- 그래야 IK가 방금 있던
          자리에서부터 이어서 풀리지, 옛 목표로 갑자기 끌려가지 않는다.
        """
        if mode == self.arm_mode[side]:
            return
        q_des = self.q_des[side]

        if mode == "fk":
            self.fk_q_deg[side] = [math.degrees(v) for v in q_des]
        else:
            state = self.whole_body_solver.site_state(self.data, side)
            target_pos = targets.world_to_target_pos(self, side, state.position)
            rpy_deg = targets.world_quat_to_target_rpy(
                self, side, state.quaternion)

            self.targets[f"pos_{side}"] = target_pos
            self.targets[f"rpy_{side}"] = rpy_deg
            self.smoothed_pos[side] = np.array(target_pos)
            self.smoothed_rpy[side] = np.array(rpy_deg)

        self.arm_mode[side] = mode

    def set_whole_body_enabled(self, enabled):
        """월드 목표를 움직이지 않고 전신 IK와 팔 전용 IK를 전환한다.

        두 모드는 의도적으로 서로 다른 UI 좌표계를 사용한다. 전신 목표는 시작 월드
        기준에 머물고, 팔 전용 목표는 현재 차체를 따라간다. 따라서 전환할 때 수치 목표를
        새 좌표계로 다시 표현해야 한다. 또한 캐시된 베이스 명령을 모두 지워 OFF가 가중치
        변경 지연이 아니라 즉시 정지 요청으로 동작하게 한다.
        """
        enabled = bool(enabled)
        if enabled == self.whole_body_enabled:
            return

        hand_world_poses = {
            side: tuple(
                value.copy() for value in targets.target_world_pose(self, side))
            for side in SIDES
        }
        virtual_world_pose = tuple(
            value.copy() for value in targets.virtual_object_world_pose(self))
        self.whole_body_enabled = enabled

        virtual_pos, virtual_quat = virtual_world_pose
        if enabled:
            self.targets["virtual_object_pos"] = targets.world_to_anchor_local_pos(
                self, virtual_pos).tolist()
        else:
            self.targets["virtual_object_pos"] = targets.world_to_base_pos(
                self, virtual_pos).tolist()
        self.targets["virtual_object_rpy"] = list(
            targets.world_quat_to_virtual_rpy(self, virtual_quat))

        if self.cyclo_grasp_captured:
            # 양손 MoveL 모드에서는 캡처된 가상 물체가 최종 목표의 기준이다.
            self.apply_virtual_object_target()
        else:
            for side, (world_pos, world_quat) in hand_world_poses.items():
                self.targets[f"pos_{side}"] = targets.world_to_target_pos(
                    self, side, world_pos)
                self.targets[f"rpy_{side}"] = list(
                    targets.world_quat_to_target_rpy(
                        self, side, world_quat))

        for side in SIDES:
            self.smoothed_pos[side] = np.asarray(
                self.targets[f"pos_{side}"], dtype=float).copy()
            self.smoothed_rpy[side] = np.asarray(
                self.targets[f"rpy_{side}"], dtype=float).copy()

        rebased_targets = {
            side: targets.target_world_pose(self, side) for side in SIDES}
        self.whole_body_solver.rebase(self.data, rebased_targets)
        self.whole_body_base_twist = base.BodyTwist()
        self.commanded_base_twist = base.BodyTwist()
        targets.sync_ik_mocaps_from_targets(self)

    def toggle_whole_body_control(self):
        """현재 상태의 반대 값으로 전신 IK 활성 여부를 안전하게 전환한다."""
        self.set_whole_body_enabled(not self.whole_body_enabled)

    def capture_grasp(self):
        """Cyclo 방식의 ``/capture_grasp true`` 동작을 수행한다.

        가상 물체 마커를 기준으로 양손 목표 자세를 기록한다. 이후에는
        ``virtual_object_pos/rpy``가 명령 원점이 되고, 캡처한 오프셋으로 양손 MoveL
        목표를 계산한다.
        """
        targets.capture_grasp(self)
        self.whole_body_solver.set_rigid_grasp(self.data, True)

    def release_grasp(self):
        """Cyclo 방식의 ``/capture_grasp false``로 독립적인 양손 MoveL 목표로 돌아간다."""
        targets.release_grasp(self)
        self.whole_body_solver.set_rigid_grasp(self.data, False)

    def apply_virtual_object_target(self):
        """가상 물체 목표를 캡처된 상대 변환에 따라 양손 목표로 반영한다."""
        targets.apply_virtual_object_target(self)

    # 메인 루프

    def run(self):
        """창이 닫힐 때까지 입력·제어·물리·렌더링 프레임 루프를 실행한다.

        종료 조건을 만나면 렌더러, ImGui와 GLFW 자원을 ``render.shutdown``으로
        정리한다. 이 메서드는 앱의 최상위 blocking 실행 진입점이다.
        """
        # 매 프레임 (1) 입력 처리 (2) IK 풀기 (3) 물리 스텝 (4) 렌더링을 전부 한
        # 스레드/한 루프 안에서 순서대로 실행한다.
        while not glfw.window_should_close(self.window):
            t0 = time.perf_counter()
            io = render.begin_frame(self)

            render.handle_camera_mouse(self, io)
            self._handle_edge_keys(io)
            drive_keys = self._read_drive_and_lift_keys(io)
            ui.draw_panel(self)
            self._step_physics(drive_keys)
            render.render_scene(self)
            render.end_frame(self, t0)

        render.shutdown(self)

    def _handle_edge_keys(self, io):
        """눌렀다 뗄 때 한 번만 반응하는 R/G/V/C 유틸리티 키."""
        if io.want_capture_keyboard:
            return
        if self.keys.pressed(self.window, glfw.KEY_R):
            self.reset_active_object()
        if self.keys.pressed(self.window, glfw.KEY_G):
            self.contact_viz = not self.contact_viz
        if self.keys.pressed(self.window, glfw.KEY_V):
            self.toggle_collision_visualization()
        if self.keys.pressed(self.window, glfw.KEY_C):
            self.cycle_camera()

    def _read_drive_and_lift_keys(self, io):
        """주행과 리프트 키의 현재 눌림 상태를 매 프레임 읽는다.

        R/G/V/C와 달리 주행 키는 누르는 동안 계속 반응해야 하므로 엣지 입력이 아니라
        레벨 입력으로 처리한다. 위/아래 방향키는 전진·후진, 왼쪽/오른쪽 방향키는 회전,
        대괄호 키는 좌우 이동에 사용한다. WASD는 다른 MuJoCo 도구에서 익숙한 단축키와
        충돌하므로 사용하지 않는다. 좌우 이동도 회전 키나 보조키와 겹치지 않도록 별도
        대괄호 키에 배치했다.
        """
        drive_keys = {"w": False, "a": False, "s": False, "d": False, "left": False, "right": False}
        lift_dir = 0.0
        if not io.want_capture_keyboard:
            drive_keys["w"] = glfw.get_key(self.window, glfw.KEY_UP) == glfw.PRESS
            drive_keys["s"] = glfw.get_key(self.window, glfw.KEY_DOWN) == glfw.PRESS
            drive_keys["left"] = glfw.get_key(self.window, glfw.KEY_LEFT) == glfw.PRESS
            drive_keys["right"] = glfw.get_key(self.window, glfw.KEY_RIGHT) == glfw.PRESS
            drive_keys["a"] = glfw.get_key(self.window, glfw.KEY_LEFT_BRACKET) == glfw.PRESS
            drive_keys["d"] = glfw.get_key(self.window, glfw.KEY_RIGHT_BRACKET) == glfw.PRESS
            if glfw.get_key(self.window, glfw.KEY_E) == glfw.PRESS:
                lift_dir += 1.0
            if glfw.get_key(self.window, glfw.KEY_Q) == glfw.PRESS:
                lift_dir -= 1.0
        if lift_dir != 0.0:
            self.targets["lift"] = float(np.clip(
                self.targets["lift"] + lift_dir * LIFT_JOG_SPEED * self.frame_dt,
                LIFT_RANGE[0], LIFT_RANGE[1]))
        return drive_keys

    def _read_base_feedback(self):
        """수동 주행과 전신 제어 전환에 사용할 바퀴·차체 피드백을 읽는다."""
        data = self.data
        steering_positions = {
            wheel: float(data.qpos[address])
            for wheel, address in self.wheel_steer_qadrs.items()
        }
        wheel_velocities = {
            wheel: float(data.qvel[address])
            for wheel, address in self.wheel_drive_dofs.items()
        }
        yaw = float(data.qpos[self.base_yaw_qadr])
        cosine, sine = math.cos(yaw), math.sin(yaw)
        vx_world = float(data.qvel[self.base_x_dof])
        vy_world = float(data.qvel[self.base_y_dof])
        body_twist = base.BodyTwist(
            cosine * vx_world + sine * vy_world,
            -sine * vx_world + cosine * vy_world,
            float(data.qvel[self.base_yaw_dof]),
        )
        base_pose = np.array(
            [
                data.qpos[self.base_x_qadr],
                data.qpos[self.base_y_qadr],
                data.qpos[self.base_yaw_qadr],
            ],
            dtype=float,
        )
        return steering_positions, wheel_velocities, body_twist, base_pose

    def _update_grasp_targets(self):
        """원터치 열기·닫기 명령을 변화율 제한해 손 슬라이더 값으로 반영한다."""
        for side in SIDES:
            state = self.grab_state[side]
            if state is None:
                continue
            desired = 1.0 if state else 0.0
            for name in (f"grasp_{side}", f"thumb_{side}"):
                delta = np.clip(
                    desired - self.targets[name],
                    -GRASP_COMMAND_RATE * self.frame_dt,
                    GRASP_COMMAND_RATE * self.frame_dt,
                )
                self.targets[name] += float(delta)

    def _smooth_hand_targets(self):
        """슬라이더 목표의 이동 속도를 IK와 제어기가 추종 가능한 범위로 제한한다."""
        for side in SIDES:
            raw_position = np.asarray(self.targets[f"pos_{side}"], dtype=float)
            position_delta = np.clip(
                raw_position - self.smoothed_pos[side],
                -MAX_POS_STEP_PER_FRAME,
                MAX_POS_STEP_PER_FRAME,
            )
            self.smoothed_pos[side] += position_delta

            raw_rpy = np.asarray(self.targets[f"rpy_{side}"], dtype=float)
            rpy_delta = np.clip(
                raw_rpy - self.smoothed_rpy[side],
                -MAX_RPY_STEP_PER_FRAME_DEG,
                MAX_RPY_STEP_PER_FRAME_DEG,
            )
            self.smoothed_rpy[side] += rpy_delta

    def _smoothed_target_poses(self):
        """변화율이 제한된 UI 목표에서 월드 좌표계 자세를 만든다."""
        return {
            side: (
                targets.target_pos_to_world_pos(
                    self, side, self.smoothed_pos[side]
                ),
                targets.target_rpy_to_world_quat(
                    self, side, self.smoothed_rpy[side]
                ),
            )
            for side in SIDES
        }

    def _step_actuators(self, wheel_commands):
        """렌더링 한 프레임 동안 현재 명령 묶음을 모든 물리 서브스텝에 적용한다."""
        data = self.data
        for _ in range(self.steps_per_frame):
            for side in SIDES:
                self.arm_controllers[side].apply(data, self.q_des[side])
            data.ctrl[self.lift_aid] = self.lift_cmd
            for wheel, (steer_angle, drive_speed) in wheel_commands.items():
                data.ctrl[self.wheel_steer_aids[wheel]] = steer_angle
                data.ctrl[self.wheel_drive_aids[wheel]] = drive_speed
            for side in SIDES:
                grasp.apply_grasp(
                    self.model,
                    data,
                    grasp=self.targets[f"grasp_{side}"],
                    thumb=self.targets[f"thumb_{side}"],
                    side=side,
                )
            mujoco.mj_step(self.model, data)

    def _step_physics(self, drive_keys):
        """실제 물리 반영: target rate-limit -> world-fixed pose -> whole-body solve ->
        팔 torque/lift position/swerve/grasp actuator ctrl -> ``mj_step``. Solver는 live
        qpos를 읽기만 하며, robot qpos를 직접 쓰는 kinematic override는 없다."""
        data = self.data
        (
            steering_positions,
            wheel_velocities,
            measured_body_twist,
            current_base_pose,
        ) = self._read_base_feedback()
        # 수동 주행에서 WBIK로 전환할 때 차체 피드백으로 정지를 판정한다. 제동 중의
        # 일시적인 타이어 미끄러짐 때문에 바퀴 오도메트리보다 모델에서 직접 읽는 평면
        # 베이스 속도가 더 신뢰할 수 있는 정지 지표다.
        manual_twist = self.base_drive.base.update_body(
            drive_keys, self.frame_dt, measured_body_twist)
        manual_keys_active = any(drive_keys.values())
        measured_motion_active = (
            math.hypot(measured_body_twist.vx, measured_body_twist.vy)
            > MANUAL_STOP_LINEAR_SPEED
            or abs(measured_body_twist.wz) > MANUAL_STOP_ANGULAR_SPEED)
        if manual_keys_active and not self._manual_override_active:
            # 수동 입력 시작 전의 WBIK 이동을 수동 이동으로 오인하지 않도록 시작점을 잡는다.
            self._manual_reference_base_pose = current_base_pose.copy()
        carry_manual_targets = manual_keys_active or self._manual_override_active
        if carry_manual_targets:
            # 입력을 놓은 직후의 물리 제동 이동까지 목표 운반에 포함한다.
            targets.carry_world_targets_with_base(
                self, self._manual_reference_base_pose, current_base_pose)
        self._manual_reference_base_pose = current_base_pose.copy()

        if self.cyclo_grasp_captured:
            self.apply_virtual_object_target()
        self._update_grasp_targets()
        self._smooth_hand_targets()

        # 전신 IK 목표는 월드 좌표계에 고정되어야 한다. 매 프레임 현재 베이스 자세에서
        # 목표를 다시 만들면 베이스와 목표가 같은 만큼 움직여 작업 오차를 줄일 수 없다.
        target_poses = self._smoothed_target_poses()

        if carry_manual_targets:
            # 전환 시 새 차체 자세를 공통 이동의 영점으로 다시 정의해야 한다. 그렇지 않으면
            # 시작 기준 때문에 WBIK가 관성처럼 느껴지는 역방향 명령을 낸다.
            self.whole_body_solver.rebase(data, target_poses)

        active_sides = tuple(side for side in SIDES if self.arm_mode[side] == "ik")
        whole_body_cmd = self.whole_body_solver.solve(
            data, target_poses, self.frame_dt,
            active_sides=active_sides,
            arm_nominal={"r": HOME_Q_R, "l": HOME_Q_L},
            lift_nominal=self.targets["lift"],
            rigid_grasp=(self.cyclo_controller == "bimanual_movel"
                         and self.cyclo_grasp_captured),
            whole_body_enabled=self.whole_body_enabled,
        )
        self.whole_body_base_twist = whole_body_cmd.base_twist
        self.collision_active_pairs = whole_body_cmd.active_collision_pairs
        self.collision_min_distance = whole_body_cmd.minimum_collision_distance
        self.collision_constraint_violation = whole_body_cmd.collision_constraint_violation
        self.lift_cmd = (whole_body_cmd.lift_position if self.whole_body_enabled
                         else self.targets["lift"])
        for side in SIDES:
            if side in whole_body_cmd.arm_positions:
                self.q_des[side] = whole_body_cmd.arm_positions[side]
                self.ik_err_mm[side] = whole_body_cmd.position_errors[side] * 1000.0
            else:
                self.q_des[side] = np.radians(self.fk_q_deg[side])

        # 명시적인 키보드 이동이 우선한다. 키를 놓으면 차체 피드백이 정지를 확인할 때까지
        # 영 속도를 명령하고, 그 뒤에만 같은 스워브 경로로 전신 IK를 재개한다.
        if manual_keys_active:
            self.commanded_base_twist = manual_twist
        elif self._manual_override_active:
            self.commanded_base_twist = base.BodyTwist()
        elif self.whole_body_enabled:
            self.commanded_base_twist = self.whole_body_base_twist
        else:
            self.commanded_base_twist = base.BodyTwist()
        previous_manual_override = self._manual_override_active
        wheel_cmds = self.base_drive.update_twist(
            self.commanded_base_twist, self.frame_dt,
            steering_positions, wheel_velocities)
        self._manual_override_active = bool(
            manual_keys_active
            or (previous_manual_override
                and measured_motion_active))
        if previous_manual_override and not self._manual_override_active:
            self.base_drive.base.reset_motion()

        self._step_actuators(wheel_cmds)

def _parse_args(argv):
    """텔레옵 CLI 인자를 파싱하고 설정 파일이 import 전에 적용됐는지 확인한다."""
    parser = argparse.ArgumentParser(description="FFW-SH5 teleop app")
    parser.add_argument(
        "--config", metavar="YAML",
        help=(f"설정 파일 경로. 실행 전에 {CONFIG_ENV_VAR} 환경 변수로 적용되며 "
              "src/teleop_app.py 진입점을 사용할 때 지원됩니다."))
    args = parser.parse_args(argv)
    if args.config:
        requested = str(Path(args.config).expanduser().resolve())
        if requested != str(SETTINGS.path.resolve()):
            raise RuntimeError(
                "--config는 모듈 import 전에 적용해야 합니다. "
                "python3 src/teleop_app.py --config <파일>로 실행하세요.")


def main(argv=None):
    """명령행 인자를 검사한 뒤 :class:`TeleopApp`을 생성하고 메인 루프를 시작한다."""
    _parse_args(sys.argv[1:] if argv is None else argv)
    TeleopApp().run()


if __name__ == "__main__":
    main()
