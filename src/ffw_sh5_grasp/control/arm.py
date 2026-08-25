"""Phase 3 팔 토크 제어: 소프트웨어 PD와 중력·코리올리 전향 보상.

Phase 3에서 MuJoCo 내장 ``<position>`` 액추에이터가 적분이나 전향 보상 없이 순수
비례 항만 사용한다는 점을 확인했다. 정적 팔 자세를 자체 하중에 맞서 유지하면 약
15~20 mm의 site 오차가 남았다. 가능한 원인을 다음 순서로 시험해 배제했다.

1. 새 ``MjData``에 ``q_grasp``를 직접 쓰고 ``mj_forward``를 한 번 호출한 시험에서는
   site 오차가 0.004 mm였다. IK 목표점과 측정점은 일치한다.
2. 정착 상태에서 관절별 ``actuator_force``와 ``forcerange``를 비교했지만 포화된 관절은
   없었다. 최악의 경우에도 31.7 N·m 중 약 11.5 N·m의 여유가 있었다.
3. 보낸 모든 값이 ``data.ctrl``과 정확히 일치해 ctrl 제한도 원인이 아니었다.

60초 정착 시험에서는 오차가 고정점에 멈추지 않고 수 초의 시정수로 천천히 줄었다.
이는 각 관절을 독립 SISO 시스템으로 가정하고 링크 사이 관성 결합을 무시하는
``<position>``의 관절별 ``dampratio=1``이 결합된 7링크 체인을 실제로 임계 감쇠하지
못한다는 증거다. 느린 모드는 주로 kp 제한 때문이 아니어서 kp를 5배로 올려도 개선이
작았다.

따라서 팔의 ``<position>`` 액추에이터를 순수 토크 입력인 ``<motor>``로 바꾸고 매
물리 스텝마다 다음 표준 로봇 팔 제어 법칙으로 구동한다.

    tau = qfrc_bias[joint]      (현재 상태의 중력·코리올리·원심력을 상쇄하는 전향 보상)
        + kp * (q_des - q)      (위치 피드백)
        - kd * qvel             (결합 시스템에 맞춘 속도 피드백과 능동 감쇠)

정상 상태 오차를 없애는 핵심은 더 큰 비례 이득이 아니라 현재 상태의 bias force를
상쇄하는 항이다. 손은 기존의 힘 제한 ``<position>`` 액추에이터를 유지한다. 접촉 시
토크가 포화되는 유연한 파지 동작은 의도된 것이며 Phase 1/2에서 검증했다. 이 모듈은
강체 팔의 위치 제어만 담당한다.
"""

import mujoco
import numpy as np

from .. import mujoco_utils
from ..config import SETTINGS

DEFAULT_KP = SETTINGS.number("arm_control.proportional_gain", positive=True)
DEFAULT_KD = SETTINGS.number("arm_control.derivative_gain", minimum=0.0)


class ArmTorqueController:
    """팔 관절을 <motor>(순수 토크) 액추에이터로 구동하는 PD + 중력/코리올리
    feedforward 제어기. MuJoCo의 내장 <position> 액추에이터가 정적 하중에서 남기는
    비례오차(모듈 docstring 참고)를 없애기 위해, 매 스텝 직접 토크를 계산해서 쓴다.
    """

    def __init__(self, model, joint_names, kp=DEFAULT_KP, kd=DEFAULT_KD):
        """팔 관절과 motor actuator 주소, 토크 범위, PD 이득을 한 번 찾아 저장한다.

        ``joint_names``의 순서는 목표 관절 벡터 ``q_des``의 순서가 된다. 각 관절에
        연결된 액추에이터가 없으면 잘못된 모델 연결로 보고 ``ValueError``를 낸다.
        """
        self.model = model
        joint_names = tuple(joint_names)
        self.joint_ids = np.array(
            [
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                for name in joint_names
            ],
            dtype=int,
        )
        self.qpos_adrs = np.asarray(model.jnt_qposadr[self.joint_ids], dtype=int)
        self.dof_ids = np.asarray(model.jnt_dofadr[self.joint_ids], dtype=int)
        actuator_ids = []
        # 관절 이름에서 해당 관절을 구동하는 motor 액추에이터 ID를 미리 찾아 저장한다.
        # 매 스텝 다시 찾지 않도록 초기화할 때 한 번만 수행한다.
        for name, jid in zip(joint_names, self.joint_ids):
            aid = mujoco_utils.find_actuator_for_joint(model, jid)
            if aid is None:
                raise ValueError(f"no motor actuator found for joint {name}")
            actuator_ids.append(aid)
        self.actuator_ids = np.asarray(actuator_ids, dtype=int)
        self.ctrl_lower = np.asarray(
            model.actuator_ctrlrange[self.actuator_ids, 0], dtype=float
        )
        self.ctrl_upper = np.asarray(
            model.actuator_ctrlrange[self.actuator_ids, 1], dtype=float
        )
        self.kp = float(kp)
        self.kd = float(kd)

    def apply(self, data, q_des, kp_scale=1.0):
        """현재 스텝의 토크를 계산해 ``data.ctrl``에 기록한다.

        상태 피드백으로 ``data.qpos``, ``data.qvel``, ``data.qfrc_bias``를 읽으며
        ``data.qpos``에는 절대 쓰지 않는다.
        """
        # 매 물리 스텝마다 현재 관절각과 각속도를 읽고 목표각(q_des)과의
        # 차이를 PD로 보정하되, qfrc_bias(중력+코리올리+원심력)를 더해 "지금 이
        # 자세를 버티는 데 필요한 힘"을 미리 상쇄해준다 -- 이게 정적 처짐을 없애는
        # 핵심이고, kp를 올리는 것과는 다른 얘기다.
        q = data.qpos[self.qpos_adrs]
        qd = data.qvel[self.dof_ids]
        qfrc_bias = data.qfrc_bias[self.dof_ids]
        tau = qfrc_bias + self.kp * kp_scale * (np.asarray(q_des) - q) - self.kd * qd
        # 모든 팔 actuator의 범위를 초기화 때 한 번만 모아 두고 일괄 기록한다.
        data.ctrl[self.actuator_ids] = np.clip(tau, self.ctrl_lower, self.ctrl_upper)
        return tau
