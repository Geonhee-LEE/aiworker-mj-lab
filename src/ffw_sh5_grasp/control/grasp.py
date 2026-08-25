"""손 시너지 명령과 접촉력 기반 파지 판정.

``grasp``는 검지와 중지를 굽히고 ``thumb``은 엄지 굽힘과 yaw를 제어한다. 약지와
새끼는 모양을 위한 작은 굽힘만 받는다. 양손은 같은 매핑을 공유하며, 대칭인 왼손
엄지 범위는 보간 방향을 명시적으로 지정한다.

조정된 열린 비율은 시작부터 접촉하지 않으면서 손가락을 물체 가까이에 둔다. 엄지
yaw는 접근 중 충돌에 안전한 각도를 유지하고 엄지가 닫힐 때만 손바닥 쪽으로 돈다.
파지는 물체 자세가 아니라 MuJoCo 접촉력으로 판정한다.
"""

import mujoco
import numpy as np

from .. import mujoco_utils
from ..config import SETTINGS

SIDES = ("l", "r")

# 검지/중지 각 손가락의 pip/dip/tip 관절 이름 (좌/우 손 각각). grasp 스칼라 하나가
# 이 여섯 관절 전부의 목표 각도로 동시에 퍼진다.
FINGER_CURL_JOINTS = {
    "l": {
        "index": ("finger_l_joint6", "finger_l_joint7", "finger_l_joint8"),
        "middle": ("finger_l_joint10", "finger_l_joint11", "finger_l_joint12"),
    },
    "r": {
        "index": ("finger_r_joint6", "finger_r_joint7", "finger_r_joint8"),
        "middle": ("finger_r_joint10", "finger_r_joint11", "finger_r_joint12"),
    },
}
# 엄지 mcp_pitch/ip 관절 (thumb 스칼라가 매핑되는 대상).
THUMB_CURL_JOINTS = {
    "l": ("finger_l_joint3", "finger_l_joint4"),
    "r": ("finger_r_joint3", "finger_r_joint4"),
}
# 오른손 엄지는 관절 범위 하한에서 열리고 대칭인 왼손 엄지는 상한에서 열린다.
# 부호에 의존하지 않도록 보간 방향을 명시적으로 저장한다.
THUMB_CURL_OPEN_AT_HI = {"l": True, "r": False}
# 약지/새끼의 pip/dip/tip (mcp는 range=0으로 잠겨 있어 여기 없음) -- 실제 grasp에는
# 참여하지 않고, grasp 스칼라에 비례해 보기 좋으라고만 살짝 굽힌다(아래 apply_grasp
# 참고).
RING_PINKY_CURL_JOINTS = {
    "l": (
        "finger_l_joint14",
        "finger_l_joint15",
        "finger_l_joint16",
        "finger_l_joint18",
        "finger_l_joint19",
        "finger_l_joint20",
    ),
    "r": (
        "finger_r_joint14",
        "finger_r_joint15",
        "finger_r_joint16",
        "finger_r_joint18",
        "finger_r_joint19",
        "finger_r_joint20",
    ),
}
# 엄지 CMC(벌림) 관절은 grasp/thumb 스칼라와 무관하게 항상 이 고정값으로 유지된다 --
# Phase 2에서 FK 그리드 서치로 찾은, 검지·중지 수렴 지점을 엄지가 마주보게 하는 각도.
# CMC만 여기 있다 -- MCP yaw는 더 이상 고정값이 아니라 thumb 스칼라로 램프된다
# (THUMB_YAW_REST/THUMB_YAW_CURL, 바로 아래 참고).
THUMB_PRESHAPE = {
    "l": {"finger_l_joint1": SETTINGS.number("grasp.thumb_preshape_rad.left")},
    "r": {"finger_r_joint1": SETTINGS.number("grasp.thumb_preshape_rad.right")},
}
# 접근 중 MCP yaw는 충돌에 안전한 각도를 유지하고 엄지가 닫히면서 손바닥 쪽으로
# 회전한다. 왼손 값은 조정된 오른손 값을 대칭으로 반영한다.
THUMB_YAW_REST = {
    "l": SETTINGS.number("grasp.thumb_yaw_rest_rad.left"),
    "r": SETTINGS.number("grasp.thumb_yaw_rest_rad.right"),
}
THUMB_YAW_CURL = {
    "l": SETTINGS.number("grasp.thumb_yaw_curl_rad.left"),
    "r": SETTINGS.number("grasp.thumb_yaw_curl_rad.right"),
}

# grasp/thumb=0일 때도 관절 range 전체(lo)까지 펴지 않고 이만큼 남겨둔다 --
# "접촉 직전" 자세를 만들어 자유낙하하는 캔을 놓치지 않기 위함(모듈 docstring 참고).
FINGER_OPEN_FRAC = SETTINGS.number("grasp.finger_open_fraction", minimum=0.0)
THUMB_OPEN_FRAC = SETTINGS.number("grasp.thumb_open_fraction", minimum=0.0)
# 약지/새끼 pip/dip/tip이 grasp=1.0일 때 굽는 최대 비율(자기 range의 35%까지만) --
# pick 성공률을 0.20~0.60으로 스윕해서 찾은 안전한 상한(0.40/0.45 사이가 절벽).
RING_PINKY_MAX_FRAC = SETTINGS.number("grasp.ring_pinky_max_fraction", minimum=0.0)
DEFAULT_MIN_FINGERS = SETTINGS.integer("grasp.detection.minimum_fingers", minimum=1)
DEFAULT_MIN_TOTAL_FORCE = SETTINGS.number(
    "grasp.detection.minimum_total_force_n", minimum=0.0
)
DEFAULT_REQUIRE_THUMB = SETTINGS.get("grasp.detection.require_thumb")
for _name, _fraction in (
    ("finger_open_fraction", FINGER_OPEN_FRAC),
    ("thumb_open_fraction", THUMB_OPEN_FRAC),
    ("ring_pinky_max_fraction", RING_PINKY_MAX_FRAC),
):
    if _fraction > 1.0:
        raise ValueError(f"grasp.{_name}는 1 이하여야 합니다.")

# 접촉력 판정(get_finger_can_contacts)에서 "이 body가 어느 손가락 그룹 소속인지"
# 조회하는 데 쓰는 역방향 매핑의 원본 데이터.
FINGER_BODY_GROUPS = {
    "l": {
        "thumb": (
            "finger_l_link1",
            "finger_l_link2",
            "finger_l_link3",
            "finger_l_link4",
        ),
        "index": (
            "finger_l_link5",
            "finger_l_link6",
            "finger_l_link7",
            "finger_l_link8",
        ),
        "middle": (
            "finger_l_link9",
            "finger_l_link10",
            "finger_l_link11",
            "finger_l_link12",
        ),
    },
    "r": {
        "thumb": (
            "finger_r_link1",
            "finger_r_link2",
            "finger_r_link3",
            "finger_r_link4",
        ),
        "index": (
            "finger_r_link5",
            "finger_r_link6",
            "finger_r_link7",
            "finger_r_link8",
        ),
        "middle": (
            "finger_r_link9",
            "finger_r_link10",
            "finger_r_link11",
            "finger_r_link12",
        ),
    },
}
BODY_TO_FINGER_GROUP = {
    side: {body: group for group, bodies in groups.items() for body in bodies}
    for side, groups in FINGER_BODY_GROUPS.items()
}

CAN_GEOM_NAME = "can_geom"

# ``apply_grasp``는 물리 서브스텝마다 한 번씩 호출되어 파지 시험 한 번에 수천 번
# 실행된다. 이전에는 매번 ``mj_name2id``로 관절을 다시 찾고, 전달 액추에이터를 찾기
# 위해 전체 액추에이터를 O(nu) Python 순회했다. 약지·새끼 관절 6개를 추가한 뒤
# Phase 4 파지 시험이 눈에 띄게 느려졌으며, 순회 포함 약 1.1 ms/step에 비해
# ``mj_step`` 단독은 약 0.1 ms/step이었다. 각 ``(model, joint_name) -> (jid, aid)``
# 조회를 처음 한 번만 수행해 동작 변화 없이 반복 비용을 없앤다. 테스트마다 새
# ``MjModel``을 만들기 때문에 캐시 키에는 ``id(model)``을 사용한다.
_JOINT_ACTUATOR_CACHE = {}
_COMMAND_COEFFICIENT_CACHE = {}


def _validate_side(side):
    """손 구분자가 ``'l'`` 또는 ``'r'``인지 확인하고 잘못된 값은 거부한다."""
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}, got {side!r}")


def _resolve_joint_actuator(model, joint_name):
    """관절 이름 -> (joint id, actuator id) 조회를 캐싱한다.

    관절 자체는 mj_name2id로 바로 찾지만, "이 관절을 움직이는 actuator가 몇 번인지"는
    MuJoCo가 직접 안 알려줘서 전체 actuator를 선형 탐색(O(nu))해야 한다 -- 매 물리
    스텝마다 이 탐색을 반복하면 비용이 커서(실측: mj_step 단독 0.1ms/스텝 vs 캐싱 전
    1.1ms/스텝) 처음 조회한 결과를 (model, joint_name) 키로 캐싱해둔다.
    해당 관절에 actuator가 아예 없는 모델(hand_only.xml/arm_hand.xml의 약지·새끼)에서는
    aid가 None으로 캐싱된다 -- 호출부에서 반드시 None 체크할 것.
    """
    key = (id(model), joint_name)
    cached = _JOINT_ACTUATOR_CACHE.get(key)
    if cached is not None:
        return cached
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    aid = mujoco_utils.find_actuator_for_joint(model, jid) if jid != -1 else None
    _JOINT_ACTUATOR_CACHE[key] = (jid, aid)
    return jid, aid


def _command_coefficients(model, side):
    """모델마다 한 번씩 한 손의 ``ctrl = offset + grasp*g + thumb*t``를 구성한다."""
    key = (id(model), side)
    cached = _COMMAND_COEFFICIENT_CACHE.get(key)
    if cached is not None:
        return cached

    actuator_ids = []
    offsets = []
    grasp_slopes = []
    thumb_slopes = []

    def add(joint_name, offset, grasp_slope=0.0, thumb_slope=0.0):
        """관절 하나의 actuator ID와 선형 명령 계수를 현재 손 목록에 추가한다."""
        joint_id, actuator_id = _resolve_joint_actuator(model, joint_name)
        if joint_id == -1 or actuator_id is None:
            raise ValueError(f"no actuated joint found for {joint_name}")
        actuator_ids.append(actuator_id)
        offsets.append(offset)
        grasp_slopes.append(grasp_slope)
        thumb_slopes.append(thumb_slope)

    def joint_range(joint_name):
        """관절 이름에 대응하는 ``(최솟값, 최댓값, 범위 폭)``을 반환한다."""
        joint_id, _actuator_id = _resolve_joint_actuator(model, joint_name)
        lo, hi = model.jnt_range[joint_id]
        return lo, hi, hi - lo

    for joint_name, value in THUMB_PRESHAPE[side].items():
        add(joint_name, value)
    add(
        f"finger_{side}_joint2",
        THUMB_YAW_REST[side],
        thumb_slope=THUMB_YAW_CURL[side] - THUMB_YAW_REST[side],
    )

    for joint_name in THUMB_CURL_JOINTS[side]:
        lo, hi, span = joint_range(joint_name)
        direction = -1.0 if THUMB_CURL_OPEN_AT_HI[side] else 1.0
        open_edge = hi if direction < 0.0 else lo
        offset = open_edge + direction * THUMB_OPEN_FRAC * span
        slope = direction * (1.0 - THUMB_OPEN_FRAC) * span
        add(joint_name, offset, thumb_slope=slope)

    for finger_joints in FINGER_CURL_JOINTS[side].values():
        for joint_name in finger_joints:
            lo, _hi, span = joint_range(joint_name)
            add(
                joint_name,
                lo + FINGER_OPEN_FRAC * span,
                grasp_slope=(1.0 - FINGER_OPEN_FRAC) * span,
            )

    for joint_name in RING_PINKY_CURL_JOINTS[side]:
        joint_id, actuator_id = _resolve_joint_actuator(model, joint_name)
        if joint_id == -1 or actuator_id is None:
            continue
        lo, hi = model.jnt_range[joint_id]
        add(
            joint_name,
            lo,
            grasp_slope=RING_PINKY_MAX_FRAC * (hi - lo),
        )

    cached = tuple(
        np.asarray(values, dtype=dtype)
        for values, dtype in (
            (actuator_ids, int),
            (offsets, float),
            (grasp_slopes, float),
            (thumb_slopes, float),
        )
    )
    _COMMAND_COEFFICIENT_CACHE[key] = cached
    return cached


def apply_grasp(model, data, grasp: float, thumb: float, *, side: str):
    """[0, 1]로 제한한 두 시너지 스칼라를 액추에이터 ctrl 목표로 변환한다.

    ``grasp``는 검지와 중지의 PIP/DIP/tip을 각 관절 범위의
    ``[FINGER_OPEN_FRAC, 1.0]`` 구간에서 보간한다. ``thumb``은 엄지 MCP pitch와 IP를
    ``[THUMB_OPEN_FRAC, 1.0]`` 구간에서 보간한다. 엄지 CMC는 고정하고 yaw는 안전한
    접근 자세와 굽힌 자세 사이를 보간한다. ``side``는 왼손과 오른손을 선택한다.
    """
    _validate_side(side)
    grasp = float(np.clip(grasp, 0.0, 1.0))
    thumb = float(np.clip(thumb, 0.0, 1.0))
    actuator_ids, offsets, grasp_slopes, thumb_slopes = _command_coefficients(
        model, side
    )
    data.ctrl[actuator_ids] = offsets + grasp * grasp_slopes + thumb * thumb_slopes


def get_finger_can_contacts(model, data, *, side: str):
    """현재 캔에 닿은 손가락 그룹별 총 법선력을 사전으로 반환한다.

    그룹 이름은 ``thumb``, ``index``, ``middle`` 중 하나다. 법선력은 접촉 좌표계의
    법선 성분 크기이며, 현재 스텝에서 해당 그룹에 속한 모든 접촉점의 값을 합한다.
    """
    _validate_side(side)
    can_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, CAN_GEOM_NAME)
    body_to_group = BODY_TO_FINGER_GROUP[side]

    forces = {}
    force_vec = np.zeros(6)
    # 이번 스텝에 발생한 접촉(data.contact) 전체를 훑어서, 캔과 맞닿은 접촉만 골라
    # 어느 손가락 그룹인지 확인하고 법선력을 합산한다 -- 위치가 아니라 실제 접촉력을
    # 근거로 판정하기 위함(이 프로젝트의 핵심 규칙).
    for i in range(data.ncon):
        c = data.contact[i]
        if can_gid not in (c.geom1, c.geom2):
            continue
        other = c.geom1 if c.geom2 == can_gid else c.geom2
        bname = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[other]
        )
        group = body_to_group.get(bname)
        if group is None:
            continue
        mujoco.mj_contactForce(model, data, i, force_vec)
        normal_force = abs(force_vec[0])  # 접촉 좌표계에서 0번 성분이 법선 방향 힘이다.
        forces[group] = forces.get(group, 0.0) + normal_force
    return forces


def is_grasped(
    model,
    data,
    min_fingers=DEFAULT_MIN_FINGERS,
    min_total_force=DEFAULT_MIN_TOTAL_FORCE,
    require_thumb=DEFAULT_REQUIRE_THUMB,
    *,
    side: str,
):
    """위치나 부착 상태를 이용하지 않고 접촉력만으로 파지를 판정한다.

    서로 다른 손가락 그룹이 ``min_fingers``개 이상 캔에 닿고 합산 법선력이
    ``min_total_force`` N을 넘으면 참이다. 기본 설정에서는 엄지 접촉도 필요하다.
    """
    forces = get_finger_can_contacts(model, data, side=side)
    # 엄지가 반드시 포함되고(기본값), 서로 다른 손가락 그룹 2개 이상이 닿아 있으며,
    # 합산 법선력이 임계값을 넘어야 "쥐었다"고 판정한다 -- 셋 다 접촉력 기반이라
    # 위치/부착 치팅이 끼어들 여지가 없다.
    return (
        (not require_thumb or "thumb" in forces)
        and len(forces) >= min_fingers
        and sum(forces.values()) >= min_total_force
    )
