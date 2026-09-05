"""P7.1: 월드 프레임 목표에서 후보 베이스 (x, y, yaw)를 고른다.

P7.0의 ``ReachabilityMap``은 베이스 프레임 기준이다 — 이 모듈이 월드
좌표를 베이스 프레임으로 바꾸는 책임을 진다. 후보 베이스 자세마다
(1) reachability map 점수 (2) 베이스 발자국 충돌 없음 (3) 현재 베이스
위치와의 근접도 순으로 정렬해 최선을 고른다.

베이스가 목표 자세에 도착한 뒤에는 기존 고정-베이스 팔 계획 파이프라인
(``RightArmSpace``/``ArmCollisionChecker``/``EdgeChecker``/``plan_rrt_connect``)
을 그대로, 무수정으로 재사용한다 — 이 모듈이 하는 일은 "베이스를 어디에
둘까" 뿐이다.
"""

import copy
import math
from dataclasses import dataclass

import mujoco
import numpy as np

_BASE_JOINT_NAMES = ("base_x", "base_y", "base_yaw")


@dataclass(frozen=True)
class BasePoseResult:
    """``select_base_pose`` 한 번의 결과."""

    success: bool
    base_pose: object  # (x, y, yaw) np.ndarray 또는 실패 시 None
    reachability_score: float
    reason: str


class BaseFootprintChecker:
    """베이스 발자국(``base_link`` 충돌 geom)이 정적 장애물과 겹치는지 판정한다.

    ``ArmCollisionChecker``와 같은 아키텍처를 그대로 따른다 — scratch
    ``copy.deepcopy`` 모델, live ``MjData``는 ``set_snapshot``에서 한 번만
    읽고 절대 건드리지 않으며, ``mj_kinematics``+``mj_collision``만 쓴다
    (새 충돌 알고리즘을 만들지 않는다). 바퀴 geom은 ``base_link``에 속하지
    않으므로(별도 body) 자동으로 제외된다 — 바퀴-지면 접촉을 오판하지 않는다.
    """

    def __init__(self, model, *, padding_m=0.05):
        if padding_m < 0.0:
            raise ValueError("padding_m은 음수일 수 없습니다")
        self.model = copy.deepcopy(model)
        self.padding_m = float(padding_m)

        base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if base_body_id < 0:
            raise ValueError("모델에 base_link 바디가 없습니다")
        self._base_geoms = frozenset(
            geom_id
            for geom_id in range(self.model.ngeom)
            if int(self.model.geom_bodyid[geom_id]) == base_body_id
            and int(self.model.geom_contype[geom_id]) != 0
        )
        if self.padding_m > 0.0 and self._base_geoms:
            indices = np.fromiter(self._base_geoms, dtype=int)
            self.model.geom_margin[indices] += self.padding_m

        joint_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in _BASE_JOINT_NAMES
        ]
        if any(joint_id < 0 for joint_id in joint_ids):
            raise ValueError("모델에 base_x/base_y/base_yaw 관절이 없습니다")
        self._qpos_adrs = np.array(
            [self.model.jnt_qposadr[joint_id] for joint_id in joint_ids], dtype=int
        )

        self.data = mujoco.MjData(self.model)
        self._snapshot_qpos = np.asarray(self.model.qpos0, dtype=float).copy()

    def set_snapshot(self, data):
        """live ``data``의 전체 qpos를 스냅샷으로 복사한다."""
        qpos = np.asarray(data.qpos, dtype=float)
        if qpos.shape != self._snapshot_qpos.shape:
            raise ValueError(
                f"qpos 크기가 다릅니다: 기대 {self._snapshot_qpos.shape}, 받음 {qpos.shape}"
            )
        self._snapshot_qpos = qpos.copy()

    @property
    def snapshot_qpos(self):
        return self._snapshot_qpos.copy()

    def is_valid(self, base_pose):
        """``base_pose``(x, y, yaw)에서 베이스 발자국이 무충돌이면 ``True``."""
        self.data.qpos[:] = self._snapshot_qpos
        self.data.qpos[self._qpos_adrs] = np.asarray(base_pose, dtype=float)
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_collision(self.model, self.data)
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            if int(contact.geom1) in self._base_geoms or int(contact.geom2) in self._base_geoms:
                return False
        return True


def world_to_base_frame(target_world_xyz, base_pose):
    """월드 좌표를 베이스 프레임(원점 (x, y), yaw 회전)으로 바꾼다.

    ``base_link``은 z 방향으로는 절대 움직이지 않는다(``models/full_scene.xml``
    — base_x/base_y는 world x/y 축을 따르는 slide, base_yaw는 z축 둘레
    hinge뿐이고, 높이는 별도의 ``lift_joint``가 담당한다). 그래서 z는 그대로
    통과시킨다 — P7.0의 reachability 격자 z 범위(0.5~1.9)가 그대로 월드
    z 절대값과 같은 이유이기도 하다(격자를 만들 때 베이스가 원점이었으므로).
    """
    target_world_xyz = np.asarray(target_world_xyz, dtype=float)
    base_x, base_y, base_yaw = (float(v) for v in base_pose)
    dx = target_world_xyz[0] - base_x
    dy = target_world_xyz[1] - base_y
    cos_t, sin_t = math.cos(-base_yaw), math.sin(-base_yaw)
    relative_x = cos_t * dx - sin_t * dy
    relative_y = sin_t * dx + cos_t * dy
    return np.array([relative_x, relative_y, target_world_xyz[2]])


def select_base_pose(
    reachability_map,
    target_world_xyz,
    *,
    footprint_checker,
    current_base_pose,
    candidate_radii,
    candidate_angles,
    min_reachability=0.5,
):
    """목표 주위 후보 베이스 (x, y, yaw)를 만들어 최선을 고른다.

    위치는 ``target_world_xyz``에서 ``candidate_radii``만큼 떨어진
    ``candidate_angles`` 방향 원 위에 놓는다. 방향(yaw)은 로봇이 실제로
    어느 쪽을 "정면"으로 팔을 뻗는지 가정하지 않기 위해 — reachability
    map의 실측 도달 영역이 base 프레임 +x보다 -y 쪽으로 더 넓게 퍼져
    있어(P7.0 기본 격자 y∈[-1.1, 0.2] 참고) 특정 축을 전제하면 틀리기
    쉽다 — 같은 ``candidate_angles`` 집합을 yaw 후보로도 그대로 재사용해
    (위치, 방향) 전체 조합 중 점수가 가장 높은 것을 찾는다.

    같은 점수면 ``current_base_pose``에 더 가까운 후보를 우선한다(불필요한
    이동을 피함). 후보가 하나도 없으면 ``success=False``를 반환한다.
    """
    target_world_xyz = np.asarray(target_world_xyz, dtype=float)
    current_base_pose = np.asarray(current_base_pose, dtype=float)

    best = None
    best_key = None
    for radius in candidate_radii:
        for position_angle in candidate_angles:
            base_x = target_world_xyz[0] - radius * math.cos(position_angle)
            base_y = target_world_xyz[1] - radius * math.sin(position_angle)
            for base_yaw in candidate_angles:
                candidate = np.array([base_x, base_y, base_yaw])
                if not footprint_checker.is_valid(candidate):
                    continue
                relative = world_to_base_frame(target_world_xyz, candidate)
                score = reachability_map.query(relative)
                if score < min_reachability:
                    continue
                distance_from_current = float(
                    np.linalg.norm(candidate[:2] - current_base_pose[:2])
                )
                key = (-score, distance_from_current)
                if best_key is None or key < best_key:
                    best_key = key
                    best = BasePoseResult(True, candidate, score, "")

    if best is None:
        return BasePoseResult(
            False, None, 0.0,
            "reachability 점수·충돌 조건을 만족하는 후보 베이스 자세를 찾지 못함",
        )
    return best


__all__ = ["BaseFootprintChecker", "BasePoseResult", "select_base_pose", "world_to_base_frame"]
