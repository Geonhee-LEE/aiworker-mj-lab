"""오른팔 configuration의 충돌 유효성을 판정하는 scratch 상태 검사기.

sampling 플래너는 초당 수천 번의 boolean ``is_valid(q)``가 필요하다. 기존
``kinematics.collision.collision_distance_gradient``는 gradient까지 계산하는
무거운 함수이므로 그대로 쓰지 않는다. 대신 MuJoCo 엔진 자체의 broad-phase
(``mj_kinematics`` + ``mj_collision``)를 쓰고, 유효 여유 거리만큼 관련 geom의
``geom_margin``/``pair_margin``을 부풀려 판정한다. 이러면 모델에 이미 선언된
``<exclude>``와 ``<pair>`` 규칙(손가락 체인 제외, palm-table 명시 margin 등)을
다시 구현하지 않고 그대로 물려받는다.

live 시뮬레이션 ``MjData``는 ``set_snapshot``에서 한 번만 읽고, 그 뒤로는 절대
건드리지 않는다.
"""

import copy
from dataclasses import dataclass

import mujoco
import numpy as np

from ..kinematics.collision import collision_distance_gradient
from ..kinematics.tree import KinematicTree
from .arm_state import RightArmSpace
from .obstacles import right_arm_collision_pairs

_FROZEN_HAND_BODY_PREFIXES = ("hx5_r_base", "finger_r_link")


@dataclass(frozen=True)
class CollisionReport:
    """단일 configuration의 유효성 판정 요약."""

    valid: bool
    minimum_distance: float
    pair_name: str


class ArmCollisionChecker:
    """오른팔 configuration의 충돌·관절한계 유효성을 판정한다.

    live 시뮬레이션 ``MjData``는 절대 건드리지 않는다. 생성자에서 전용 scratch
    모델(``copy.deepcopy``)과 ``MjData``를 만들고, ``set_snapshot``으로 기준
    상태(base, lift, head, 왼팔, 손가락, ``can_free`` 포함 전체 qpos)를 복사한
    뒤 이후에는 오른팔 qpos 주소만 덮어쓴다.
    """

    def __init__(
        self,
        model,
        space=None,
        *,
        padding_m,
        clearance_report_m=0.2,
        ignore_hand_internal_contacts=True,
        require_contact_geoms=(),
        collision_pairs=None,
    ):
        if padding_m < 0.0:
            raise ValueError("padding_m은 음수일 수 없습니다")
        self.model = copy.deepcopy(model)
        self.space = space if space is not None else RightArmSpace.from_model(self.model)
        self.padding_m = float(padding_m)
        self.clearance_report_m = float(clearance_report_m)
        self.ignore_hand_internal_contacts = bool(ignore_hand_internal_contacts)
        self._checks = 0

        self._planned_geoms = self._collect_planned_geoms(self.model)
        self._frozen_hand_bodies = self._collect_frozen_hand_bodies(self.model)
        self._pad_margins(self.model, self._planned_geoms, self.padding_m)

        for geom_name in require_contact_geoms:
            self._assert_can_collide_with_planned_set(self.model, geom_name)

        self.tree = KinematicTree(self.model)
        self.data = mujoco.MjData(self.model)
        self._collision_pairs = (
            collision_pairs
            if collision_pairs is not None
            else right_arm_collision_pairs(self.model)
        )
        self._snapshot_qpos = np.asarray(self.model.qpos0, dtype=float).copy()

    @staticmethod
    def _collect_planned_geoms(model):
        prefixes = ("arm_r_link",) + _FROZEN_HAND_BODY_PREFIXES
        geoms = []
        for geom_id in range(model.ngeom):
            if int(model.geom_contype[geom_id]) == 0:
                continue
            body_id = int(model.geom_bodyid[geom_id])
            body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
            if body_name.startswith(prefixes):
                geoms.append(geom_id)
        return frozenset(geoms)

    @staticmethod
    def _collect_frozen_hand_bodies(model):
        bodies = []
        for body_id in range(model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
            if name.startswith(_FROZEN_HAND_BODY_PREFIXES):
                bodies.append(body_id)
        return frozenset(bodies)

    @staticmethod
    def _pad_margins(model, planned_geoms, padding_m):
        if padding_m <= 0.0 or not planned_geoms:
            return
        indices = np.fromiter(planned_geoms, dtype=int)
        model.geom_margin[indices] += padding_m
        # 명시적 <pair> 행(예: palm-table/floor)은 margin을 geom_margin이 아닌
        # pair_margin에서 가져온다. 여기를 빼먹으면 그 표면만 패딩이 안 먹는다.
        if model.npair > 0:
            in_planned_1 = np.isin(model.pair_geom1, indices)
            in_planned_2 = np.isin(model.pair_geom2, indices)
            rows = np.flatnonzero(in_planned_1 | in_planned_2)
            model.pair_margin[rows] += padding_m

    @staticmethod
    def _assert_can_collide_with_planned_set(model, geom_name):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        if geom_id < 0:
            raise ValueError(f"장애물 geom을 모델에서 찾을 수 없습니다: {geom_name!r}")
        contype = int(model.geom_contype[geom_id])
        conaffinity = int(model.geom_conaffinity[geom_id])
        planned = ArmCollisionChecker._collect_planned_geoms(model)
        for other in planned:
            other_contype = int(model.geom_contype[other])
            other_conaffinity = int(model.geom_conaffinity[other])
            if (contype & other_conaffinity) or (other_contype & conaffinity):
                return
        raise ValueError(
            f"장애물 geom {geom_name!r}이 계획 대상 오른팔과 충돌할 수 없습니다 "
            "(contype/conaffinity 불일치 — 상자라면 enable_task_collisions로 "
            "승격한 모델을 넘겼는지 확인하세요)"
        )

    def set_snapshot(self, data):
        """live ``data``의 전체 qpos를 스냅샷으로 복사한다(캔 free joint 포함)."""
        qpos = np.asarray(data.qpos, dtype=float)
        if qpos.shape != self._snapshot_qpos.shape:
            raise ValueError(
                f"qpos 크기가 다릅니다: 기대 {self._snapshot_qpos.shape}, 받음 {qpos.shape}"
            )
        self._snapshot_qpos = qpos.copy()

    @property
    def snapshot_qpos(self):
        return self._snapshot_qpos.copy()

    @property
    def state_checks(self):
        return self._checks

    def _is_frozen_hand_body(self, body_id):
        return body_id in self._frozen_hand_bodies

    def _disqualifying_contact(self, allowed_geom_pairs):
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            geom_a, geom_b = int(contact.geom1), int(contact.geom2)
            if geom_a not in self._planned_geoms and geom_b not in self._planned_geoms:
                continue
            if self.ignore_hand_internal_contacts:
                body_a = int(self.model.geom_bodyid[geom_a])
                body_b = int(self.model.geom_bodyid[geom_b])
                if self._is_frozen_hand_body(body_a) and self._is_frozen_hand_body(body_b):
                    continue
            key = tuple(sorted((geom_a, geom_b)))
            if key in allowed_geom_pairs:
                continue
            return contact
        return None

    def _forward(self, q):
        self._checks += 1
        self.data.qpos[:] = self._snapshot_qpos
        self.space.write(self.data.qpos, q)
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_collision(self.model, self.data)

    def is_valid(self, q, *, allowed_geom_pairs=frozenset()):
        """``q``(오른팔 7-DOF)가 관절 범위 안이고 충돌이 없으면 ``True``."""
        if not self.space.contains(q):
            return False
        self._forward(q)
        return self._disqualifying_contact(allowed_geom_pairs) is None

    def report(self, q, *, allowed_geom_pairs=frozenset()):
        """유효성과 함께 위반된 접촉(있다면)의 이름·거리를 반환한다."""
        if not self.space.contains(q):
            return CollisionReport(False, float("-inf"), "joint-limit")
        self._forward(q)
        contact = self._disqualifying_contact(allowed_geom_pairs)
        if contact is None:
            return CollisionReport(True, float(self.padding_m), "")
        geom_a = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1))
        geom_b = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2))
        return CollisionReport(False, float(contact.dist), f"{geom_a}/{geom_b}")

    def clearance(self, q):
        """벤치마크용 정확한 최소 signed distance. ``is_valid``보다 훨씬 비싸다."""
        self.data.qpos[:] = self._snapshot_qpos
        self.space.write(self.data.qpos, q)
        frame_cache = {}
        best = float("inf")
        for pair in self._collision_pairs:
            constraint = collision_distance_gradient(
                self.model,
                self.data,
                pair,
                self.tree,
                self.space.joint_ids,
                self.clearance_report_m,
                frame_cache,
            )
            if constraint is not None:
                best = min(best, float(constraint.distance))
        return best


__all__ = ["ArmCollisionChecker", "CollisionReport"]
