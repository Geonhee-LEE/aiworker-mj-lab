"""오른팔 exact clearance 보고용 충돌 쌍 목록.

boolean 유효성 검사(``collision_state.ArmCollisionChecker.is_valid``)는 MuJoCo의
엔진 레벨 broad-phase(``mj_collision``)를 그대로 쓰고 여기서 만드는 쌍 목록에
의존하지 않는다. 이 모듈은 오직 ``clearance()``의 정확한 signed-distance 보고를
위해서만 쓰인다 — 기존 ``kinematics.collision.collision_distance_gradient``를
재사용해 convex mesh/box 보정과 tabletop/bounding-sphere 처리를 다시 구현하지 않는다.
"""

import itertools

import mujoco

from ..kinematics.collision import CollisionPair, default_collision_pairs

RIGHT_ARM_BODIES = tuple(f"arm_r_link{i}" for i in range(1, 8)) + ("hx5_r_base",)
# default_collision_pairs가 다루지 않는 장애물: 상자, 바닥.
# ``can_geom``은 일부러 뺐다 — ``can_free``는 free joint라 KinematicTree의
# FK/Jacobian이 지원하는 scalar hinge/slide 체인이 아니고, 이 body를 지나는
# point_jacobian 호출은 NotImplementedError를 낸다(kinematics/tree.py 참고).
# 캔과의 충돌은 boolean is_valid() 쪽에서 mj_collision의 broad-phase로 이미
# 정확히 잡힌다 — clearance()의 exact-distance 보고에서만 캔이 빠진다.
_EXTRA_OBSTACLE_GEOMS = (
    "target_bin_floor",
    "target_bin_front",
    "target_bin_back",
    "target_bin_left",
    "target_bin_right",
    "target_bin_red_floor",
    "target_bin_red_front",
    "target_bin_red_back",
    "target_bin_red_left",
    "target_bin_red_right",
    "floor",
)


def _representative_geom(model, body_name):
    """``kinematics.collision.default_collision_pairs``와 같은 우선순위로 body의
    대표 충돌 geom 하나를 고른다(group 3 mesh 우선, group 0 box는 fallback)."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        return None
    candidates = [
        geom_id
        for geom_id in range(model.ngeom)
        if int(model.geom_bodyid[geom_id]) == body_id
        and int(model.geom_contype[geom_id]) != 0
    ]
    if not candidates:
        return None
    candidates.sort(key=lambda geom_id: int(model.geom_group[geom_id]) != 3)
    return candidates[0]


def right_arm_collision_pairs(model):
    """오른팔 관련 충돌 쌍 전체 — 기존 curated 집합 중 오른팔 관련분 + 상자/캔/바닥."""
    base_pairs = tuple(
        pair
        for pair in default_collision_pairs(model)
        if "arm_r_" in pair.name or "hx5_r_" in pair.name
    )

    pairs = {tuple(sorted((pair.geom_a, pair.geom_b))): pair for pair in base_pairs}

    obstacle_geoms = []
    for name in _EXTRA_OBSTACLE_GEOMS:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id >= 0:
            obstacle_geoms.append((name, geom_id))

    for body_name, (obstacle_name, obstacle_geom) in itertools.product(
        RIGHT_ARM_BODIES, obstacle_geoms
    ):
        arm_geom = _representative_geom(model, body_name)
        if arm_geom is None or arm_geom == obstacle_geom:
            continue
        key = tuple(sorted((arm_geom, obstacle_geom)))
        if key in pairs:
            continue
        pairs[key] = CollisionPair(
            f"obstacle:{body_name}/{obstacle_name}", key[0], key[1], "geom"
        )

    return tuple(pairs.values())


__all__ = ["RIGHT_ARM_BODIES", "right_arm_collision_pairs"]
