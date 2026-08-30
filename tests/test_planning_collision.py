"""``ArmCollisionChecker``의 MuJoCo 기반 충돌 유효성 시험.

live 시뮬레이션 ``MjData``를 절대 건드리지 않는다는 계약, 상자 가시성 가드,
``can_free`` 자유 관절 스냅샷 보존을 검증한다. 렌더러를 만들지 않으므로
``MUJOCO_GL``과 무관하게 동작한다.

Headless 단독 실행: ``python3 tests/test_planning_collision.py``
"""

import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.imitation.simulation.environment import (  # noqa: E402
    enable_task_collisions,
)
from ffw_sh5_grasp.paths import MODEL_PATH  # noqa: E402
from ffw_sh5_grasp.planning import ArmCollisionChecker, RightArmSpace  # noqa: E402

REQUIRE_CONTACT_GEOMS = (
    "target_bin_floor",
    "target_bin_red_floor",
    "can_geom",
    "table",
    "floor",
)
# imitation.scenarios.can_color_sort.right_arm_start_position_rad — 실제 과업
# 시작 자세다. 일반 teleop ``home`` 자세는 상자 승격 후 상태와 일관되지 않으므로
# (상자를 승격하는 wrapper 없이 정의된 자세이기 때문) 시험에 쓰지 않는다.
CAN_SORT_START_Q = np.array(
    [-0.0381287163, -0.2902937754, 0.1700398805, -1.9341540186,
     -0.3104451628, 0.3364858776, -0.2716638607]
)


def _promoted_model():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    enable_task_collisions(model, ("target_bin", "target_bin_red"))
    return model


def _checker(model, *, padding_m=0.012):
    space = RightArmSpace.from_model(model)
    return ArmCollisionChecker(
        model,
        space,
        padding_m=padding_m,
        require_contact_geoms=REQUIRE_CONTACT_GEOMS,
    ), space


def _synced_data(model):
    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, home_key)
    mujoco.mj_forward(model, data)
    return data


def test_missing_obstacle_visibility_raises():
    raw_model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    try:
        ArmCollisionChecker(
            raw_model, padding_m=0.01, require_contact_geoms=("target_bin_floor",)
        )
    except ValueError as error:
        assert "target_bin_floor" in str(error)
    else:
        raise AssertionError("승격되지 않은 모델에서 가드가 예외를 내지 않았습니다")


def test_dive_into_table_is_invalid():
    model = _promoted_model()
    checker, space = _checker(model)
    data = _synced_data(model)
    checker.set_snapshot(data)
    dive_q = np.array([0.0, -1.5, 0.0, -2.9, 0.0, -1.5, 0.0])
    assert not checker.is_valid(dive_q)


def test_out_of_range_configuration_is_invalid_via_joint_limit():
    model = _promoted_model()
    checker, space = _checker(model)
    data = _synced_data(model)
    checker.set_snapshot(data)
    oob_q = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    report = checker.report(oob_q)
    assert not report.valid
    assert report.pair_name == "joint-limit"


def test_live_data_is_never_mutated():
    model = _promoted_model()
    checker, space = _checker(model)
    data = _synced_data(model)
    checker.set_snapshot(data)

    qpos_before = data.qpos.copy()
    qvel_before = data.qvel.copy()
    ctrl_before = data.ctrl.copy()
    time_before = data.time

    rng = np.random.default_rng(2)
    for _ in range(200):
        checker.is_valid(space.sample(rng))

    assert np.array_equal(qpos_before, data.qpos)
    assert np.array_equal(qvel_before, data.qvel)
    assert np.array_equal(ctrl_before, data.ctrl)
    assert time_before == data.time


def test_snapshot_preserves_free_joint():
    model = _promoted_model()
    checker, space = _checker(model)
    data = _synced_data(model)

    can_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    can_adr = model.jnt_qposadr[can_joint_id]
    # 캔을 원래 위치에서 멀리 옮긴 뒤 스냅샷을 뜬다.
    displaced = data.qpos.copy()
    displaced[can_adr : can_adr + 3] = [1.5, 1.5, 1.5]
    data.qpos[:] = displaced
    checker.set_snapshot(data)

    assert np.allclose(
        checker.snapshot_qpos[can_adr : can_adr + 3], [1.5, 1.5, 1.5]
    )


def test_check_cost_is_bounded():
    import time as _time

    model = _promoted_model()
    checker, space = _checker(model)
    data = _synced_data(model)
    checker.set_snapshot(data)

    rng = np.random.default_rng(4)
    samples = [space.sample(rng) for _ in range(500)]
    start = _time.perf_counter()
    for q in samples:
        checker.is_valid(q)
    elapsed = _time.perf_counter() - start
    print(f"{len(samples)} checks in {elapsed:.3f}s -> {len(samples) / elapsed:.0f}/s")
    # CI 러너는 느리고 공유 자원이므로 느슨하게 잡는다. 실측치는 벤치마크 TSV가
    # 별도로 남긴다(사람이 읽는 참고용, 회귀 감지가 목적이 아니다).
    assert elapsed < 2.0


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
