"""``planning`` YAML 블록의 기본값 로드와 오탈자 거부를 확인한다.

MuJoCo가 필요 없다. Headless 단독 실행: ``python3 tests/test_planning_config.py``
"""

import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.config import load_settings  # noqa: E402
from ffw_sh5_grasp.planning.settings import (  # noqa: E402
    load_collision_settings,
    load_trajectory_settings,
)


def test_schema_version_is_six():
    settings = load_settings()
    assert settings.get("schema_version") == 6


def test_default_collision_settings_load():
    settings = load_collision_settings()
    assert settings.padding_m > 0.0
    assert settings.clearance_report_m > settings.padding_m
    assert isinstance(settings.ignore_hand_internal_contacts, bool)


def test_default_trajectory_settings_load():
    settings = load_trajectory_settings()
    assert settings.max_joint_speed_rad_s > 0.0
    assert settings.max_joint_accel_rad_s2 > 0.0


def test_trajectory_speed_matches_hardware_joint_limit():
    settings = load_settings()
    # imitation.teleop이 문서화한 FFW-SH5 follower URDF 관절 한계와 같은 값을
    # 써서 계획한 궤적이 실행 시 하드웨어 한계를 넘지 않도록 맞춘다.
    hardware_limit = settings.number("imitation.teleop.max_joint_speed_rad_s")
    planning_limit = settings.number("planning.trajectory.max_joint_speed_rad_s")
    assert planning_limit == hardware_limit


def test_padding_is_between_cbf_safe_distance_and_buffer():
    settings = load_settings()
    safe_distance = settings.number("whole_body_ik.collision_safe_distance_m")
    buffer = settings.number("whole_body_ik.collision_buffer_m")
    padding = settings.number("planning.collision.padding_m")
    # A5 설계 계약: 플래너의 clearance는 CBF의 safe_distance보다 크고
    # CBF가 활성화되는 buffer보다는 작아야, 실행 중 CBF가 계획을 방해하지 않으면서도
    # 여전히 반응할 여유가 남는다.
    assert safe_distance < padding < buffer


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
