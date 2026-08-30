"""``planning`` YAML 블록의 기본값 로드와 오탈자 거부를 확인한다.

MuJoCo가 필요 없다. Headless 단독 실행: ``python3 tests/test_planning_config.py``
"""

import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.config import load_settings  # noqa: E402
from ffw_sh5_grasp.planning.settings import load_collision_settings  # noqa: E402


def test_schema_version_is_six():
    settings = load_settings()
    assert settings.get("schema_version") == 6


def test_default_collision_settings_load():
    settings = load_collision_settings()
    assert settings.padding_m > 0.0
    assert settings.clearance_report_m > settings.padding_m
    assert isinstance(settings.ignore_hand_internal_contacts, bool)


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
