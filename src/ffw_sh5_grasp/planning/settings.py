"""``config/default.yaml``의 ``planning`` 블록을 읽는 유일한 지점.

``tests/test_config.py``는 YAML의 모든 leaf가 ``SETTINGS.*("리터럴.경로")`` 형태로
소스에서 실제로 읽히는지 AST로 검사한다. 그래서 이 모듈에만 리터럴 dotted path를
모아 두고, 나머지 ``planning/`` 코드는 이 모듈이 반환한 값만 받는다. P1~P4가
sampler/shortcut/trajectory/goal 설정을 실제로 소비하기 전까지는 그 블록의
YAML 키를 미리 추가하지 않는다 — 쓰지 않는 leaf는 CI에서 바로 실패한다.
"""

from dataclasses import dataclass

from ..config import SETTINGS


@dataclass(frozen=True)
class CollisionSettings:
    """``ArmCollisionChecker`` 생성에 필요한 값."""

    padding_m: float
    clearance_report_m: float
    ignore_hand_internal_contacts: bool


def load_collision_settings():
    return CollisionSettings(
        padding_m=SETTINGS.number("planning.collision.padding_m", positive=True),
        clearance_report_m=SETTINGS.number(
            "planning.collision.clearance_report_m", positive=True
        ),
        ignore_hand_internal_contacts=SETTINGS.get(
            "planning.collision.ignore_hand_internal_contacts"
        ),
    )


__all__ = ["CollisionSettings", "load_collision_settings"]
