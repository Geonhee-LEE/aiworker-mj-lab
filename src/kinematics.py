"""기존 ``import kinematics``를 유지하는 호환 facade.

새 코드는 :mod:`ffw_sh5_grasp.kinematics`를 직접 사용한다.
"""

from ffw_sh5_grasp.kinematics import *  # noqa: F401,F403
