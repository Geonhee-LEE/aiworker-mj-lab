"""텔레옵 모듈이 공유하는 작은 MuJoCo 모델 조회 보조 함수.

이전에는 ``grasp.py``와 ``control/arm.py``가 어떤 액추에이터가 관절을 구동하는지
찾는 선형 탐색을 각각 복사해 사용했다. MuJoCo는 액추에이터에서 관절로 가는
``actuator_trnid``만 제공하고 반대 방향 조회는 직접 제공하지 않으므로 두 호출부가
모든 액추에이터를 순회해 비교해야 했다. 전달 방식 대응 논리를 한곳에 모아 앞으로
변경이 필요할 때 한 구현만 수정하도록 했다.
"""

import mujoco


def find_actuator_for_joint(model, joint_id):
    """직접 관절 전달 방식으로 ``joint_id``를 구동하는 액추에이터 ID를 반환한다.

    MuJoCo는 액추에이터에서 관절로 가는 ``actuator_trnid``만 제공하므로 전체
    액추에이터를 선형 탐색한다. 해당 액추에이터가 없으면 ``None``을 반환한다.
    """
    for aid in range(model.nu):
        if (model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT
                and model.actuator_trnid[aid, 0] == joint_id):
            return aid
    return None
