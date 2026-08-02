# API 레퍼런스

이 절은 “이 함수가 내부적으로 어떤 수식을 쓰는가”보다 **무엇을 넣으면 무엇이
나오고, 언제 호출해야 하는가**를 빠르게 찾기 위한 참고서다. 수식의 유도와 설계
근거는 각 함수의 `상세 원리` 링크에서 읽는다.

## 읽는 법

각 항목은 같은 순서를 사용한다.

1. **직관**: 로봇 동작으로 바꾸어 설명한 한 문장
2. **입력**: 좌표계·단위·필수 상태
3. **반환/변경**: 반환값과 수정되는 객체
4. **사용 시점**: 새 코드에서 호출할 상황

이름이 `_`로 시작하는 함수와 메서드는 구현 세부사항이다. 테스트에서 특정 내부
단계를 검증하는 경우가 아니면 직접 호출하지 않는다.

## 패키지별 찾기

| 하고 싶은 일 | API 문서 | 원리 문서 |
|---|---|---|
| YAML 설정 읽기·검증 | [설정 API](config.md) | [YAML 파라미터 설정](../configuration.md) |
| 앱 실행, 모드 변경, 목표 좌표 변환 | [애플리케이션 API](application.md) | [앱 조립](../guide/teleop_app.md), [목표 좌표](../guide/teleop_targets.md) |
| Quaternion, Tree, FK/Jacobian, 단일 팔 IK, 충돌 거리 | [기구학 API](kinematics.md) | [기구학 학습 안내](../guide/kinematics.md) |
| 전신 IK, 최적화, 팔·베이스·손 제어 | [제어 API](control.md) | [전신 IK](../guide/whole_body_ik.md) |
| UI, 장면, 카메라, Gizmo | [시각화 API](visualization.md) | [UI](../guide/teleop_ui.md), [렌더링](../guide/teleop_render.md) |

## 가장 자주 쓰는 진입점

```python
from ffw_sh5_grasp.config import load_settings
from ffw_sh5_grasp.kinematics import KinematicTree, KinematicsSolver
from ffw_sh5_grasp.control.arm import ArmTorqueController
from ffw_sh5_grasp.control.base import BodyTwist, SwerveDrive
from ffw_sh5_grasp.control.whole_body import WholeBodyIK
```

| 목적 | 첫 호출 | 핵심 결과 |
|---|---|---|
| 앱 실행 | `application.teleop.main()` | 텔레옵 frame loop 시작 |
| 전신/팔 전용 명령 | `WholeBodyIK.solve()` | base twist, lift·팔 목표각 |
| 단일 팔 IK | `KinematicsSolver.solve_pose()` | 목표 pose에 가까운 관절각 |
| Site FK/Jacobian | `KinematicTree.forward_site()` | world pose와 (6\times N) Jacobian |
| 차체 속도를 바퀴 명령으로 | `SwerveDrive.update_twist()` | 바퀴별 조향각·회전속도 |
| 팔 목표각을 토크로 | `ArmTorqueController.apply()` | 팔 actuator torque |
| 손가락 닫기 | `grasp.apply_grasp()` | 손가락 actuator target |

!!! note "호환 진입점"
    `src/teleop_app.py`, `src/kinematics.py`, `src/ik.py`는 기존 실행 명령과 import를
    위한 얇은 호환 계층이다. 새 코드는 `ffw_sh5_grasp` 패키지에서 import한다.
