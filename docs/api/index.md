# API 레퍼런스

호출할 함수의 입력, 반환값과 데이터 변경 여부를 찾는 문서다. 알고리즘 설명은
[시스템 이해와 개발](../guide/index.md)에 분리했다.

## 빠른 찾기 { #api-architecture }

| 작업 | 대표 API | 문서 |
|---|---|---|
| YAML 설정 읽기 | `load_settings()` | [설정](config.md) |
| 앱 실행·모드 전환 | `TeleopApp`, `main()` | [애플리케이션](application.md#teleop) |
| target↔world 변환 | `target_world_pose()` | [애플리케이션](application.md#targets) |
| FK·Jacobian | `KinematicTree.forward_site()` | [기구학](kinematics.md#tree) |
| pose task·제약 구성 | `velocity_task()`, `joint_velocity_bounds()` | [기구학](kinematics.md#tasks) |
| Pseudoinverse·DLS·QP | `DifferentialIKSolver.solve()` | [기구학](kinematics.md#solver) |
| 충돌 거리와 gradient | `collision_distance_gradient()` | [기구학](kinematics.md#collision) |
| 전신 명령 계산 | `WholeBodyIK.solve()` | [제어](control.md#whole-body) |
| 팔·base·손 명령 | `ArmTorqueController`, `SwerveDrive`, `apply_grasp()` | [제어](control.md) |
| UI·렌더링 연결 | `draw_panel()`, `render_scene()` | [시각화](visualization.md) |

```python
from ffw_sh5_grasp.control.whole_body import WholeBodyIK
from ffw_sh5_grasp.kinematics import KinematicTree
from ffw_sh5_grasp.kinematics.solver import DifferentialIKSolver
```

## 공통 규칙

- Quaternion 배열 순서는 MuJoCo와 같은 `(w, x, y, z)`다.
- 위치는 m, 별도 표기가 없는 각도는 rad다. 함수명에 `_deg`가 있으면 degree다.
- 이름이 `_`로 시작하면 내부 구현이며 외부 호출을 전제로 하지 않는다.
- `KinematicTree`와 `DifferentialIKSolver`는 입력 배열이나 `MjData`를 바꾸지 않는다.
- `WholeBodyIK.solve()`는 명령만 반환한다. `apply()` 계열과 앱의 물리 단계가
  `data.ctrl`을 기록한다.

`src/teleop_app.py`, `src/kinematics.py`, `src/ik.py`는 기존 실행 명령과 import를 위한
호환 파일이다. 새 코드는 `ffw_sh5_grasp` 패키지에서 import한다.
