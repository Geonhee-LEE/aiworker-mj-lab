# `kinematics.legacy`

이전 버전의 import 경로를 유지하는 호환 모듈이다. 새 기능 구현은
[`KinematicsSolver`](kinematics-solver.md)를 직접 사용한다.

## `InverseKinematics`

`KinematicsSolver`를 상속한 호환 클래스다. 생성자, `forward()`,
`forward_kinematics()`, `solve_pose()`와 `solve_pose_multistart()`의 동작은 동일하다.

```python
# 기존 코드: 계속 지원
from ffw_sh5_grasp.kinematics.legacy import InverseKinematics

# 새 코드: 권장
from ffw_sh5_grasp.kinematics import KinematicsSolver
```

호환 모듈에는 별도 알고리즘 구현이 없으며 제거 시점이 정해진 deprecated API도 아니다.
