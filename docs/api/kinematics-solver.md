# `kinematics.solver`

`solver.py`는 실제 수치 해법만 소유한다. FK, task 생성, constraint 생성과 UI 상태를
알지 않는다.

## `DifferentialIKSolver`

```python
solver = DifferentialIKSolver(method="qp")
qdot = solver.solve(matrix, target, lower, upper)
```

| API | 의미 |
|---|---|
| `IKMethod` | `pseudoinverse`, `dls`, `qp` 열거형과 alias 검증 |
| `set_method(method)` | 다음 frame에 사용할 해법 변경 |
| `solve(A, b, lower, upper)` | weighted task와 box bound를 선택 해법으로 계산 |
| `enforce_constraints(...)` | collision soft-CBF를 무차원 속도 공간에서 projection |

- Pseudoinverse는 SVD Moore–Penrose 최소노름 해다.
- DLS는 `A.T @ A + damping² I`를 사용한다.
- QP는 `kinematics.optimization`의 box active-set을 사용한다.
- 세 해법 모두 고정/포화 DOF를 active set으로 처리한다.

soft objective는 [`kinematics.tasks`](kinematics-tasks.md), box와 barrier 생성은
[`kinematics.constraints`](kinematics-constraints.md)을 참고한다.
