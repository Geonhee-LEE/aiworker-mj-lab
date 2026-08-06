# `kinematics.optimization`

로봇이나 task를 모르는 저수준 NumPy active-set QP 구현이다.

| API | 의미 |
|---|---|
| `least_squares_to_qp(A, b)` | `||Ax-b||²`를 Hessian과 선형항으로 변환 |
| `bounded_quadratic_program(...)` | box-constrained convex QP 계산 |
| `bounded_quadratic_program_with_barriers(...)` | soft linear barrier slack가 있는 QP 계산 |

가중치, 단위, joint 이름, collision pair 정책은 이 모듈에 넣지 않는다. 그 정보는
각각 `tasks.py`, `constraints.py`, `whole_body.py`에 남는다.
