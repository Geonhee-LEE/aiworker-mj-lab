# `control.optimization`

로봇 모델을 모르는 순수 NumPy bounded least-squares 함수다.

## `bounded_least_squares(matrix, vector, lower, upper)`

- **기능:** `Ax ≈ b`를 최소화하면서 각 변수를 box 안에 둔다.
- **입력:** `A (M,N)`, `b (M,)`, lower/upper `(N,)`.
- **반환:** 제한된 해 `x (N,)`.
- **오류:** shape 불일치 또는 `lower > upper`.

## `bounded_least_squares_with_barriers(...)`

```python
bounded_least_squares_with_barriers(
    A, b, lower, upper,
    barrier_matrix, barrier_lower, slack_weight,
)
```

- **기능:** box에 더해 `Gx ≥ h`를 가능한 한 지키고 불가능한 양은 slack 비용으로 푼다.
- **반환:** box를 항상 만족하고 활성 barrier 위반을 벌점 처리한 해.
- **사용:** collision CBF처럼 작은 진단 가능한 위반이 hard infeasibility보다 안전한 문제.

두 함수는 [전신 IK API](control-whole-body.md)가 task row와 bound를 조립한 뒤 호출한다.
