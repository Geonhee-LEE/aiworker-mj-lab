# `control.optimization`

로봇 모델을 모르는 순수 NumPy convex QP 함수다. 전신 IK는 weighted DLS 행렬을
명시적인 QP 비용으로 바꾼 뒤 box와 soft CBF 제약을 적용한다.

## `least_squares_to_qp(matrix, vector)`

- **기능:** `||Ax-b||²`를 표준 QP의 `0.5*x.T@H@x + g.T@x`로 변환한다.
- **입력:** `A (M,N)`, `b (M,)`.
- **반환:** `H=2*A.T@A`, `g=-2*A.T@b`.
- **사용:** weighted pose task, 자유도별 damping 비용과 posture task를 하나의 QP로 변환.

## `bounded_quadratic_program(H, g, lower, upper)`

- **기능:** `lower <= x <= upper`인 convex box-QP를 KKT active-set으로 푼다.
- **반환:** box를 만족하는 QP 최적해 `x (N,)`.
- **오류:** 비대칭 Hessian, shape 불일치 또는 `lower > upper`.
- **사용:** `WholeBodyIK.solve()`의 명목 18자유도 constrained DLS 해.

## `bounded_quadratic_program_with_barriers(...)`

```python
bounded_quadratic_program_with_barriers(
    H, g, lower, upper,
    barrier_matrix, barrier_lower, slack_weight,
)
```

- **기능:** box에 더해 `Gx ≥ h`를 가능한 한 지키고 불가능한 양은 slack 비용으로 푼다.
- **반환:** box를 항상 만족하고 활성 barrier 위반을 벌점 처리한 해.
- **사용:** collision CBF처럼 작은 진단 가능한 위반이 hard infeasibility보다 안전한 문제.

## 기존 least-squares 호환 함수

`bounded_least_squares()`와 `bounded_least_squares_with_barriers()`는 기존 외부 호출을
위해 유지한다. 두 함수도 내부에서 `least_squares_to_qp()`와 위 QP solver를 호출한다.
전신 런타임 본 경로는 명시적인 QP API를 직접 사용한다.
