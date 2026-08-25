"""Whole-body differential IK의 수치 solver 진입점.

이 모듈은 pseudoinverse, DLS, QP 해법과 safety projection만 소유한다. MJCF
FK/Jacobian은 :mod:`.tree`, soft task는 :mod:`.tasks`, hard bound 생성은
:mod:`.constraints`, 저수준 active-set QP는 :mod:`.optimization`에 위임한다.
"""

from enum import Enum

import numpy as np

from ..config import SETTINGS
from .optimization import (
    bounded_quadratic_program,
    bounded_quadratic_program_with_barriers,
    least_squares_to_qp,
)

DEFAULT_DIFFERENTIAL_IK_METHOD = SETTINGS.get("whole_body_ik.solver.method")
DEFAULT_PSEUDOINVERSE_RCOND = SETTINGS.number(
    "whole_body_ik.solver.pseudoinverse_rcond", positive=True
)
DEFAULT_DLS_DAMPING = SETTINGS.number("whole_body_ik.solver.dls_damping", positive=True)
BOUND_TOLERANCE = 1e-10


class IKMethod(str, Enum):
    """지원하는 velocity-level IK 해법."""

    PSEUDOINVERSE = "pseudoinverse"
    DLS = "dls"
    QP = "qp"

    @classmethod
    # coerce()는 IKMethod 열거형의 인스턴스를 반환한다. 문자열이나 다른 유형의 입력을 받아서 해당하는 IKMethod 인스턴스로 변환한다.
    # 만약 입력이 이미 IKMethod 인스턴스라면 그대로 반환하고, 문자열 입력에 대해서는 미리 정의된 별칭을 통해 적절한 IKMethod를 찾아 반환한다.
    # 만약 입력이 유효하지 않으면 ValueError를 발생시킨다.
    def coerce(cls, value):
        if isinstance(value, cls):
            return value
        # 별칭을 정의하여 다양한 문자열 입력을 IKMethod 열거형 값으로 매핑한다.
        aliases = {
            # PSEUDOINVERSE와 관련된 다양한 문자열 입력을 모두 PSEUDOINVERSE로 매핑한다.
            "pinv": cls.PSEUDOINVERSE,
            "pseudo_inverse": cls.PSEUDOINVERSE,
            "pseudoinverse": cls.PSEUDOINVERSE,
            # DLS와 관련된 다양한 문자열 입력을 모두 DLS로 매핑한다.
            "dls": cls.DLS,
            "damped_least_squares": cls.DLS,
            # QP와 관련된 다양한 문자열 입력을 모두 QP로 매핑한다.
            "qp": cls.QP,
            "quadratic_program": cls.QP,
        }
        key = str(value).strip().lower().replace("-", "_")
        try:
            return aliases[key]
        except KeyError as error:
            choices = ", ".join(method.value for method in cls)
            raise ValueError(
                f"unknown IK method {value!r}; expected one of: {choices}"
            ) from error


# DifferentialIKSolver 클래스는 pseudoinverse, DLS, QP와 같은 다양한 방법을 사용하여 bounded velocity IK 문제를 해결하는 기능을 제공한다.
# 먼저 pseudoinverse의 수식은 다음과 같다:
# dq = J^+ * dx
# 여기서 dq는 관절 속도 벡터, J^+는 Jacobian의 pseudoinverse, dx는 task space에서의 속도 벡터이다.
# DLS(Damped Least Squares) 방법은 pseudoinverse의 안정성을 향상시키기 위해 damping term을 추가한 방법으로, 수식은 다음과 같다:
# dq = (J^T * J + λ^2 * I)^-1 * J^T * dx
# 여기서 λ는 damping factor이며, I는 단위 행렬이다. DLS는 singularity 문제를 완화하고, 작은 singular value에 대한 민감성을 줄이는 데 유용하다.
# QP(Quadratic Programming) 방법은 task를 quadratic cost function으로 정의하고, 관절 속도에 대한 box constraints를 적용하여 최적화 문제를 해결하는 방법이다.
# QP는 다음과 같은 형태로 표현된다:
# minimize: 0.5 * ||J * dq - dx||^2
# subject to: lower <= dq <= upper
# 이때 free variables는 관절 속도 벡터 dq이며, lower와 upper는 각 관절 속도의 하한과 상한을 나타낸다.
# QP는 다양한 제약 조건을 포함할 수 있으며, 이를 통해 안전하고 효율적인 IK 솔루션을 제공한다.


class DifferentialIKSolver:
    """Pseudoinverse, DLS 또는 QP로 bounded velocity IK를 푼다."""

    # 여기서 bounded velocity는 로봇의 관절 속도가 특정 범위 내에 있어야 한다는 것을 의미한다.
    # 이 클래스는 주어진 task를 만족시키면서도 각 관절의 속도가 설정된 lower와 upper bounds 사이에 있도록 해주는 solver를 제공한다.

    def __init__(
        self,
        method=DEFAULT_DIFFERENTIAL_IK_METHOD,
        *,
        pseudoinverse_rcond=DEFAULT_PSEUDOINVERSE_RCOND,
        dls_damping=DEFAULT_DLS_DAMPING,
    ):
        self.method = IKMethod.coerce(method)
        self.pseudoinverse_rcond = float(pseudoinverse_rcond)
        self.dls_damping = float(dls_damping)
        if self.pseudoinverse_rcond <= 0.0 or self.dls_damping <= 0.0:
            raise ValueError("pseudoinverse rcond and DLS damping must be positive")

    # set_method() 메서드는 DifferentialIKSolver 클래스의 인스턴스에서 사용되는 IK 해법을 설정하는 역할을 한다.
    # 즉, pseudoinverse, DLS, QP 중에서 어떤 방법을 사용할지 지정할 수 있다.
    def set_method(self, method):
        self.method = IKMethod.coerce(method)

    # _solve_free() 메서드는 주어진 행렬과 벡터를 사용하여 자유 변수에 대한 해를 계산하는 역할을 한다.
    # 이 메서드는 pseudoinverse, DLS, QP와 같은 다양한 방법을 사용하여 자유 변수에 대한 해를 계산한다.
    # free variables는 제약 조건이 없는 변수들을 의미하며, 이 메서드는 이러한 변수들에 대한 최적의 해를 찾는 데 사용된다.
    # dq = (J^T * J + λ^2 * I)^-1 * J^T * b
    # 즉 목적함수는 최소제곱 문제를 해결하는 것이며, DLS 방법을 사용할 경우 damping term을 추가하여 안정성을 높인다.
    # pseudoinverse 방법을 사용할 경우, np.linalg.pinv()를 사용하여 pseudoinverse를 계산하고, 이를 통해 자유 변수에 대한 해를 구한다.
    # QP 방법을 사용할 경우, bounded_quadratic_program() 함수를 호출하여 제약 조건을 만족하는 최적의 해를 계산한다.
    # 다시 말해 이 함수는 제약이 없는 dls나 pseudoinverse 방법을 사용하여 자유 변수에 대한 해를 계산하는 역할을 한다.
    def _solve_free(self, matrix, vector):
        # 만약 입력된 행렬의 열 수가 0이면, 즉 자유 변수가 없으면, 0으로 채워진 벡터를 반환한다.
        if matrix.shape[1] == 0:
            return np.zeros(0)
        if self.method is IKMethod.PSEUDOINVERSE:
            return np.linalg.pinv(matrix, rcond=self.pseudoinverse_rcond) @ vector
        normal = matrix.T @ matrix
        normal.flat[:: normal.shape[0] + 1] += self.dls_damping**2
        return np.linalg.solve(normal, matrix.T @ vector)

    # _solve_with_bounds() 메서드는 주어진 행렬과 벡터, 하한 및 상한을 사용하여 제약 조건을 만족하는 해를 계산하는 역할을 한다.
    # 입력으론 matrix, vector, lower, upper가 들어오며, 이 메서드는 active set 방법을 사용하여 제약 조건을 만족하는 해를 찾는다.
    # matrix는 task space에서의 Jacobian 행렬,
    # vector는 task space에서의 속도 벡터, 즉 원하는 task를 나타낸다. 구하는 방법은 목표 task를 만족시키는 관절 속도 벡터 dq를 찾는 것이다.
    # lower와 upper는 각 관절 속도의 하한과 상한을 나타낸다.
    # 관절 속도의 하한과 상한의 데이터 타입은 float이며, 이 메서드는 제약 조건을 만족하는 관절 속도 벡터를 반환한다.
    def _solve_with_bounds(self, matrix, vector, lower, upper):
        """Active set으로 포화축을 고정하고 남은 축에 task를 재분배한다.
        # _solve_with_bounds() 메서드는 주어진 행렬과 벡터, 하한 및 상한을 사용하여
        # 제약 조건을 만족하는 해를 계산하는 역할을 한다."""
        # equality는 lower와 upper가 거의 같은 경우를 나타내며, 이 경우 해당 관절 속도는 고정되어야 한다.
        equality = upper - lower <= BOUND_TOLERANCE
        # solution은 초기 해를 나타내며, equality인 관절 속도는 lower와 upper의 평균값으로 설정된다.
        solution = np.zeros(matrix.shape[1], dtype=float)
        # equality인 관절 속도는 lower와 upper의 평균값으로 설정된다.
        solution[equality] = 0.5 * (lower[equality] + upper[equality])
        # active_lower와 active_upper는 각각 하한과 상한에 도달한 관절 속도를 나타내며, 초기에는 모두 False로 설정된다.
        active_lower = np.zeros(matrix.shape[1], dtype=bool)
        # active_upper는 각각 하한과 상한에 도달한 관절 속도를 나타내며, 초기에는 모두 False로 설정된다.
        active_upper = np.zeros(matrix.shape[1], dtype=bool)

        # 반복문은 최대 4 * matrix.shape[1] + 4번 반복되며, 각 반복에서 현재 해(solution)가 제약 조건을 만족하는지 확인하고,
        # 만족하지 않으면 active set을 업데이트하여 새로운 해를 계산한다.
        # 식은 다음과 같다:
        # 1. 현재 해(solution)가 제약 조건을 만족하는지 확인한다.
        # 2. 만족하지 않으면, active set을 업데이트하여 새로운 해를 계산한다.
        # 3. 새로운 해가 제약 조건을 만족하면, 해당 해를 반환한다.
        # minimize: 0.5 * ||J * dq - dx||^2
        # subject to: lower <= dq <= upper
        for _ in range(4 * matrix.shape[1] + 4):
            # active는 equality, active_lower, active_upper 중 하나라도 True인 관절 속도를 나타내며,
            # 즉, 활성화된 관절 속도를 나타낸다. 쉽게말해 이미 제약 조건에 의해 고정된 관절 속도를 나타낸다.
            # free는 active가 아닌 관절 속도를 나타낸다. 즉, free는 제약 조건이 없는 관절 속도를 나타낸다.
            # active set 방법은 현재 해(solution)가 제약 조건을 만족하지 않을 경우,
            # active set을 업데이트하여 새로운 해를 계산하는 방법이다.

            # active set 방법은 현재 해(solution)가 제약 조건을 만족하지 않을 경우, active set을 업데이트하여 새로운 해를 계산하는 방법이다.
            # 다음 아래에선 equality, active_lower, active_upper를 사용하여 현재 해(solution)가 제약 조건을 만족하는지 확인하고,
            # 만족하지 않으면 active set을 업데이트하여 새로운 해를 계산한다.
            active = equality | active_lower | active_upper
            free = ~active
            # 식은 다음과 같다:
            # 1. 현재 해(solution)가 제약 조건을 만족하는지 확인한다.
            # 2. 만족하지 않으면, active set을 업데이트하여 새로운 해를 계산한다.
            # 3. 새로운 해가 제약 조건을 만족하면, 해당 해를 반환한다.
            # residual은 현재 해(solution)가 제약 조건을 만족하는지 확인하기 위해 계산되는 벡터이다.
            # r = b - A * x
            # 여기서 r은 residual, b는 vector, A는 matrix, x는 solution
            # r = v - J * dq
            residual = vector - matrix[:, active] @ solution[active]
            # free variables에 대한 새로운 해(candidate)를 계산한다.
            candidate = solution.copy()
            # candidate[free]가 의미하는 것은 현재 해(solution)에서 자유 변수(free variables)에 대한 새로운 해를 계산하는 것이다.
            # 즉, 현재 해(solution)에서 제약 조건이 없는 관절 속도(free variables)에 대한 새로운 해를 계산하는 것이다.
            candidate[free] = self._solve_free(matrix[:, free], residual)
            # free variables에 대한 새로운 해(candidate)가 제약 조건을 만족하는지 확인한다.
            # below는 candidate가 lower bound보다 작은 경우를 나타내며, above는 candidate가 upper bound보다 큰 경우를 나타낸다.
            below = free & (candidate < lower - BOUND_TOLERANCE)
            above = free & (candidate > upper + BOUND_TOLERANCE)
            # 만약 candidate가 lower bound보다 작은 경우(below)나 upper bound보다 큰 경우(above)가 있다면,
            # violation을 계산하고, 가장 큰 violation을 가진 관절 속도를 active set에 추가한다.
            # violation은 현재 해(solution)가 제약 조건을 얼마나 위반하고 있는지를 나타내는 값으로, violation이 큰 관절 속도를 active set에 추가하여 새로운 해를 계산한다.
            if np.any(below | above):
                span = np.maximum(upper - lower, BOUND_TOLERANCE)
                violation = np.zeros_like(candidate)
                violation[below] = (lower[below] - candidate[below]) / span[below]
                violation[above] = (candidate[above] - upper[above]) / span[above]
                index = int(np.argmax(violation))
                if below[index]:
                    solution[index] = lower[index]
                    active_lower[index] = True
                else:
                    solution[index] = upper[index]
                    active_upper[index] = True
                continue
            # 만약 candidate가 제약 조건을 만족한다면, 현재 해(solution)를 candidate로 업데이트하고,
            # gradient를 계산하여 가장 큰 violation을 가진 관절 속도를 active set에 추가한다.
            # gradient는 현재 해(solution)가 제약 조건을 얼마나 위반하고 있는지를 나타내는 값으로,
            # gradient가 큰 관절 속도를 active set에 추가하여 새로운 해를 계산한다.
            # 수식은 다음과 같다:
            # gradient = J^T * (J * dq - dx)
            solution = candidate
            gradient = matrix.T @ (matrix @ solution - vector)
            # 만약 DLS 방법을 사용한다면, gradient에 damping term을 추가하여 안정성을 높인다.
            if self.method is IKMethod.DLS:
                gradient += self.dls_damping**2 * solution
            # active_lower와 active_upper는 각각 하한과 상한에 도달한 관절 속도를 나타내며,
            # gradient를 사용하여 가장 큰 violation을 가진 관절 속도를 active set에 추가한다.
            lower_violation = np.where(active_lower, -gradient, -np.inf)
            upper_violation = np.where(active_upper, gradient, -np.inf)
            lower_index = int(np.argmax(lower_violation))
            upper_index = int(np.argmax(upper_violation))
            worst_lower = lower_violation[lower_index]
            worst_upper = upper_violation[upper_index]
            if max(worst_lower, worst_upper) <= BOUND_TOLERANCE:
                return np.clip(solution, lower, upper)
            if worst_lower >= worst_upper:
                active_lower[lower_index] = False
            else:
                active_upper[upper_index] = False
        return np.clip(solution, lower, upper)

    # solve() 메서드는 DifferentialIKSolver 클래스의 인스턴스에서 사용되는 IK 해법을 설정하는 역할을 한다.
    # 즉, pseudoinverse, DLS, QP 중에서 어떤 방법을 사용할지 지정할 수 있다. 이 메서드는 주어진 행렬과 벡터,
    # 하한 및 상한을 사용하여 제약 조건을 만족하는 해를 계산하는 역할을 한다.
    def solve(self, matrix, vector, lower, upper):
        """Weighted task와 generalized-velocity box를 선택한 해법으로 푼다."""
        matrix = np.asarray(matrix, dtype=float)
        vector = np.asarray(vector, dtype=float)
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        if matrix.ndim != 2 or vector.shape != (matrix.shape[0],):
            raise ValueError("incompatible differential IK task shapes")
        if lower.shape != (matrix.shape[1],) or upper.shape != lower.shape:
            raise ValueError("incompatible differential IK bound shapes")
        if np.any(lower > upper):
            raise ValueError("differential IK lower bound exceeds upper bound")
        # 만약 선택한 해법이 QP라면, least_squares_to_qp() 함수를 사용하여 task를 quadratic cost function으로 변환하고,
        # bounded_quadratic_program() 함수를 호출하여 제약 조건을 만족하는 최적의 해를 계산한다.
        if self.method is IKMethod.QP:
            hessian, linear = least_squares_to_qp(matrix, vector)
            return bounded_quadratic_program(hessian, linear, lower, upper)
        return self._solve_with_bounds(matrix, vector, lower, upper)

    @staticmethod
    def enforce_constraints(
        reference,
        lower,
        upper,
        barrier_matrix=None,
        barrier_lower=None,
        barrier_weight=1.0,
        *,
        variable_scale=None,
        barrier_scale=1.0,
    ):
        """무차원 속도 공간에서 box와 collision soft-CBF를 적용한다."""
        reference = np.asarray(reference, dtype=float)
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        if reference.shape != lower.shape or upper.shape != lower.shape:
            raise ValueError("incompatible constraint projection shapes")
        if np.any(lower > upper):
            raise ValueError("constraint projection lower bound exceeds upper bound")
        if barrier_matrix is None:
            return np.clip(reference, lower, upper)
        if barrier_lower is None:
            raise ValueError("barrier_lower is required with barrier_matrix")

        scale = (
            np.ones_like(reference)
            if variable_scale is None
            else np.asarray(variable_scale, dtype=float)
        )
        barrier_scale = float(barrier_scale)
        if scale.shape != reference.shape:
            raise ValueError("incompatible constraint projection variable scale")
        if np.any(scale <= 0.0) or barrier_scale <= 0.0:
            raise ValueError("constraint projection scales must be positive")

        normalized_reference = reference / scale
        normalized_barrier = (
            np.asarray(barrier_matrix, dtype=float) * scale[None, :] / barrier_scale
        )
        normalized_lower = np.asarray(barrier_lower, dtype=float) / barrier_scale
        hessian, linear = least_squares_to_qp(
            np.eye(reference.size), normalized_reference
        )
        normalized_solution = bounded_quadratic_program_with_barriers(
            hessian,
            linear,
            lower / scale,
            upper / scale,
            normalized_barrier,
            normalized_lower,
            barrier_weight,
        )
        return normalized_solution * scale


__all__ = [
    "DEFAULT_DIFFERENTIAL_IK_METHOD",
    "DEFAULT_DLS_DAMPING",
    "DEFAULT_PSEUDOINVERSE_RCOND",
    "DifferentialIKSolver",
    "IKMethod",
]
