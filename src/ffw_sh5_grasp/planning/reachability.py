"""P7.0: 역-도달가능성 지도(inverse reachability map, IRM).

베이스 프레임 기준 상대 위치 격자마다 "고정-베이스 오른팔 IK로 이 지점에
닿을 수 있는가"를 오프라인으로 한 번 표로 만든다. 회전·이동에 불변이라
로봇 1대당 한 번만 만들면 되고, 나중에 어떤 베이스 자세에서도 그대로
재사용할 수 있다(월드 좌표를 베이스 프레임으로 변환하는 건 이 지도를
쓰는 쪽 — 예: P7.1 ``planning.base_pose``의 책임이다).

IK는 새로 만들지 않는다 — ``scripts/demo_plan_right_arm.py``의
``_ik_attempt``/``_solve_valid_ik``와 같은 패턴(여러 무작위 시드 중
수렴+무충돌인 첫 해를 채택하는 position-only DLS)을 그대로 재사용한다.
"""

from dataclasses import dataclass

import numpy as np

_CONVERGENCE_TOL_M = 0.003
_MAX_IK_ITER = 150
_JOINT_STEP_CLAMP_RAD = 0.1
_DAMPING_SQ = 0.05**2


@dataclass(frozen=True)
class ReachabilityMap:
    """베이스 프레임 기준 (dx, dy, dz) 격자별 IK 성공률."""

    grid_points: object  # (N, 3) np.ndarray — 베이스 프레임 상대 위치
    success_rate: object  # (N,) np.ndarray — 0.0~1.0

    def query(self, relative_xyz, *, k=8):
        """``relative_xyz``에서 가장 가까운 ``k``개 격자점의 역거리 가중 평균.

        질의점이 격자점과 정확히 같으면(거리 0) 그 격자점 값을 그대로
        반환한다. 격자가 ``k``개보다 작으면 있는 만큼만 쓴다.
        """
        relative_xyz = np.asarray(relative_xyz, dtype=float)
        distances = np.linalg.norm(self.grid_points - relative_xyz, axis=1)
        exact = np.flatnonzero(distances < 1e-9)
        if exact.size > 0:
            return float(self.success_rate[exact[0]])

        k = min(k, len(distances))
        nearest = np.argpartition(distances, k - 1)[:k]
        weights = 1.0 / distances[nearest]
        weights /= weights.sum()
        return float(np.dot(weights, self.success_rate[nearest]))


def default_grid(*, x_range=(-0.8, 0.8), y_range=(-1.1, 0.2), z_range=(0.5, 1.9), step=0.2):
    """실측(3000개 무작위 유효 표본의 FK 손끝 위치 1~99 백분위) 기반 기본 격자.

    베이스가 원점(``base_x=base_y=base_yaw=0``)인 ``home`` 키프레임에서
    측정한 도달 영역을 감싸는 경계상자를 ``step`` 간격으로 채운다 —
    x∈[-0.70, 0.72], y∈[-1.02, 0.08], z∈[0.54, 1.89]였던 실측값에 여유를
    둔 기본값이다.
    """
    xs = np.arange(x_range[0], x_range[1] + 1e-9, step)
    ys = np.arange(y_range[0], y_range[1] + 1e-9, step)
    zs = np.arange(z_range[0], z_range[1] + 1e-9, step)
    grid = np.stack(np.meshgrid(xs, ys, zs, indexing="ij"), axis=-1)
    return grid.reshape(-1, 3)


def _ik_attempt(solver, q_init, target_pos, context_qpos, q_reference):
    """position-only DLS + nullspace 정칙화 1회 시도.

    ``scripts/demo_plan_right_arm.py``의 ``_ik_attempt``와 동일한 계약
    (수렴 허용오차, 감쇠, nullspace gain)을 쓴다 — 새 IK 알고리즘이 아니다.
    """
    n = solver.n
    q = np.clip(q_init, solver.joint_ranges[:, 0], solver.joint_ranges[:, 1])
    for _ in range(_MAX_IK_ITER):
        state = solver.forward(q, context_qpos)
        position_error = target_pos - state.position
        if float(np.linalg.norm(position_error)) < _CONVERGENCE_TOL_M:
            return q, True
        jacobian = state.jacobian[:3]
        gram = jacobian @ jacobian.T + _DAMPING_SQ * np.eye(3)
        pseudo_inverse = jacobian.T @ np.linalg.inv(gram)
        primary = pseudo_inverse @ position_error
        nullspace_projector = np.eye(n) - pseudo_inverse @ jacobian
        secondary = 0.2 * (q_reference - q)
        delta = primary + nullspace_projector @ secondary
        delta = np.clip(delta, -_JOINT_STEP_CLAMP_RAD, _JOINT_STEP_CLAMP_RAD)
        q = np.clip(q + delta, solver.joint_ranges[:, 0], solver.joint_ranges[:, 1])
    return q, False


def _probe(solver, checker, space, target_pos, context_qpos, rng, *, n_restarts):
    """``_solve_valid_ik``와 같은 패턴 — 수렴+무충돌인 첫 해가 나오면 성공."""
    candidates = [rng.uniform(solver.joint_ranges[:, 0], solver.joint_ranges[:, 1])
                  for _ in range(n_restarts)]
    for candidate in candidates:
        q, converged = _ik_attempt(solver, candidate, target_pos, context_qpos, candidate)
        if converged and checker.is_valid(q):
            return True
    return False


def build_reachability_map(solver, checker, space, rng, *, grid=None, n_restarts=10):
    """``grid``(기본 ``default_grid()``)의 각 점마다 IK 도달성을 확률로 기록한다.

    ``grid``는 베이스 프레임 기준 (dx, dy, dz)다 — ``checker``/``solver``를
    베이스가 원점(``home`` 키프레임)인 장면으로 만들어 호출해야 결과가
    실제로 베이스 프레임 기준이 된다(호출자 책임, 이 함수는 검증하지 않는다).
    각 격자점을 한 번의 IK 성공/실패(0.0/1.0)로 기록한다 — 여러 시드를 이미
    ``_probe``가 소진하므로 "성공률"은 사실상 이 seed에서의 도달 가능
    여부다. 더 정밀한 통계가 필요하면 호출자가 여러 rng로 반복 호출해
    평균을 내면 된다(이 함수는 단일 실행만 책임진다).
    """
    if grid is None:
        grid = default_grid()
    grid = np.asarray(grid, dtype=float)
    context_qpos = checker.snapshot_qpos
    success = np.empty(len(grid), dtype=float)
    for index, offset in enumerate(grid):
        reached = _probe(solver, checker, space, offset, context_qpos, rng, n_restarts=n_restarts)
        success[index] = 1.0 if reached else 0.0
    return ReachabilityMap(grid_points=grid, success_rate=success)


__all__ = ["ReachabilityMap", "build_reachability_map", "default_grid"]
