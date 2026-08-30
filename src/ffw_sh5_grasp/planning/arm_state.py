"""오른팔 7자유도 관절 공간 추상화.

``RightArmSpace``는 관절 이름·id·qpos 주소·범위를 한 번 계산해 보관하고, 샘플링·
보간·클리핑처럼 플래너가 반복적으로 쓰는 연산을 제공한다. 관절 이름 상수는 이
모듈에서 자체 정의한다 — ``application.teleop``(GLFW를 import한다)이나
``imitation.data.schema``를 재사용하면 계층 경계가 꼬이기 때문이다.
"""

import mujoco
import numpy as np

RIGHT_ARM_JOINTS = tuple(f"arm_r_joint{i}" for i in range(1, 8))


class RightArmSpace:
    """오른팔 7-DOF 관절 공간의 이름·주소·범위와 샘플링/보간 연산."""

    def __init__(self, joint_names, joint_ids, qpos_adrs, dof_ids, lower, upper, limited):
        self.joint_names = tuple(joint_names)
        self.n = len(self.joint_names)
        self.joint_ids = np.asarray(joint_ids, dtype=int)
        self.qpos_adrs = np.asarray(qpos_adrs, dtype=int)
        self.dof_ids = np.asarray(dof_ids, dtype=int)
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        self.limited = np.asarray(limited, dtype=bool)
        if not (
            self.joint_ids.shape
            == self.qpos_adrs.shape
            == self.dof_ids.shape
            == self.lower.shape
            == self.upper.shape
            == self.limited.shape
            == (self.n,)
        ):
            raise ValueError("RightArmSpace의 모든 배열은 길이 n이어야 합니다")

    @classmethod
    def from_model(cls, model, joint_names=RIGHT_ARM_JOINTS):
        """MuJoCo 모델에서 관절 주소와 범위를 읽어 생성한다."""
        joint_names = tuple(joint_names)
        joint_ids = []
        for name in joint_names:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise ValueError(f"모델에 관절이 없습니다: {name!r}")
            joint_ids.append(joint_id)
        joint_ids = np.asarray(joint_ids, dtype=int)
        qpos_adrs = model.jnt_qposadr[joint_ids]
        dof_ids = model.jnt_dofadr[joint_ids]
        ranges = model.jnt_range[joint_ids]
        limited = model.jnt_limited[joint_ids].astype(bool)
        return cls(
            joint_names,
            joint_ids,
            qpos_adrs,
            dof_ids,
            ranges[:, 0],
            ranges[:, 1],
            limited,
        )

    @classmethod
    def from_limits(cls, lower, upper, joint_names=None):
        """MuJoCo 없이 순수 범위 배열만으로 생성한다 (단위 테스트용)."""
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        if lower.shape != upper.shape or lower.ndim != 1:
            raise ValueError("lower와 upper는 같은 길이의 1차원 배열이어야 합니다")
        n = lower.shape[0]
        names = tuple(joint_names) if joint_names else tuple(f"joint{i}" for i in range(n))
        indices = np.arange(n, dtype=int)
        return cls(names, indices, indices, indices, lower, upper, np.ones(n, dtype=bool))

    def clip(self, q):
        """범위가 있는 자유도만 모델 범위로 자른 새 벡터를 반환한다."""
        q = np.asarray(q, dtype=float).copy()
        q[self.limited] = np.clip(
            q[self.limited], self.lower[self.limited], self.upper[self.limited]
        )
        return q

    def contains(self, q, *, tolerance=0.0):
        """``q``가 관절 범위 안에 있는지 판정한다."""
        q = np.asarray(q, dtype=float)
        if q.shape != (self.n,):
            return False
        within = np.ones(self.n, dtype=bool)
        within[self.limited] = (q[self.limited] >= self.lower[self.limited] - tolerance) & (
            q[self.limited] <= self.upper[self.limited] + tolerance
        )
        return bool(np.all(within))

    def sample(self, rng):
        """``rng``(``np.random.Generator``)로 범위 안에서 균등 표본을 뽑는다."""
        q = rng.uniform(self.lower, self.upper)
        # 범위가 없는 자유도(현재 없음)는 uniform이 정의되지 않으므로 0으로 둔다.
        q[~self.limited] = 0.0
        return q

    def distance(self, a, b):
        """두 configuration 사이의 L2 거리(rad)."""
        return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))

    def max_component(self, a, b):
        """두 configuration 사이의 최대 성분 변화량(rad). 선분 세분 기준에 쓴다."""
        return float(np.max(np.abs(np.asarray(a, dtype=float) - np.asarray(b, dtype=float))))

    def interpolate(self, a, b, fraction):
        """``a``에서 ``b``로 ``fraction``(0..1) 만큼 선형 보간한다."""
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        return a + fraction * (b - a)

    def steer(self, source, target, max_step_rad):
        """``source``에서 ``target``방향으로 최대 ``max_step_rad``만큼 나아간 점."""
        source = np.asarray(source, dtype=float)
        target = np.asarray(target, dtype=float)
        delta = target - source
        distance = float(np.linalg.norm(delta))
        if distance <= max_step_rad or distance < 1e-12:
            return target.copy()
        return source + delta * (max_step_rad / distance)

    def write(self, qpos, q):
        """``qpos``(전체 nq 길이 배열)의 오른팔 주소에 ``q``를 쓰고 반환한다."""
        qpos = np.asarray(qpos, dtype=float)
        qpos[self.qpos_adrs] = np.asarray(q, dtype=float)
        return qpos


__all__ = ["RIGHT_ARM_JOINTS", "RightArmSpace"]
