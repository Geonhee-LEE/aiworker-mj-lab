"""두 configuration 사이 관절공간 선분의 충돌 유효성 검사.

순차적으로 한쪽 끝에서부터 검사하면 충돌 선분을 늦게 발견한다. 중점부터
이분(bisection) 순서로 검사하면 평균적으로 훨씬 빨리 기각할 수 있다.
"""

import numpy as np


class EdgeChecker:
    """관절공간 선분을 ``resolution_rad`` 간격으로 나눠 검사한다."""

    def __init__(self, space, is_valid, *, resolution_rad):
        if resolution_rad <= 0.0:
            raise ValueError("resolution_rad는 양수여야 합니다")
        self.space = space
        self.is_valid = is_valid
        self.resolution_rad = float(resolution_rad)

    def steps(self, a, b):
        """``a``-``b`` 구간을 검사할 표본 개수(양 끝 제외 내부 분할 수)."""
        distance = self.space.max_component(a, b)
        return max(1, int(np.ceil(distance / self.resolution_rad)))

    def _bisection_order(self, count):
        """중점부터 검사하는 순서의 fraction 목록(0과 1은 제외)."""
        if count <= 1:
            return [0.5]
        order = [0.5]
        # 이분 트리 순서: [0,1] 구간을 반복해서 절반으로 나눈다.
        queue = [(0.0, 1.0)]
        seen = {0.5}
        while len(order) < count:
            lo, hi = queue.pop(0)
            mid = (lo + hi) / 2.0
            for candidate in ((lo + mid) / 2.0, (mid + hi) / 2.0):
                if candidate not in seen and len(order) < count:
                    seen.add(candidate)
                    order.append(candidate)
            queue.append((lo, mid))
            queue.append((mid, hi))
        return order[:count]

    def is_valid_edge(self, a, b, *, check_endpoints=True):
        """``a``-``b`` 구간이 전부 유효하면 ``True``."""
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        if check_endpoints and not (self.is_valid(a) and self.is_valid(b)):
            return False
        count = self.steps(a, b)
        for fraction in self._bisection_order(count):
            point = self.space.interpolate(a, b, fraction)
            if not self.is_valid(point):
                return False
        return True

    def last_valid(self, a, b):
        """``a``에서 시작해 ``b``방향으로 유효한 마지막 지점과 그 fraction."""
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        if not self.is_valid(a):
            return a.copy(), 0.0
        count = self.steps(a, b)
        best_point, best_fraction = a.copy(), 0.0
        for step in range(1, count + 1):
            fraction = step / count
            point = self.space.interpolate(a, b, fraction)
            if not self.is_valid(point):
                break
            best_point, best_fraction = point, fraction
        return best_point, best_fraction


__all__ = ["EdgeChecker"]
