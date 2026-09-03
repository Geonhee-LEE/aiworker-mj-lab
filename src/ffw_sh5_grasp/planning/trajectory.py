"""경로 waypoint를 제어 주기 간격의 시간 표본으로 바꾸는 사다리꼴 속도 프로파일.

각 세그먼트(연속한 두 waypoint 사이)를 독립적으로 사다리꼴(짧으면 삼각형)
프로파일로 시간 파라미터화하고, 매 waypoint에서 속도가 정확히 0으로
돌아온 뒤 다음 세그먼트를 시작한다 — moveit 계열
``IterativeParabolicTimeParameterization``과 같은 표준 접근이다
(``research/2026-08/001.md`` 조사). 이 저장소는 모든 관절에 같은 스칼라
속도·가속도 상한을 쓰므로, "여러 관절 중 가장 오래 걸리는 관절이 세그먼트
시간을 결정한다"는 동기화 규칙은 정확히 "최대 성분(Linf) 거리를 세그먼트
길이로 쓰는 사다리꼴 프로파일"과 같아진다 — 관절별 변위 비율이 병목 관절보다
작으면 속도·가속도도 비례해서 작아지기 때문이다.

세그먼트를 하나로 이어붙여 웨이포인트마다 멈추지 않는 전역 프로파일도
시도했으나, 인접한 두 세그먼트의 방향이 다르면 그 경계에서 관절 속도가
불연속으로 바뀌어(방향 전환) 가속도가 무한대에 가까워진다 — 촘촘한 RRT
원경로에서 실측으로 확인된 버그다. 매 waypoint에서 완전히 멈추면 그
불연속이 항상 "0에서 0으로"가 되어 사라진다. 멈추지 않고 매끄럽게 지나가려면
코너 blending(참고 자료의 "Part 3")이 필요한데, 이는 MP-0006 범위 밖이다.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Trajectory:
    """제어 주기 간격의 시간 표본 경로.

    ``positions[i]``는 ``times[i]``(초, ``times[0] == 0.0``에서 단조 증가)의
    목표 관절각이다.
    """

    times: object  # (M,) np.ndarray
    positions: object  # (M, n) np.ndarray


def _trapezoid_profile(distance, max_speed_rad_s, max_accel_rad_s2):
    """길이 ``distance``인 단일 세그먼트의 사다리꼴(또는 삼각형) 프로파일.

    ``(segment_time, s_of_t)``를 반환한다. ``s_of_t(t)``는 세그먼트 시작
    기준 경과 시간 ``t``에서 0으로 시작해 ``distance``까지 단조 증가하는
    path parameter다.
    """
    accel_time = max_speed_rad_s / max_accel_rad_s2
    accel_dist = 0.5 * max_accel_rad_s2 * accel_time**2

    if 2.0 * accel_dist <= distance:
        cruise_dist = distance - 2.0 * accel_dist
        cruise_time = cruise_dist / max_speed_rad_s
        peak_speed = max_speed_rad_s
    else:
        # 세그먼트가 최고 속도에 도달하기엔 짧다 — 삼각형 프로파일로 축소.
        accel_time = float(np.sqrt(distance / max_accel_rad_s2))
        accel_dist = 0.5 * max_accel_rad_s2 * accel_time**2
        cruise_time = 0.0
        peak_speed = max_accel_rad_s2 * accel_time

    segment_time = 2.0 * accel_time + cruise_time

    def s_of_t(t):
        if t <= accel_time:
            return 0.5 * max_accel_rad_s2 * t**2
        if t <= accel_time + cruise_time:
            return accel_dist + peak_speed * (t - accel_time)
        remaining = segment_time - t
        return distance - 0.5 * max_accel_rad_s2 * remaining**2

    return segment_time, s_of_t


def time_parameterize(space, path, *, max_speed_rad_s, max_accel_rad_s2, control_period_s):
    """``path``((K, n) waypoint 배열)를 사다리꼴 속도 프로파일로 시간 파라미터화한다.

    반환하는 ``Trajectory``는 항상 ``times[0] == 0.0``에 ``path[0]``,
    마지막 표본에 정확히 ``path[-1]``을 포함한다. 길이가 0인 세그먼트(중복
    waypoint)는 건너뛴다. 모든 세그먼트 안에서 각 관절의 순간 속도·가속도는
    세그먼트 길이를 Linf로 재기 때문에 ``max_speed_rad_s``/
    ``max_accel_rad_s2``를 넘지 않는다(모듈 독스트링 참고).
    """
    if max_speed_rad_s <= 0.0:
        raise ValueError("max_speed_rad_s는 양수여야 합니다")
    if max_accel_rad_s2 <= 0.0:
        raise ValueError("max_accel_rad_s2는 양수여야 합니다")
    if control_period_s <= 0.0:
        raise ValueError("control_period_s는 양수여야 합니다")
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[0] < 1:
        raise ValueError("path는 (K, n) 배열이어야 하며 K >= 1이어야 합니다")

    times = [0.0]
    positions = [path[0].copy()]
    elapsed = 0.0

    for i in range(len(path) - 1):
        distance = space.max_component(path[i], path[i + 1])
        if distance <= 0.0:
            continue
        segment_time, s_of_t = _trapezoid_profile(distance, max_speed_rad_s, max_accel_rad_s2)
        num_steps = int(np.floor(segment_time / control_period_s + 1e-9))
        local_times = np.arange(1, num_steps + 1) * control_period_s
        local_times = local_times[local_times < segment_time - 1e-9]
        local_times = np.append(local_times, segment_time)
        for t in local_times:
            fraction = s_of_t(t) / distance
            positions.append(space.interpolate(path[i], path[i + 1], fraction))
            times.append(elapsed + t)
        elapsed += segment_time

    positions[-1] = path[-1]  # 부동소수 누적 오차 없이 목표를 정확히 보존
    return Trajectory(np.array(times), np.stack(positions))


__all__ = ["Trajectory", "time_parameterize"]
