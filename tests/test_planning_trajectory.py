"""``time_parameterize``(사다리꼴 속도 프로파일)의 순수 numpy 성질 시험.

MuJoCo가 필요 없어 빠르게 돈다.

Headless 단독 실행: ``python3 tests/test_planning_trajectory.py``
"""

import pathlib
import sys

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from ffw_sh5_grasp.planning.arm_state import RightArmSpace  # noqa: E402
from ffw_sh5_grasp.planning.trajectory import time_parameterize  # noqa: E402

MAX_SPEED = 2.0
MAX_ACCEL = 4.0
DT = 0.02
TOLERANCE = 1e-6


def _space(n=7):
    return RightArmSpace.from_limits(np.full(n, -10.0), np.full(n, 10.0))


def _parameterize(path, **overrides):
    kwargs = dict(
        max_speed_rad_s=MAX_SPEED, max_accel_rad_s2=MAX_ACCEL, control_period_s=DT
    )
    kwargs.update(overrides)
    return time_parameterize(_space(path.shape[1]), path, **kwargs)


def _velocity_and_accel_within_bounds(trajectory):
    times, positions = trajectory.times, trajectory.positions
    dt = np.diff(times)
    assert np.all(dt > 0.0), "시간이 단조 증가해야 합니다"
    velocities = np.diff(positions, axis=0) / dt[:, None]
    assert np.all(np.abs(velocities) <= MAX_SPEED + TOLERANCE), velocities.max()
    if len(velocities) > 1:
        accel_dt = 0.5 * (dt[:-1] + dt[1:])
        accelerations = np.diff(velocities, axis=0) / accel_dt[:, None]
        assert np.all(np.abs(accelerations) <= MAX_ACCEL + TOLERANCE), accelerations.max()


def test_endpoints_are_preserved():
    path = np.array([[0.0] * 7, [1.0] * 7, [3.0] + [0.0] * 6])
    trajectory = _parameterize(path)
    assert trajectory.times[0] == 0.0
    assert np.array_equal(trajectory.positions[0], path[0])
    assert np.array_equal(trajectory.positions[-1], path[-1])


def test_times_start_at_zero_and_increase():
    path = np.array([[0.0] * 7, [2.0] + [0.0] * 6, [2.0, 2.0] + [0.0] * 5])
    trajectory = _parameterize(path)
    assert trajectory.times[0] == 0.0
    assert np.all(np.diff(trajectory.times) > 0.0)


def test_velocity_and_acceleration_respect_bounds_trapezoid_case():
    # 충분히 긴 경로 -> 정속 구간이 생기는 완전한 사다리꼴.
    path = np.array([[0.0] * 7, [5.0] + [0.0] * 6])
    trajectory = _parameterize(path)
    _velocity_and_accel_within_bounds(trajectory)


def test_velocity_and_acceleration_respect_bounds_triangular_case():
    # 짧은 경로 -> 최고 속도에 못 미치고 감속하는 삼각형 프로파일.
    path = np.array([[0.0] * 7, [0.3] + [0.0] * 6])
    trajectory = _parameterize(path)
    _velocity_and_accel_within_bounds(trajectory)


def test_multi_waypoint_path_respects_bounds_and_visits_all_joints():
    rng = np.random.default_rng(0)
    path = np.concatenate([[np.zeros(7)], np.cumsum(rng.uniform(-1.0, 1.0, size=(6, 7)), axis=0)])
    trajectory = _parameterize(path)
    _velocity_and_accel_within_bounds(trajectory)
    assert np.array_equal(trajectory.positions[0], path[0])
    assert np.array_equal(trajectory.positions[-1], path[-1])


def test_bottleneck_joint_reaches_max_speed_others_scale_down():
    # joint0은 4rad 움직이고 joint1은 그 절반(2rad)만 움직인다 -> 두 관절
    # 모두 같은 시간에 도착해야 하며, joint1의 피크 속도는 joint0의 정확히
    # 절반이어야 한다(같은 path parameter에 비례해 스케일되므로).
    path = np.array([[0.0, 0.0] + [0.0] * 5, [4.0, 2.0] + [0.0] * 5])
    trajectory = _parameterize(path)
    velocities = np.diff(trajectory.positions, axis=0) / np.diff(trajectory.times)[:, None]
    peak0 = np.max(np.abs(velocities[:, 0]))
    peak1 = np.max(np.abs(velocities[:, 1]))
    assert np.isclose(peak0, MAX_SPEED, atol=1e-2)
    assert np.isclose(peak1, MAX_SPEED / 2.0, atol=1e-2)


def test_single_waypoint_path_returns_single_sample():
    path = np.array([[1.0] * 7])
    trajectory = _parameterize(path)
    assert trajectory.times.shape == (1,)
    assert np.array_equal(trajectory.positions[0], path[0])


def test_zero_length_path_collapses_to_single_sample():
    path = np.array([[1.0] * 7, [1.0] * 7, [1.0] * 7])
    trajectory = _parameterize(path)
    assert trajectory.times.shape == (1,)
    assert np.array_equal(trajectory.positions[0], path[0])


def test_deterministic_for_same_input():
    path = np.array([[0.0] * 7, [1.5] + [0.0] * 6, [1.5, 1.0] + [0.0] * 5])
    first = _parameterize(path)
    second = _parameterize(path)
    assert np.array_equal(first.times, second.times)
    assert np.array_equal(first.positions, second.positions)


def test_rejects_non_positive_limits():
    path = np.array([[0.0] * 7, [1.0] * 7])
    space = _space()
    for bad_kwargs in (
        dict(max_speed_rad_s=0.0, max_accel_rad_s2=MAX_ACCEL, control_period_s=DT),
        dict(max_speed_rad_s=MAX_SPEED, max_accel_rad_s2=-1.0, control_period_s=DT),
        dict(max_speed_rad_s=MAX_SPEED, max_accel_rad_s2=MAX_ACCEL, control_period_s=0.0),
    ):
        try:
            time_parameterize(space, path, **bad_kwargs)
        except ValueError:
            continue
        raise AssertionError(f"거부되어야 할 입력이 통과했습니다: {bad_kwargs}")


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("PASS")


if __name__ == "__main__":
    main()
