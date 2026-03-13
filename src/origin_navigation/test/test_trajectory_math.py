"""Unit tests for the trajectory math helpers."""

import math
import pathlib
import sys

sys.path.insert(
    0,
    str(pathlib.Path(__file__).resolve().parents[1]),
)

from origin_navigation.trajectory_math import (
    generate_timed_trajectory,
    smooth_waypoints,
    target_speed_from_trajectory,
)


def test_smooth_waypoints_preserves_open_path_endpoints():
    """Open-path smoothing should keep the original start and end points."""

    waypoints = [
        (0.0, 0.0),
        (1.0, 0.2),
        (2.0, 0.0),
        (3.0, -0.1),
    ]

    smoothed = smooth_waypoints(
        waypoints,
        sample_spacing=0.1,
        spline_smoothing=0.01,
        closed_path=False,
    )

    assert len(smoothed) > len(waypoints)
    assert math.isclose(smoothed[0][0], waypoints[0][0], abs_tol=1e-6)
    assert math.isclose(smoothed[0][1], waypoints[0][1], abs_tol=1e-6)
    assert math.isclose(smoothed[-1][0], waypoints[-1][0], abs_tol=1e-6)
    assert math.isclose(smoothed[-1][1], waypoints[-1][1], abs_tol=1e-6)


def test_smooth_waypoints_returns_dense_closed_loop():
    """Closed-path smoothing should add interpolation points."""

    waypoints = [
        (1.0, 0.0),
        (0.0, 1.0),
        (-1.0, 0.0),
        (0.0, -1.0),
    ]

    smoothed = smooth_waypoints(
        waypoints,
        sample_spacing=0.2,
        spline_smoothing=0.0,
        closed_path=True,
    )

    assert len(smoothed) > len(waypoints)


def test_generate_timed_trajectory_is_monotonic_for_open_path():
    """Open-path timestamps should increase monotonically."""

    points = [
        (0.0, 0.0),
        (0.5, 0.0),
        (1.0, 0.0),
        (1.5, 0.0),
    ]

    trajectory = generate_timed_trajectory(
        points,
        cruise_speed=0.5,
        acceleration=0.5,
        closed_path=False,
    )

    times = [point['time_from_start'] for point in trajectory]

    assert times[0] == 0.0
    assert times == sorted(times)
    assert trajectory[-1]['time_from_start'] > 0.0


def test_target_speed_matches_constant_speed_closed_loop():
    """Closed-loop timing should recover the requested cruise speed."""

    points = [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
        (3.0, 0.0),
    ]

    trajectory = generate_timed_trajectory(
        points,
        cruise_speed=1.0,
        acceleration=0.5,
        closed_path=True,
    )

    speed = target_speed_from_trajectory(trajectory, 1, closed_path=True)
    assert math.isclose(speed, 1.0, rel_tol=1e-6)


def test_generate_timed_trajectory_closed_loop_uses_trapezoidal_profile():
    """Closed loops should honor the trapezoidal timing option."""

    points = [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
        (3.0, 0.0),
        (4.0, 0.0),
    ]

    trajectory = generate_timed_trajectory(
        points,
        cruise_speed=1.0,
        acceleration=1.0,
        velocity_profile='trapezoidal',
        closed_path=True,
    )

    speeds = [point['desired_speed'] for point in trajectory]
    times = [point['time_from_start'] for point in trajectory]

    assert times[0] == 0.0
    assert times == sorted(times)
    assert math.isclose(speeds[0], 0.0, abs_tol=1e-9)
    assert math.isclose(speeds[-1], 0.0, abs_tol=1e-9)
    assert max(speeds) <= 1.0 + 1e-9
    assert any(
        not math.isclose(speed, 1.0, rel_tol=1e-6, abs_tol=1e-6)
        for speed in speeds
    )
