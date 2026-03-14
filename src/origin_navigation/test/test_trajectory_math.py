"""Unit tests for the trajectory math helpers."""

import importlib
import math
import pathlib
import sys

sys.path.insert(
    0,
    str(pathlib.Path(__file__).resolve().parents[1]),
)

trajectory_math = importlib.import_module('origin_navigation.trajectory_math')

generate_timed_trajectory = trajectory_math.generate_timed_trajectory
smooth_waypoints = trajectory_math.smooth_waypoints


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
    )

    assert len(smoothed) > len(waypoints)


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
