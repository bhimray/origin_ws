"""
Provide shared geometry, heading, smoothing, and timing utilities.

This module contains helpers for angle and quaternion conversion, waypoint
distance accumulation, heading estimation, spline-based path smoothing, and
trapezoidal timing calculations used to build the reference trajectory.
"""

import math

import numpy as np
from scipy.interpolate import splev, splprep


def normalize_angle(angle):

    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_yaw(yaw):

    half_yaw = yaw * 0.5
    return (
        0.0,
        0.0,
        math.sin(half_yaw),
        math.cos(half_yaw),
    )


def quaternion_to_yaw(quaternion):

    siny_cosp = (
        2.0 *
        (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    )
    cosy_cosp = (
        1.0 -
        2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    )
    return math.atan2(siny_cosp, cosy_cosp)


def pairwise_distances(points):

    if len(points) < 2:
        return []

    distances = []

    for index in range(1, len(points)):
        dx = points[index][0] - points[index - 1][0]
        dy = points[index][1] - points[index - 1][1]
        distances.append(math.hypot(dx, dy))

    return distances


def cumulative_distances(points):

    cumulative = [0.0]

    for distance in pairwise_distances(points):
        cumulative.append(cumulative[-1] + distance)

    return cumulative


def estimate_headings(points):

    if not points:
        return []

    headings = []
    total_points = len(points)

    for index, (x_pos, y_pos) in enumerate(points):
        if total_points == 1:
            headings.append(0.0)
            continue

        prev_index = (index - 1) % total_points
        next_index = (index + 1) % total_points

        prev_x, prev_y = points[prev_index]
        next_x, next_y = points[next_index]

        if (
            math.isclose(next_x, prev_x, abs_tol=1e-9) and
            math.isclose(next_y, prev_y, abs_tol=1e-9)
        ):
            if headings:
                headings.append(headings[-1])
            else:
                headings.append(0.0)
            continue

        headings.append(math.atan2(next_y - prev_y, next_x - prev_x))

    return headings


def smooth_waypoints(
    waypoints,
    sample_spacing=0.05,
    spline_smoothing=0.0,
):

    if len(waypoints) < 2:
        return list(waypoints)

    if len(waypoints) == 2:
        return list(waypoints)

    x_points = [point[0] for point in waypoints]
    y_points = [point[1] for point in waypoints]

    spline_degree = min(3, len(waypoints) - 1)
    tck, _ = splprep(
        [x_points, y_points],
        s=spline_smoothing,
        per=True,
        k=spline_degree,
    )

    dense_samples = max(200, len(waypoints) * 25)
    dense_u = np.linspace(0.0, 1.0, dense_samples, endpoint=False)
    dense_x, dense_y = splev(dense_u, tck)
    dense_points = list(zip(dense_x, dense_y))

    arc_lengths = cumulative_distances(dense_points)
    total_length = arc_lengths[-1]

    if total_length < 1e-6:
        return [(float(x_points[0]), float(y_points[0]))]

    sample_count = max(2, int(math.ceil(total_length / sample_spacing)))
    target_distances = np.linspace(
        0.0,
        total_length,
        sample_count,
        endpoint=False,
    )

    resampled_x = np.interp(target_distances, arc_lengths, dense_x)
    resampled_y = np.interp(target_distances, arc_lengths, dense_y)
    return [
        (float(x_pos), float(y_pos))
        for x_pos, y_pos in zip(resampled_x, resampled_y)
    ]


def trapezoidal_time_profile(path_length, cruise_speed, acceleration):

    if path_length <= 0.0:
        return 0.0, 0.0, 0.0, 0.0

    accel_distance = (cruise_speed * cruise_speed) / (2.0 * acceleration)

    if 2.0 * accel_distance <= path_length:
        accel_time = cruise_speed / acceleration
        cruise_distance = path_length - 2.0 * accel_distance
        cruise_time = cruise_distance / cruise_speed
        total_time = 2.0 * accel_time + cruise_time
    else:
        accel_time = math.sqrt(path_length * acceleration) / acceleration
        cruise_distance = 0.0
        cruise_time = 0.0
        total_time = 2.0 * accel_time

    return accel_time, cruise_time, cruise_distance, total_time


def trapezoidal_speed_at_distance(
    distance,
    path_length,
    cruise_speed,
    acceleration,
):

    accel_time, _, cruise_distance, _ = trapezoidal_time_profile(
        path_length,
        cruise_speed,
        acceleration,
    )

    if path_length <= 0.0:
        return 0.0

    accel_distance = 0.5 * acceleration * accel_time * accel_time
    decel_start = accel_distance + cruise_distance
    peak_speed = acceleration * accel_time

    if distance <= accel_distance:
        return min(peak_speed, math.sqrt(2.0 * acceleration * distance))

    if distance <= decel_start:
        return peak_speed

    remaining_distance = max(path_length - distance, 0.0)
    return min(
        peak_speed,
        math.sqrt(2.0 * acceleration * remaining_distance),
    )


def time_at_distance(distance, path_length, cruise_speed, acceleration):

    accel_time, _, cruise_distance, total_time = (
        trapezoidal_time_profile(path_length, cruise_speed, acceleration)
    )

    if path_length <= 0.0:
        return 0.0

    peak_speed = acceleration * accel_time
    accel_distance = 0.5 * acceleration * accel_time * accel_time
    decel_start = accel_distance + cruise_distance

    if distance <= accel_distance:
        return math.sqrt((2.0 * distance) / acceleration)

    if distance <= decel_start:
        cruise_distance_traveled = distance - accel_distance
        return accel_time + (cruise_distance_traveled / peak_speed)

    remaining_distance = path_length - distance
    decel_time = math.sqrt((2.0 * max(remaining_distance, 0.0)) / acceleration)
    return max(0.0, total_time - decel_time)


def generate_timed_trajectory(
    points,
    cruise_speed=0.3,
    acceleration=0.4,
):

    if not points:
        return []

    distances = cumulative_distances(points)
    path_length = distances[-1]
    headings = estimate_headings(points)

    trajectory = []

    for index, (point, distance, heading) in enumerate(
        zip(points, distances, headings)
    ):
        time_from_start = time_at_distance(
            distance,
            path_length,
            cruise_speed,
            acceleration,
        )
        desired_speed = trapezoidal_speed_at_distance(
            distance,
            path_length,
            cruise_speed,
            acceleration,
        )

        trajectory.append({
            'index': index,
            'x': point[0],
            'y': point[1],
            'heading': heading,
            'time_from_start': time_from_start,
            'desired_speed': desired_speed,
        })

    return trajectory
