import math


TEST_TRACK_NUM_POINTS = 36


def generate_test_track_waypoints(num_points=TEST_TRACK_NUM_POINTS):

    waypoints = []

    for i in range(num_points):
        theta = 2.0 * math.pi * i / num_points
        radius = (
            2.10 +
            0.75 * math.cos(2.0 * theta - 0.2) +
            0.28 * math.sin(5.0 * theta) -
            0.18 * math.cos(7.0 * theta + 0.4)
        )
        x = radius * math.cos(theta)
        y = 0.72 * radius * math.sin(theta)
        waypoints.append((x, y))

    return waypoints


def shift_waypoints_to_origin(waypoints):

    if not waypoints:
        return []

    origin_x, origin_y = waypoints[0]

    return [
        (x - origin_x, y - origin_y)
        for x, y in waypoints
    ]


def get_waypoints():

    return shift_waypoints_to_origin(generate_test_track_waypoints())


def get_initial_waypoint():

    return get_waypoints()[0]
