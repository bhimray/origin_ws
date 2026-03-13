import math


CIRCLE_RADIUS = 2.0
CIRCLE_NUM_POINTS = 12
DEFAULT_PATH_PRESET = 'test_track'
LISSAJOUS_X_SCALE = 2.6
LISSAJOUS_Y_SCALE = 1.8
LISSAJOUS_NUM_POINTS = 24
TEST_TRACK_NUM_POINTS = 36


def generate_circle_waypoints(radius=CIRCLE_RADIUS, num_points=CIRCLE_NUM_POINTS):

    waypoints = []

    for i in range(num_points):
        theta = 2.0 * math.pi * i / num_points
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        waypoints.append((x, y))

    return waypoints


def generate_lissajous_waypoints(
    x_scale=LISSAJOUS_X_SCALE,
    y_scale=LISSAJOUS_Y_SCALE,
    num_points=LISSAJOUS_NUM_POINTS,
):

    waypoints = []

    for i in range(num_points):
        theta = 2.0 * math.pi * i / num_points
        x = x_scale * math.sin(3.0 * theta + (math.pi / 2.0))
        y = y_scale * math.sin(2.0 * theta)
        waypoints.append((x, y))

    return waypoints


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


def generate_clover_waypoints(radius=2.1, num_points=28):

    waypoints = []

    for i in range(num_points):
        theta = 2.0 * math.pi * i / num_points
        modulation = 0.65 + 0.35 * math.cos(4.0 * theta)
        x = radius * modulation * math.cos(theta)
        y = radius * modulation * math.sin(theta)
        waypoints.append((x, y))

    return waypoints


def generate_switchback_waypoints(width=2.4, height=1.8, lanes=4):

    if lanes < 2:
        raise ValueError('lanes must be at least 2')

    waypoints = []
    y_values = [
        -height + (2.0 * height * lane / (lanes - 1))
        for lane in range(lanes)
    ]

    for lane, y in enumerate(y_values):
        x = width if lane % 2 == 0 else -width
        waypoints.append((x, y))

        if lane == len(y_values) - 1:
            continue

        next_y = y_values[lane + 1]
        corner_x = width * 0.35 if lane % 2 == 0 else -width * 0.35
        waypoints.append((corner_x, y))
        waypoints.append((corner_x, next_y))

    return waypoints


def shift_waypoints_to_origin(waypoints):

    if not waypoints:
        return []

    origin_x, origin_y = waypoints[0]

    return [
        (x - origin_x, y - origin_y)
        for x, y in waypoints
    ]


def get_waypoints(path_preset=DEFAULT_PATH_PRESET):

    if path_preset == 'circle':
        return shift_waypoints_to_origin(generate_circle_waypoints())

    if path_preset == 'test_track':
        return shift_waypoints_to_origin(generate_test_track_waypoints())

    if path_preset == 'lissajous':
        return shift_waypoints_to_origin(generate_lissajous_waypoints())

    if path_preset == 'clover':
        return shift_waypoints_to_origin(generate_clover_waypoints())

    if path_preset == 'switchback':
        return shift_waypoints_to_origin(generate_switchback_waypoints())

    raise ValueError(f'Unsupported path preset: {path_preset}')

def get_initial_waypoint(path_preset=DEFAULT_PATH_PRESET):

    return get_waypoints(path_preset)[0]
