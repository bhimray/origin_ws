"""Unit tests for trajectory controller logic."""

import math
import pathlib
import sys
from types import SimpleNamespace

sys.path.insert(
    0,
    str(pathlib.Path(__file__).resolve().parents[1]),
)

from origin_navigation.trajectory_controller import TrajectoryController


class _FakeParameterValue:

    def __init__(self, value):
        self.double_value = value


class _FakeParameter:

    def __init__(self, value):
        self._value = value

    def get_parameter_value(self):
        return _FakeParameterValue(self._value)


def _build_controller(parameters=None):
    controller = TrajectoryController.__new__(TrajectoryController)
    controller.trajectory = []
    controller.current_index = 0
    controller.state = None

    parameter_values = {
        'front_axle_offset': 0.20,
        'heading_gain': 2.0,
        'stanley_gain': 3.2,
        'softening_velocity': 0.05,
        'speed_gain': 1.2,
        'max_linear_velocity': 0.16,
        'max_angular_velocity': 2.2,
    }
    if parameters:
        parameter_values.update(parameters)

    controller.get_parameter = lambda name: _FakeParameter(
        parameter_values[name]
    )
    return controller


def _make_odom_msg(x, y, yaw=0.0, linear_velocity=0.0):
    half_yaw = yaw * 0.5
    orientation = SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
                orientation=orientation,
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=linear_velocity),
            )
        ),
    )


def test_front_axle_position_uses_yaw_and_offset():
    controller = _build_controller({'front_axle_offset': 0.5})
    controller.state = {
        'x': 1.0,
        'y': 2.0,
        'yaw': math.pi / 2.0,
        'linear_velocity': 0.0,
    }

    front_x, front_y = controller.front_axle_position()

    assert math.isclose(front_x, 1.0, abs_tol=1e-9)
    assert math.isclose(front_y, 2.5, abs_tol=1e-9)


def test_next_point_wraps_to_start_for_closed_loop():
    controller = _build_controller()
    controller.trajectory = [
        {'x': 0.0, 'y': 0.0},
        {'x': 1.0, 'y': 0.0},
        {'x': 2.0, 'y': 0.0},
    ]

    assert controller.next_point(2) == controller.trajectory[0]


def test_reference_speed_returns_segment_speed():
    controller = _build_controller()
    controller.trajectory = [
        {'desired_speed': 0.05},
        {'desired_speed': 0.12},
    ]

    assert math.isclose(controller.reference_speed(1), 0.12)


def test_command_speed_scales_down_for_large_heading_error():
    controller = _build_controller(
        {
            'speed_gain': 1.0,
            'max_linear_velocity': 0.16,
        }
    )
    controller.state = {
        'x': 0.0,
        'y': 0.0,
        'yaw': 0.0,
        'linear_velocity': 0.0,
    }

    commanded_speed = controller.command_speed(0.16, 1.6)

    assert math.isclose(commanded_speed, 0.024, rel_tol=1e-9, abs_tol=1e-9)


def test_command_speed_clamps_to_max_linear_velocity():
    controller = _build_controller(
        {
            'speed_gain': 2.0,
            'max_linear_velocity': 0.16,
        }
    )
    controller.state = {
        'x': 0.0,
        'y': 0.0,
        'yaw': 0.0,
        'linear_velocity': 0.10,
    }

    commanded_speed = controller.command_speed(0.30, 0.0)

    assert math.isclose(commanded_speed, 0.16, abs_tol=1e-9)


def test_find_nearest_segment_index_chooses_expected_segment():
    controller = _build_controller()
    controller.trajectory = [
        {'x': 0.0, 'y': 0.0},
        {'x': 1.0, 'y': 0.0},
        {'x': 1.0, 'y': 1.0},
        {'x': 0.0, 'y': 1.0},
    ]

    nearest_index = controller.find_nearest_segment_index(1.05, 0.60)

    assert nearest_index == 1


def test_odom_callback_ignores_messages_until_trajectory_is_ready():
    controller = _build_controller()
    controller.trajectory = [{'x': 0.0, 'y': 0.0, 'desired_speed': 0.0}]
    published_commands = []
    controller.publish_command = (
        lambda linear_velocity, angular_velocity: published_commands.append(
            (linear_velocity, angular_velocity)
        )
    )

    controller.odom_callback(_make_odom_msg(0.0, 0.0))

    assert controller.state is None
    assert published_commands == []


def test_odom_callback_stops_for_zero_length_segment():
    controller = _build_controller()
    controller.trajectory = [
        {'x': 0.0, 'y': 0.0, 'desired_speed': 0.1},
        {'x': 0.0, 'y': 0.0, 'desired_speed': 0.1},
    ]
    published_commands = []
    controller.publish_command = (
        lambda linear_velocity, angular_velocity: published_commands.append(
            (linear_velocity, angular_velocity)
        )
    )

    controller.odom_callback(_make_odom_msg(0.0, 0.0))

    assert published_commands == [(0.0, 0.0)]


def test_odom_callback_clamps_angular_velocity_before_publishing():
    controller = _build_controller(
        {
            'heading_gain': 10.0,
            'stanley_gain': 0.0,
            'speed_gain': 1.0,
            'max_linear_velocity': 0.16,
            'max_angular_velocity': 0.5,
        }
    )
    controller.trajectory = [
        {'x': 0.0, 'y': 0.0, 'desired_speed': 0.16},
        {'x': 1.0, 'y': 0.0, 'desired_speed': 0.16},
    ]
    published_commands = []
    controller.publish_command = (
        lambda linear_velocity, angular_velocity: published_commands.append(
            (linear_velocity, angular_velocity)
        )
    )

    controller.odom_callback(_make_odom_msg(0.0, 0.0, yaw=-math.pi / 2.0))

    assert len(published_commands) == 1
    linear_velocity, angular_velocity = published_commands[0]
    assert 0.0 <= linear_velocity <= 0.16
    assert math.isclose(angular_velocity, 0.5, abs_tol=1e-9)
