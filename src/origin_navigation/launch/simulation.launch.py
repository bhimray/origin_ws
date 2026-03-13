from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from origin_navigation.path_config import get_initial_waypoint

import os


def generate_launch_description():

    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_models_dir = os.path.join(turtlebot3_gazebo_dir, 'models')

    for env_var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH'):
        existing_paths = [
            path for path in os.environ.get(env_var, '').split(os.pathsep) if path
        ]
        if turtlebot3_models_dir not in existing_paths:
            os.environ[env_var] = os.pathsep.join(
                [*existing_paths, turtlebot3_models_dir]
            )

    default_path_preset = 'test_track'
    initial_x, initial_y = get_initial_waypoint(default_path_preset)
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    path_preset = LaunchConfiguration('path_preset')
    closed_path = LaunchConfiguration('closed_path')
    cruise_speed = LaunchConfiguration('cruise_speed')
    velocity_profile = LaunchConfiguration('velocity_profile')

    declare_x = DeclareLaunchArgument(
        'x_pose',
        default_value=str(initial_x)
    )

    declare_y = DeclareLaunchArgument(
        'y_pose',
        default_value=str(initial_y)
    )

    declare_closed_path = DeclareLaunchArgument(
        'closed_path',
        default_value='true'
    )

    declare_cruise_speed = DeclareLaunchArgument(
        'cruise_speed',
        default_value='0.3'
    )

    declare_path_preset = DeclareLaunchArgument(
        'path_preset',
        default_value=default_path_preset
    )

    declare_velocity_profile = DeclareLaunchArgument(
        'velocity_profile',
        default_value='trapezoidal'
    )

    gz_sim_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        turtlebot3_models_dir
    )

    ign_gazebo_resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        turtlebot3_models_dir
    )

    # Launch Gazebo world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo_dir,
                'launch',
                'empty_world.launch.py'
            )
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Waypoint generator
    waypoint_node = Node(
        package='origin_navigation',
        executable='waypoint_publisher',
        name='waypoint_generator',
        output='screen',
        parameters=[{
            'path_preset': path_preset
        }]
    )

    # Path smoothing
    smoothing_node = Node(
        package='origin_navigation',
        executable='path_smoother',
        name='path_smoother',
        output='screen',
        parameters=[{
            'closed_path': closed_path,
            'sample_spacing': 0.08,
            'spline_smoothing': 0.02,
        }]
    )

    # Trajectory generation
    trajectory_node = Node(
        package='origin_navigation',
        executable='trajectory_generator',
        name='trajectory_generator',
        output='screen',
        parameters=[{
            'closed_path': closed_path,
            'velocity_profile': velocity_profile,
            'cruise_speed': cruise_speed,
            'acceleration': 0.5,
        }]
    )

    # Controller
    controller_node = Node(
        package='origin_navigation',
        executable='trajectory_controller',
        name='trajectory_controller',
        output='screen',
        parameters=[{
            'closed_path': closed_path,
            'max_linear_velocity': 0.16,
            'max_angular_velocity': 2.2,
            'front_axle_offset': 0.20,
            'stanley_gain': 2.8,
            'heading_gain': 1.6,
            'softening_velocity': 0.05,
            'speed_gain': 1.0,
        }]
    )

    # Odometry path for visual comparison against reference trajectory
    odom_path_node = Node(
        package='origin_navigation',
        executable='odom_path_publisher',
        name='odom_path_publisher',
        output='screen'
    )

    metrics_logger_node = Node(
        package='origin_navigation',
        executable='navigation_metrics_logger',
        name='navigation_metrics_logger',
        output='screen',
        parameters=[{
            'output_path': '/home/bim/origin_ws/results/navigation_metrics.csv'
        }]
    )

    # RViz visualization
    rviz_config = os.path.join(
        get_package_share_directory('origin_navigation'),
        'rviz',
        'navigation.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([

        gz_sim_resource_path,
        ign_gazebo_resource_path,
        declare_x,
        declare_y,
        declare_path_preset,
        declare_closed_path,
        declare_cruise_speed,
        declare_velocity_profile,

        gazebo,

        waypoint_node,
        smoothing_node,
        trajectory_node,
        controller_node,
        odom_path_node,
        metrics_logger_node,

        rviz
    ])
