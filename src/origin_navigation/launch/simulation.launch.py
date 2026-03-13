from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Spawn position (first circle waypoint)
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    declare_x = DeclareLaunchArgument(
        'x_pose',
        default_value='2.0'
    )

    declare_y = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0'
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
        output='screen'
    )

    # Path smoothing
    smoothing_node = Node(
        package='origin_navigation',
        executable='path_smoother',
        name='path_smoother',
        output='screen'
    )

    # Trajectory generation
    trajectory_node = Node(
        package='origin_navigation',
        executable='trajectory_generator',
        name='trajectory_generator',
        output='screen'
    )

    # Controller
    controller_node = Node(
        package='origin_navigation',
        executable='trajectory_controller',
        name='trajectory_controller',
        output='screen'
    )

    # Odometry path for visual comparison against reference trajectory
    odom_path_node = Node(
        package='origin_navigation',
        executable='odom_path_publisher',
        name='odom_path_publisher',
        output='screen'
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

        declare_x,
        declare_y,

        gazebo,

        waypoint_node,
        smoothing_node,
        trajectory_node,
        controller_node,
        odom_path_node,

        rviz
    ])
