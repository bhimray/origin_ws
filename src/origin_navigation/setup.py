from setuptools import find_packages, setup

package_name = 'origin_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['../../README.md']),

        ('share/' + package_name + '/launch',
            ['launch/simulation.launch.py']),

        ('share/' + package_name + '/rviz',
            ['rviz/navigation.rviz']),

        ('share/' + package_name + '/scripts',
            ['scripts/plot_navigation_metrics.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bim',
    maintainer_email='bim@todo.todo',
    description='Path smoothing and trajectory tracking for a ROS 2 robot.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_publisher = origin_navigation.waypoint_publisher:main',
            'path_smoother = origin_navigation.path_smoother:main',
            'trajectory_generator = origin_navigation.trajectory_generator:main',
            'trajectory_controller = origin_navigation.trajectory_controller:main',
            'odom_path_publisher = origin_navigation.odom_path_publisher:main',
            'navigation_metrics_logger = origin_navigation.navigation_metrics_logger:main',
        ],
    },
)
