# Origin Navigation Assignment

This workspace contains a ROS 2 Python package for 2D path smoothing and
trajectory tracking of a differential-drive robot in simulation.

Pipeline:

`waypoints -> smooth_path -> timed trajectory -> controller -> /cmd_vel`

## Quick start

If your ROS 2 environment and TurtleBot3 simulator are already installed, the
minimum commands are:

```bash
cd /home/bim/origin_ws
source /opt/ros/$ROS_DISTRO/setup.bash
export TURTLEBOT3_MODEL=burger
colcon build --packages-select origin_navigation
source install/setup.bash
ros2 launch origin_navigation simulation.launch.py
```

This launches Gazebo, RViz, the path smoother, the trajectory generator, and
the trajectory tracking controller.

## What is implemented

- Path smoothing with a spline-based resampler that converts sparse waypoints
  into an approximately arc-length-uniform path.
- Time-parameterized trajectory generation with:
  - trapezoidal timing for paths
- A trajectory tracking controller for a differential-drive robot using:
  - nearest-reference progress tracking
  - Trapezoidal reference velocity
  - Stanley controller for angular velocity
  - heading and cross-track feedback
- RViz visualization for:
  - original waypoints
  - smoothed reference path
  - actual odometry path
- CSV logging of path error, heading error, and robot velocities.

## Package layout

- `src/origin_navigation/origin_navigation/path_smoother.py`
  Smooths discrete waypoints into a continuous path.
- `src/origin_navigation/origin_navigation/trajectory_generator.py`
  Converts the smoothed path into a timed trajectory on `/trajectory`.
- `src/origin_navigation/origin_navigation/trajectory_controller.py`
  Publishes `/cmd_vel` commands to track the generated trajectory.
- `src/origin_navigation/origin_navigation/trajectory_math.py`
  Reusable geometry and timing utilities with unit tests.
- `src/origin_navigation/origin_navigation/navigation_metrics_logger.py`
  Writes tracking metrics to `results/navigation_metrics.csv`.

## Setup

### 1. Prerequisites

Before building this package, make sure you have:

- ROS 2 (in my case ros-jazzy) installed and working
- `colcon` available in your shell
- TurtleBot3 simulation packages installed
- Python ROS dependencies available:
  - `numpy`
  - `scipy`
  - `matplotlib`

The package dependencies declared in
`src/origin_navigation/package.xml` include:

- `rclpy`
- `geometry_msgs`
- `nav_msgs`
- `tf_transformations`
- `turtlebot3_gazebo`

### 2. Source ROS 2 and set the robot model

Open a terminal and run:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
export TURTLEBOT3_MODEL=burger
```

If you use a different TurtleBot3 model, replace `burger` with that model name.

### 3. Build the workspace

From the workspace root:

```bash
cd /home/bim/origin_ws
colcon build --packages-select origin_navigation
```

### 4. Source the workspace overlay

After the build finishes:

```bash
source /home/bim/origin_ws/install/setup.bash
```

You must source this overlay in every new terminal before running the package.

## Run the simulation

Launch the full navigation demo with:

```bash
ros2 launch origin_navigation simulation.launch.py
```

This starts:

- the TurtleBot3 simulation
- the smoothing and trajectory generation nodes
- the trajectory tracking controller
- RViz visualization
- metrics logging

### Optional launch arguments

Example with a custom cruise speed and a closed-loop path:

```bash
ros2 launch origin_navigation simulation.launch.py cruise_speed:=0.25 closed_path:=true
```

### What you should see

When the launch succeeds, you should see:

- Gazebo running with the robot in simulation
- RViz showing the waypoint path, smoothed path, and robot motion
- the robot publishing `/cmd_vel` and following the generated trajectory

## Design choices

- `nav_msgs/Path` is used for both the smoothed path and the timed trajectory so
  each trajectory sample can carry its own timestamp and heading.
- The smoother resamples by distance instead of only by spline parameter. That
  produces more uniform spacing for control.
- Closed-loop tracks use constant-speed timing because the robot should keep
  circulating. Open tracks use a trapezoidal time law so the robot starts and
  stops smoothly.
- The controller is hybrid rather than purely time-based. It uses the generated
  trajectory for desired speed and tangent heading, but it locks progress to the
  nearest path region so it can recover from tracking errors.

## Testing

Run the package tests from the workspace root:

```bash
cd /home/bim/origin_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
colcon test --packages-select origin_navigation
colcon test-result --verbose
```

The included unit tests cover smoothing density and timing monotonicity in
`src/origin_navigation/test/test_trajectory_math.py`.

## Results and plots

After a simulation run, tracking metrics are saved to:

- `results/navigation_metrics.csv`

To generate plots from the saved metrics:

```bash
cd /home/bim/origin_ws
python3 src/origin_navigation/scripts/plot_navigation_metrics.py
```

## Real robot extension

- Replace Gazebo odometry with fused wheel encoder + IMU localization.
- Tune controller gains for the physical wheelbase, latency, and actuator limits.
- Add velocity and acceleration limiters around `/cmd_vel`.
- Add a safety layer for emergency stop, stale trajectory detection, and watchdog
  behavior.

## Extra credit: obstacle avoidance

The cleanest extension is to keep the current stack and add a local planner or
reactive avoidance layer between trajectory generation and tracking. Two common
options are:

- sample trajectory rollouts and score them against an occupancy grid
- add a local obstacle repulsion term that deforms the reference path online

In a production robot, obstacle avoidance should consume live sensor data and
modify either the local target or the short planning horizon, not the original
global waypoint list in place.

## AI tools used

AI assistance was used to accelerate:

- codebase understanding
- controller and trajectory design iteration
- documentation drafting
- unit test scaffolding

