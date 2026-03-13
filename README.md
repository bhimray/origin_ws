# Origin Navigation Assignment

This workspace contains a ROS 2 Python solution for 2D path smoothing and
trajectory tracking with a differential-drive robot in simulation. The pipeline
is:

`waypoints -> smooth_path -> timed trajectory -> controller -> /cmd_vel`

## What is implemented

- Path smoothing with a spline-based resampler that converts sparse waypoints
  into an approximately arc-length-uniform path.
- Time-parameterized trajectory generation with:
  - constant-speed timing for closed loops
  - trapezoidal timing for open paths
- A trajectory tracking controller for a differential-drive robot using:
  - nearest-reference progress tracking
  - pure-pursuit-style lookahead steering
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

Prerequisites:

- ROS 2 installed and sourced
- TurtleBot3 simulation packages installed
- Python dependencies available for ROS 2:
  - `numpy`
  - `scipy`
  - `matplotlib`

Environment example:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
export TURTLEBOT3_MODEL=burger
```

Build:

```bash
cd /home/bim/origin_ws
colcon build --packages-select origin_navigation
source install/setup.bash
```

## Run

Launch the full simulation:

```bash
ros2 launch origin_navigation simulation.launch.py
```

Useful launch arguments:

```bash
ros2 launch origin_navigation simulation.launch.py cruise_speed:=0.25 closed_path:=true
```

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

Run the package tests:

```bash
cd /home/bim/origin_ws
colcon test --packages-select origin_navigation
colcon test-result --verbose
```

The included unit tests cover smoothing density and timing monotonicity in
`test/test_trajectory_math.py`.

## Results and plots

After a simulation run, metrics are saved to:

- `results/navigation_metrics.csv`

Plots can be generated with:

```bash
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

All generated code and explanations should still be reviewed and validated in
simulation before use on hardware.
