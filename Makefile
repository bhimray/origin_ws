.PHONY: build unit-test test

ROS_DISTRO ?= jazzy
PACKAGE ?= origin_navigation

build:
	bash -lc "set -eo pipefail; cd /home/bim/origin_ws; source /opt/ros/$(ROS_DISTRO)/setup.bash; colcon build --packages-select $(PACKAGE)"

unit-test:
	bash -lc "set -eo pipefail; cd /home/bim/origin_ws; source /opt/ros/$(ROS_DISTRO)/setup.bash; python3 -m pytest -q src/origin_navigation/test/test_trajectory_math.py src/origin_navigation/test/test_trajectory_controller.py"

test:
	bash -lc "set -eo pipefail; cd /home/bim/origin_ws; source /opt/ros/$(ROS_DISTRO)/setup.bash; colcon build --packages-select $(PACKAGE); source install/setup.bash; colcon test --packages-select $(PACKAGE); colcon test-result --verbose"
