#!/bin/bash
# set -e -o pipefail

cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
. "${ROS2_INSTALL_PATH}"/setup.bash || source /opt/ros/humble/setup.bash || source /opt/ros/foxy/setup.bash || echo Ros2 not found for auto-sourcing, continuing
# rm -rf install
# rm -rf build
rm -rf log
# . install/setup.bash
export RCUTILS_COLORIZED_OUTPUT=1
# colcon build --symlink-install
colcon test --packages-select easy_robot_control
colcon test-result --verbose
colcon build --cmake-args -Wno-dev
. install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export NUMBA_CACHE_DIR="./numba_cache" # this will compile numba in a permanant file

ros2 launch ./launch_stack_rviz.launch.py
# ros2 run easy_robot_control test
