#!/bin/bash
set -e -o pipefail

cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
. "${ROS2_INSTALL_PATH}"/setup.bash || . /opt/ros/humble/setup.bash || echo Ros2 Humble not found
# export ROS_DOMAIN_ID=58
# rm -rf install
# rm -rf build
# . install/setup.bash
colcon build --symlink-install
. install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1
export NUMBA_CACHE_DIR="./numba_cache" # this will compile numba in a permanant file

ros2 launch ./launch_stack_rviz.launch.py
