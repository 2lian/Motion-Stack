#!/bin/bash
set -e -o pipefail
# run this inside this folder
cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
. "${ROS2_INSTALL_PATH}"/setup.bash || . /opt/ros/humble/setup.bash || echo Ros2 Humble not found
# colcon build --packages-select rviz_basic
. install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1
# export ROS_DOMAIN_ID=58
ros2 launch src/rviz_basic/launch/rviz.launch.py
