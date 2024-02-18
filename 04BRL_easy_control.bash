#!/bin/bash
# run this inside this folder
cd "${ROS2_MOONBOT_WS}" || exit
. "${ROS2_INSTALL_PATH}"/setup.bash
# colcon build --symlink-install --packages-select easy_robot_control
. install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1
#export ROS_DOMAIN_ID=58
ros2 launch src/easy_robot_control/launch/lvl_04_mover.py
