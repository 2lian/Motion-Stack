#!/bin/bash
cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
# . "${ROS2_INSTALL_PATH}"/setup.bash || source /opt/ros/humble/setup.bash || source /opt/ros/foxy/setup.bash || echo Ros2 not found for auto-sourcing, continuing
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
# . install/setup.bash
ros2 launch motion_stack rviz_simu.launch.py
