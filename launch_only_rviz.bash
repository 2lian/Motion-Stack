#!/bin/bash
cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
ros2 launch rviz_basic rviz_vizu.launch.py
