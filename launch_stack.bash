#!/bin/bash

cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"

ros2 launch motion_stack moonbot_zero.launch.py MS_down_from_level:=1 MS_up_to_level:=2 MS_simu_mode:=true
