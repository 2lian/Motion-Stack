#!/bin/bash

cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
. "${ROS2_INSTALL_PATH}"/setup.bash || source /opt/ros/jazzy/setup.bash || source /opt/ros/humble/setup.bash || source /opt/ros/foxy/setup.bash || echo Ros2 not found for auto-sourcing, continuing
export RCUTILS_COLORIZED_OUTPUT=1
doit -n 8 build || colcon build --cmake-args -Wno-dev
# . ./venv/bin/activate
# export PYTHONPATH={here}/venv/lib/python3.12/site-packages:$PYTHONPATH
# export PATH={here}/venv/bin:$PATH
. ./install/setup.bash
. "${ROS2_INTERFACE_INSTALL_PATH}"/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"

ros2 launch motion_stack moonbot_zero.launch.py MS_down_from_level:=1 MS_up_to_level:=2 MS_simu_mode:=true
