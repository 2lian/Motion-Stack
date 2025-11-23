#!/bin/bash
# This bash is for debugging by developpers, use launch_stack.bash instead
#
. ./venv/bin/activate
# rm -r log/ build/ install/
# rm -r ./docs/build/
# doit clean
doit -n 16 html md
exit 0

export M_LEG=
export USE_RVIZ=
UP_TO=1


cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
. "${ROS2_INSTALL_PATH}"/setup.bash || source /opt/ros/humble/setup.bash || source /opt/ros/foxy/setup.bash || echo Ros2 not found for auto-sourcing, continuing
export RCUTILS_COLORIZED_OUTPUT=1
# colcon build --symlink-install --cmake-args -Wno-dev
# colcon build --cmake-args -Wno-dev
doit -n 16 build
. install/setup.bash
# colcon test-result --verbose
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export NUMBA_CACHE_DIR="./numba_cache" # this will compile numba in a permanant file

ros2 launch motion_stack_tuto myrobot.launch.py
