#!/bin/bash
# set -e -o pipefail

cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
. "${ROS2_INSTALL_PATH}"/setup.bash || source /opt/ros/humble/setup.bash || source /opt/ros/foxy/setup.bash || echo Ros2 not found for auto-sourcing, continuing
export RCUTILS_COLORIZED_OUTPUT=1
# colcon build --cmake-args -Wno-dev
doit -n 16 build
. install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export NUMBA_CACHE_DIR="./numba_cache" # this will compile numba in a permanant file

ros2 run keyboard keyboard --ros-args -r __ns:="/${OPERATOR}" &
PID_KEY=$!

ros2 run joy joy_node --ros-args --log-level WARN -r __ns:="/${OPERATOR}" -p deadzone:=0.025 -p autorepeat_rate:=0.0 &
PID_JOY=$!

ros2 run ms_operator operator \
    --ros-args \
    -p joint_speed:=0.15 \
    -p wheel_speed:=0.2 \
    -p translation_speed:=50.0 \
    -p rotation_speed:=0.15 \
    -r /keydown:="/${OPERATOR}/keydown" \
    -r /keyup:="/${OPERATOR}/keyup" \
    -r /joy:="/${OPERATOR}/joy"

wait $PID_KEY $PID_JOY
