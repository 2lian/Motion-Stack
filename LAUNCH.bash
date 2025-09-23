#!/bin/bash
# This bash is for debugging by developpers, use launch_stack.bash instead
#
# . ./venv/bin/activate
# rm -r log/ build/ install/
# rm -r ./docs/build/
# doit clean
doit -n 16 build
. ./venv/bin/activate
. ./install/setup.bash
python3 -m pytest ./src/motion_stack/test \
    # -m only \
    # -v \
    # -s \

exit 0

export M_LEG=
export USE_RVIZ=
UP_TO=1


cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
. "${ROS2_INSTALL_PATH}"/setup.bash || source /opt/ros/humble/setup.bash || source /opt/ros/foxy/setup.bash || echo Ros2 not found for auto-sourcing, continuing
# rm -r install/rviz_basic
# rm -r build/rviz_basic
# rm -r log/rviz_basic
# rm -r install/easy_robot_control
# rm -r build/easy_robot_control
# rm -r log/easy_robot_control
# rm -r log/ build/ install/
# . install/setup.bash
export RCUTILS_COLORIZED_OUTPUT=1
# colcon build --symlink-install --cmake-args -Wno-dev
# colcon build --cmake-args -Wno-dev
doit -n 16 build
. install/setup.bash
# colcon test --packages-select motion_stack easy_robot_control ros2_m_hero_pkg rviz_basic --event-handlers console_cohesion+
# colcon test-result --verbose
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export NUMBA_CACHE_DIR="./numba_cache" # this will compile numba in a permanant file

ros2 run moonbot_zero_tuto high_dbg
# ros2 run motion_stack trial
# ros2 launch motion_stack moonbot_zero.launch.py MS_up_to_level:=$UP_TO
# ros2 launch moonbot_zero_tuto myrobot.launch.py
# ros2 launch moonbot_zero_tuto real_moonbot.launch.py MS_up_to_level:=4
