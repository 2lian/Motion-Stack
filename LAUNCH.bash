#!/bin/bash
# rm -r log/ build/ install/
# doit clean
doit -n 16 md
exit 0
# This bash is for debugging, use launch_stack.bash instead
export M_LEG=
export USE_RVIZ=
UP_TO=4


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

ros2 launch motion_stack moonbot_zero.launch.py MS_up_to_level:=$UP_TO
# ros2 launch ros2_m_hero_pkg hero_dragon.launch.py MS_up_to_level:=$UP_TO
# ros2 launch ros2_m_hero_pkg hero_all.launch.py MS_up_to_level:=$UP_TO
# ros2 launch easy_robot_control gusta.launch.py MS_down_from_level:=0 MS_up_to_level:=$UP_TO MS_simu_mode:=True
# ros2 launch ros2_m_hero_pkg hero_3leg.launch.py MS_up_to_level:=$UP_TO
# ros2 launch ros2_m_hero_pkg hero_3legwheel.launch.py MS_up_to_level:=$UP_TO
# ros2 launch moonbot_zero_tuto myrobot.launch.py
# ros2 launch ./robot_launcher.launch.py
# ros2 run easy_robot_control test

# cd ~/Moonbot-Motion-Stack/src/urdf_packer/urdf/hero_7dof 
# xacro hero_dragon.xacro
# . install/setup.bash

# python ~/Moonbot-Motion-Stack/src/easy_robot_cont rol/launch/hero_dragon.py
