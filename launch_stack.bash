#!/bin/bash

cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
. "${ROS2_INSTALL_PATH}"/setup.bash || source /opt/ros/humble/setup.bash || source /opt/ros/foxy/setup.bash || echo Ros2 not found for auto-sourcing, continuing
export RCUTILS_COLORIZED_OUTPUT=1
# colcon build --cmake-args -Wno-dev
doit -n 8 build || colcon build --cmake-args -Wno-dev
# colcon test --packages-select easy_robot_control
# colcon test-result --verbose
. ./install/setup.bash
. "${ROS2_INTERFACE_INSTALL_PATH}"/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export NUMBA_CACHE_DIR="./numba_cache" # this will compile numba in a permanant file

# ros2 launch motion_stack moonbot_zero.launch.py MS_down_from_level:=1 MS_up_to_level:=4 MS_simu_mode:=true
# ros2 launch moonbot_zero_tuto myrobot.launch.py MS_up_to_level:=2
# ros2 launch ros2_m_hero_pkg hero_dragon.launch.py MS_up_to_level:=4
ros2 launch ros2_m_hero_pkg hero_all.launch.py MS_up_to_level:=4 #MS_simu_mode:=False
# ros2 launch realman_interface realman_75.py MS_up_to_level:=3
# ros2 launch moonbot_g moonbot_g_4dof.launch.py MS_up_to_level:=3
# ros2 launch moonbotg_7dof_hd moonbotg_7dof_hd.launch.py MS_up_to_level:=2
# ros2 launch moonbot_zero_tuto real_moonbot.launch.py MS_up_to_level:=2
