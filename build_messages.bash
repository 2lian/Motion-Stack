#!/bin/bash
# run this inside this folder
cd "${ROS2_MOONBOT_WS}" || exit
source "${ROS2_INSTALL_PATH}"/setup.bash
colcon build --symlink-install --packages-select custom_messages
. install/setup.bash
