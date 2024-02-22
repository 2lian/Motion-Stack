#!/bin/bash
# run this inside this folder
cd "${ROS2_MOONBOT_WS}" || exit
. "${ROS2_INSTALL_PATH}"/setup.bash
# . install/setup.bash
rm -rf install
rm -rf build
colcon build --symlink-install
. install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1

rqt || exit &
. 04BRL_easy_control.bash || exit &
ros2 run pcl_reader pointcloud_read_pub || exit &
. BL_rviz.bash || exit

#export ROS_DOMAIN_ID=58
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: 50, y: 50, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: -100, y: -100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: 100, y: 100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: -100, y: -100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: 100, y: 100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: -100, y: -100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: 100, y: 100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: -100, y: -100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: 100, y: 100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: -100, y: -100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: 100, y: 100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: -100, y: -100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: 100, y: 100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: -100, y: -100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: 100, y: 100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: -100, y: -100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: 100, y: 100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: -100, y: -100, z: 0}}"
# ros2 service call body_shift custom_messages/srv/Vect3 "{vector: {x: 100, y: 100, z: 0}}"
