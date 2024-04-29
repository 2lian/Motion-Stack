#!/bin/bash
. "${ROS2_INSTALL_PATH}"/setup.bash || . /opt/ros/humble/setup.bash || echo Ros2 Humble not found
. install/setup.bash
ros2 run rviz2 rviz2 -d ./src/rviz_basic/rviz2/urdf_vis.rviz
