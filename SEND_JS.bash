#!/bin/bash
# This bash is for debugging, use launch_stack.bash instead
export M_LEG=    
export USE_RVIZ=

cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
. "${ROS2_INSTALL_PATH}"/setup.bash || source /opt/ros/humble/setup.bash || source /opt/ros/foxy/setup.bash || echo Ros2 not found for auto-sourcing, continuing
# rm -rf install
# rm -rf build
# rm -rf log
# . install/setup.bash
export RCUTILS_COLORIZED_OUTPUT=1
. install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"

ros2 topic pub /leg0/joint_set sensor_msgs/msg/JointState "{name: [shoulder_lift_joint], position: [-1], velocity: [], effort: []}" -1
ros2 topic pub /leg0/joint_set sensor_msgs/msg/JointState "{name: [shoulder_lift_joint], position: [], velocity: [0], effort: []}" -1
ros2 topic pub /leg0/joint_set sensor_msgs/msg/JointState "{name: [shoulder_lift_joint], position: [-1], velocity: [], effort: []}" -1
