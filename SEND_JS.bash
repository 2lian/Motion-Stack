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

# JOINT="leg3link5_link6"
LEG=1
JOINT="joint${LEG}_1, joint${LEG}_2, joint${LEG}_3"
ANGLE="0, 0, 0"
# ANGLE="1.0,1.0"
# SPEED="1.0,1.0"
ros2 topic pub /leg$LEG/joint_set sensor_msgs/msg/JointState "{name: [$JOINT], position: [$ANGLE], velocity: [$SPEED], effort: []}" --rate 100
# ros2 topic pub /leg$LEG/joint_set sensor_msgs/msg/JointState "{name: [$JOINT], position: [$P, $P], velocity: [], effort: []}" -1
# # # ros2 topic pub /leg$LEG/joint_set sensor_msgs/msg/JointState "{name: [$JOINT], position: [], velocity: [-1, 0], effort: []}" -1
# ros2 topic pub /leg$LEG/joint_set sensor_msgs/msg/JointState "{name: [$JOINT], position: [0, 0], velocity: [], effort: []}" -1

# ros2 service call /leg1/set_offset motion_stack_msgs/srv/SendJointState "{js: {name: [$JOINT], position: [$P, $P], velocity: [], effort: []}}"
# ros2 topic pub /leg$LEG/joint_set sensor_msgs/msg/JointState "{name: [$JOINT], position: [0, 0], velocity: [], effort: []}" -1
