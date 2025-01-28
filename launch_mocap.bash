#!/bin/bash

cd "${ROS2_MOONBOT_WS}" || echo "No folder shortcut, working in $PWD"

. "${ROS2_INSTALL_PATH}"/setup.bash \
  || source /opt/ros/humble/setup.bash \
  || source /opt/ros/foxy/setup.bash \
  || echo "ROS 2 not found for auto-sourcing, continuing"

export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"

. install/setup.bash

SIMULATION_MODE=false

ARM_MOCAP_FRAME="mocap4gripper1"
ARM_FRAME="leg4gripper1"

WHEEL_MOCAP_FRAME="mocap14_body"
WHEEL_FRAME="wheel14_in"

ros2 launch ros2_m_hero_pkg mocap_to_arm_wheel_tf.launch.py

