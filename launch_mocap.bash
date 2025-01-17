#!/bin/bash

cd "${ROS2_MOONBOT_WS}" || echo "No folder shortcut, working in $PWD"

. "${ROS2_INSTALL_PATH}"/setup.bash || source /opt/ros/humble/setup.bash || source /opt/ros/foxy/setup.bash || echo "ROS 2 not found for auto-sourcing, continuing"

export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"

. install/setup.bash

SIMULATION_MODE=true
MOCAP_FRAME="mocap3gripper1"
ROBOT_FRAME="leg3gripper1"
OFFSET_TRANSLATION="[0.0, 0.0, 0.0]"
OFFSET_ROTATION_EULER="[0.0, 0.0, 1.57]"

ros2 launch ros2_m_hero_pkg mocap_to_robot_tf.launch.py \
  simulation_mode:="${SIMULATION_MODE}" \
  mocap_frame:="${MOCAP_FRAME}" \
  robot_frame:="${ROBOT_FRAME}" \
  offset_translation:="${OFFSET_TRANSLATION}" \
  offset_rotation_euler:="${OFFSET_ROTATION_EULER}"

