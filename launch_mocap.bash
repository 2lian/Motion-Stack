#!/bin/bash

cd "${ROS2_MOONBOT_WS}" || echo "No folder shortcut, working in $PWD"

. "${ROS2_INSTALL_PATH}"/setup.bash \
  || source /opt/ros/humble/setup.bash \
  || source /opt/ros/foxy/setup.bash \
  || echo "ROS 2 not found for auto-sourcing, continuing"

export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"

. install/setup.bash

SIMULATION_MODE=true

ARM_MOCAP_FRAME="mocap3gripper1"
ARM_FRAME="leg3gripper1"
ARM_OFFSET_TRANSLATION="[0.0, 0.0, 0.2]"
ARM_OFFSET_ROTATION_EULER="[0.0, 0.0, 1.57]"

WHEEL_MOCAP_FRAME="mocap11_body"
WHEEL_FRAME="wheel11_body"
WHEEL_OFFSET_TRANSLATION="[0.0, 0.0, 0.0]"
WHEEL_OFFSET_ROTATION_EULER="[0.0, 0.0, 1.57]"

# Launch
ros2 launch ros2_m_hero_pkg mocap_to_arm_wheel_tf.launch.py \
  simulation_mode:="${SIMULATION_MODE}" \
  arm_mocap_frame:="${ARM_MOCAP_FRAME}" \
  arm_frame:="${ARM_FRAME}" \
  arm_offset_translation:="${ARM_OFFSET_TRANSLATION}" \
  arm_offset_rotation_euler:="${ARM_OFFSET_ROTATION_EULER}" \
  wheel_mocap_frame:="${WHEEL_MOCAP_FRAME}" \
  wheel_frame:="${WHEEL_FRAME}" \
  wheel_offset_translation:="${WHEEL_OFFSET_TRANSLATION}" \
  wheel_offset_rotation_euler:="${WHEEL_OFFSET_ROTATION_EULER}"

