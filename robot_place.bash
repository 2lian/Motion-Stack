#!/bin/bash
set -e -o pipefail

cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
. "${ROS2_INSTALL_PATH}"/setup.bash || . /opt/ros/humble/setup.bash || echo Ros2 Humble not found

ros2 topic pub /r1/robot_body geometry_msgs/msg/Transform "{translation: {x: 0.2, y: 0.3, z: 0.0}, rotation: {x: 0.0, y: -0.2, z: 0.0, w: 0.8}}" --once
ros2 topic pub /r1/set_ik_target_0 geometry_msgs/msg/Vector3 "{x: 400, y: -200, z: 0.0}" --once &
ros2 topic pub /r1/set_ik_target_1 geometry_msgs/msg/Vector3 "{x: 100, y: 300, z: -150.0}" --once &
ros2 topic pub /r1/set_ik_target_2 geometry_msgs/msg/Vector3 "{x: -400, y: -100, z: -150.0}" --once &

ros2 topic pub /r2/robot_body geometry_msgs/msg/Transform "{translation: {x: -0.3, y: -0.3, z: -0.2}, rotation: {x: 0.15, y: 0.1, z: 0.0, w: 0.9}}" --once
ros2 topic pub /r2/set_ik_target_0 geometry_msgs/msg/Vector3 "{x: 400, y: 0, z: -200}" --once &
ros2 topic pub /r2/set_ik_target_1 geometry_msgs/msg/Vector3 "{x: 100, y: 350, z: 50.0}" --once &
ros2 topic pub /r2/set_ik_target_2 geometry_msgs/msg/Vector3 "{x: -400, y: -100, z: -150.0}" --once &
ros2 topic pub /r2/set_ik_target_3 geometry_msgs/msg/Vector3 "{x: 0, y: -400, z: -100.0}" --once &

ros2 topic pub /r3/robot_body geometry_msgs/msg/Transform "{translation: {x: -0.3, y: -1.2, z: -0.1}, rotation: {x: -0.1, y: 0.0, z: 0.0, w: 0.9}}" --once
ros2 topic pub /r3/set_ik_target_0 geometry_msgs/msg/Vector3 "{x: 400, y: 0, z: -200}" --once &
ros2 topic pub /r3/set_ik_target_1 geometry_msgs/msg/Vector3 "{x: 100, y: 300, z: -150.0}" --once &
ros2 topic pub /r3/set_ik_target_2 geometry_msgs/msg/Vector3 "{x: -400, y: -100, z: -150.0}" --once &
