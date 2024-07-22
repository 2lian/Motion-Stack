#!/bin/bash
set -e -o pipefail

cd "${ROS2_MOONBOT_WS}" || echo No folder shortcut, working in $PWD
. "${ROS2_INSTALL_PATH}"/setup.bash || . /opt/ros/humble/setup.bash || echo Ros2 Humble not found

# ros2 topic pub /r1/auto_place geometry_msgs/msg/Transform "{translation: {x: 1300, y: 200, z: 1000}, rotation: {x: 0.2, y: 0.0, z: 0.4, w: 1.0}}" --once
# ros2 topic pub /r2/auto_place geometry_msgs/msg/Transform "{translation: {x: -200, y: 0, z: 150}, rotation: {x: -0.15, y: 0.0, z: 0.0, w: 1.0}}" --once
# ros2 topic pub /r3/auto_place geometry_msgs/msg/Transform "{translation: {x: 1100, y: 1100, z: 1200}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" --once
ros2 topic pub /r4/auto_place geometry_msgs/msg/Transform "{translation: {x: 1300, y: -700, z: 800}, rotation: {x: 0.2, y: 0.0, z: 0.0, w: 1.0}}" --once
ros2 topic pub /r5/auto_place geometry_msgs/msg/Transform "{translation: {x: 800, y: -900, z: 500}, rotation: {x: 0.0, y: -0.3, z: 0.0, w: 1.0}}" --once

# ros2 topic pub /r6/auto_place geometry_msgs/msg/Transform "{translation: {x: 200, y: -500, z: 200}, rotation: {x: -0.1, y: 0.0, z: 0.3, w: 1.0}}" --once
# ros2 topic pub /r7/auto_place geometry_msgs/msg/Transform "{translation: {x: -800, y: 200, z: 200}, rotation: {x: -0.2, y: 0.1, z: -0.2, w: 1.0}}" --once
# ros2 topic pub /r8/auto_place geometry_msgs/msg/Transform "{translation: {x: -1400, y: 800, z: 100}, rotation: {x: 0.0, y: -0.2, z: 0.0, w: 1.0}}" --once




# ros2 topic pub /r4/auto_place geometry_msgs/msg/Transform "{translation: {x: 0.2, y: 0.4, z: 0.0}, rotation: {x: 0.0, y: -0.2, z: 0.0, w: 0.8}}" --once
# ros2 topic pub /r5/auto_place geometry_msgs/msg/Transform "{translation: {x: 0.2, y: 0.4, z: 0.0}, rotation: {x: 0.0, y: -0.2, z: 0.0, w: 0.8}}" --once
# ros2 topic pub /r6/auto_place geometry_msgs/msg/Transform "{translation: {x: 0.2, y: 0.4, z: 0.0}, rotation: {x: 0.0, y: -0.2, z: 0.0, w: 0.8}}" --once
# ros2 topic pub /r7/auto_place geometry_msgs/msg/Transform "{translation: {x: 0.2, y: 0.4, z: 0.0}, rotation: {x: 0.0, y: -0.2, z: 0.0, w: 0.8}}" --once
# ros2 topic pub /r8/auto_place geometry_msgs/msg/Transform "{translation: {x: 0.2, y: 0.4, z: 0.0}, rotation: {x: 0.0, y: -0.2, z: 0.0, w: 0.8}}" --once



ros2 topic pub /leg_1_shift custom_messages/srv/TFService "{translation: {x: 1300, y: -700, z: 800}, rotation: {x: 0.2, y: 0.0, z: 0.0, w: 1.0}}" --once
