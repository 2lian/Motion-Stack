# run this inside this folder
cd ${ROS2_MOONBOT_WS}
source ${ROS2_INSTALL_PATH}/setup.bash
colcon build --symlink-install --packages-skip ros1_bridge
. install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="{message}"
export RCUTILS_COLORIZED_OUTPUT=1
#export ROS_DOMAIN_ID=58
ros2 launch src/rviz_custom/launch/rviz.launch.py
