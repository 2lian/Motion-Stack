from typing import Any, Dict
from launch_ros.actions import Node
from launch import LaunchDescription

PACKAGE_NAME = "easy_robot_control"

from launch_setting import LEG_EE_LIST
from launch_setting import params
parameters: Dict[str, Any] = params

movement_node_list = [
    Node(
        package=PACKAGE_NAME,
        namespace="",  # Default namespace
        executable="leg_node",
        name=f"leg_{leg_index}",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[parameters | {"leg_number": leg_index}],
    )
    for leg_index, ee_name in enumerate(LEG_EE_LIST)
]


def generate_launch_description():
    return LaunchDescription(movement_node_list)
