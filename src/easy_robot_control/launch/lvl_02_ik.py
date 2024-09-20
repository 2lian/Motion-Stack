from typing import Any, Dict
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription

PACKAGE_NAME = "easy_robot_control"

from launch_setting import LEG_EE_LIST
from launch_setting import params

parameters: Dict[str, Any] = params

ik_node_list = [
    Node(
        package=PACKAGE_NAME,
        namespace="",  # Default namespace
        executable="ik_heavy_node",
        name=f"ik_{leg_index}",
        arguments=["--ros-args", "--log-level", "info"],
        emulate_tty=True,
        output="screen",
        parameters=[
            parameters | {"leg_number": leg_index, "end_effector_name": str(ee_name)}
        ],
    )
    for leg_index, ee_name in enumerate(LEG_EE_LIST)
]

nodeList = ik_node_list


def generate_launch_description():
    return LaunchDescription(nodeList)
