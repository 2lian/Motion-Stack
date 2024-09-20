from typing import Any, Dict
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription

from launch_setting import params
parameters: Dict[str, Any] = params

PACKAGE_NAME = "easy_robot_control"


other_nodes = [
    Node(
        package=PACKAGE_NAME,
        namespace="",  # Default namespace
        executable="gait_node",
        name="gait_node",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[parameters],
    )
]


def generate_launch_description():
    return LaunchDescription(other_nodes)
