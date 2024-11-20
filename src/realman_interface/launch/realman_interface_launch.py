import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="realman_interface",
                executable="interface_node",
                name="realman_interface",
                output="screen",
                # No parameters are passed since they're hardcoded
            )
        ]
    )
