import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("realman_interface"),
        "config",
        "realman_interface.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="realman_interface",
                executable="interface_node",
                name="realman_interface",
                parameters=[config_file],
                output="screen",
            )
        ]
    )
