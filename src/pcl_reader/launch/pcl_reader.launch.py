from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = "pcl_reader"

nodeList = [
    Node(
        package=PACKAGE_NAME,
        namespace="",  # Default namespace
        executable="pointcloud_read_pub",
        name=f"pointcloud_read_pub",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[{}],
    )
]


def generate_launch_description():
    return LaunchDescription(nodeList)
