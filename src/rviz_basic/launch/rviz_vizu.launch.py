from typing import List

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

PACKAGE_NAME = "urdf_packer"
ROBOT_NAME_DEFAULT = "moonbot_hero"


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=[
                    f"-d",
                    "./src/rviz_basic/rviz2/urdf_vis.rviz",
                ],
            ),
        ]  # all nodes in this list will run in their own thread
    )
