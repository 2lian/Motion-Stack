from typing import List

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

REFRESH_RATE = float(30)
SEND_BACK_ANGLES: bool = True  # /joint_commands messages will be send back on
# /joint_states, also integrating the angular speed
# You must disable this or not launch this file when
# using another interface or the real robot


PACKAGE_NAME = "urdf_packer"
ROBOT_NAME_DEFAULT = "moonbot_hero"


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rviz_basic",
                executable="rviz_interface",
                name="rviz_interface",
                emulate_tty=True,
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                remappings=[
                    # ("rviz_sim", "joint_states"),
                ],  # will listen to joint_command not joint_state
                parameters=[
                    {
                        "mirror_angles": bool(SEND_BACK_ANGLES),
                        "init_at_zero": True,
                        "refresh_rate": float(REFRESH_RATE),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    f"-d",
                    "./src/rviz_basic/rviz2/urdf_vis.rviz",
                ],
            ),
        ]  # all nodes in this list will run in their own thread
    )
