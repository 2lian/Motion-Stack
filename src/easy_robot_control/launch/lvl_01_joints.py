from typing import Any, Dict
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "easy_robot_control"

from launch_setting import LEG_EE_LIST
from launch_setting import params
parameters: Dict[str, Any] = params



def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    prefix_value = LaunchConfiguration("prefix", default="")
    prefix_arg = DeclareLaunchArgument("prefix", default_value="")
    node = Node(
        package=PACKAGE_NAME,
        namespace="",  # Default namespace
        executable="joint_node",
        name=f"joint_node",
        arguments=["--ros-args", "--log-level", "info"],
        emulate_tty=True,
        output="screen",
        parameters=[parameters],
    )
    return LaunchDescription([prefix_arg, node])
