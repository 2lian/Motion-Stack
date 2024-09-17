from launch import LaunchDescription
from launch_ros.actions import Node
# import sys
# import os
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = "easy_robot_control"

from launch_setting import *

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    prefix_value = LaunchConfiguration("prefix", default="")
    prefix_arg = DeclareLaunchArgument("prefix", default_value="")
    return LaunchDescription(
        [
            prefix_arg,
            Node(
                package=PACKAGE_NAME,
                namespace="",  # Default namespace
                executable="joint_node",
                name=f"joint_node",
                arguments=["--ros-args", "--log-level", "info"],
                emulate_tty=True,
                output="screen",
                parameters=[
                    {
                        "std_movement_time": float(std_movement_time),
                        "start_coord": START_COORD,
                        "mvmt_update_rate": movement_update_rate,
                        "mirror_angle": MIRROR_ANGLE,
                        "frame_prefix": prefix_value, # really usefull ?
                        "urdf_path": xacro_path,
                        "always_write_position": False,
                    }
                ],
            )
        ]
    )
