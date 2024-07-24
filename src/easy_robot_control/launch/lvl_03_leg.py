from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = "easy_robot_control"

# Add the launch directory to the Python path to import the settings without rebuilding
directory_to_add = f"./src/{PACKAGE_NAME}/launch"
sys.path.append(directory_to_add)
import launch_setting
from launch_setting import xacro_path

LegCount = launch_setting.number_of_legs

movement_node_list = [
    Node(
        package=PACKAGE_NAME,
        namespace="",  # Default namespace
        executable="leg_node",
        name=f"leg_{leg}",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[
            {
                "leg_number": leg,
                "std_movement_time": float(launch_setting.std_movement_time),
                "movement_update_rate": float(launch_setting.movement_update_rate),
                "urdf_path": xacro_path,
            }
        ],
    )
    for leg in range(LegCount)
]

# previousLaunchDesc = [
#     IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             [
#                 os.path.join(get_package_share_directory(PACKAGE_NAME), "launch"),
#                 "/lvl_02_ik.py",
#             ]
#         )
#     )
# ]
#
previousLaunchDesc = []


def generate_launch_description():
    return LaunchDescription(previousLaunchDesc + movement_node_list)
