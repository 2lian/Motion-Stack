from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = "easy_robot_control"
LEG_COUNT = 4

# Add the launch directory to the Python path to import the settings without rebuilding
directory_to_add = f"./src/{PACKAGE_NAME}/launch"
sys.path.append(directory_to_add)
import launch_setting

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
            }
        ],
    )
    for leg in range(LEG_COUNT)
]

previousLaunchDesc = [
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory(PACKAGE_NAME), "launch"),
                "/lvl_02_ik.py",
            ]
        )
    )
]


def generate_launch_description():
    return LaunchDescription(previousLaunchDesc + movement_node_list)
