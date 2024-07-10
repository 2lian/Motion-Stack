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

other_nodes = [
    Node(
        package=PACKAGE_NAME,
        namespace="",  # Default namespace
        executable="mover_node",
        name="mover",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[
            {
                "std_movement_time": float(launch_setting.std_movement_time),
                "movement_update_rate": float(launch_setting.movement_update_rate),
                "number_of_legs": int(launch_setting.number_of_legs),
            }
        ],
    )
]

previousLaunchDesc = [
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory(PACKAGE_NAME), "launch"),
                "/lvl_03_leg.py",
            ]
        )
    )
]

previousLaunchDesc = []


def generate_launch_description():
    return LaunchDescription(previousLaunchDesc + other_nodes)
