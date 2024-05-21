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
from launch_setting import urdf_path

LegCount = launch_setting.number_of_legs
Robot = launch_setting.moonbot_leg

ik_node_list = [
    Node(
        package=PACKAGE_NAME,
        namespace="",  # Default namespace
        executable="ik_heavy_node",
        name=f"ik_{leg}",
        arguments=["--ros-args", "--log-level", "info"],
        emulate_tty=True,
        output = "screen",
        parameters=[
            {
                "leg_number": leg,
                # "bodyToCoxa": robot.bodyToCoxa,
                # "coxaLength": robot.coxaLength,
                # "femurLength": robot.femurLength,
                # "tibiaLength": robot.tibiaLength,
                # "coxaMax": robot.coxaMax,
                # "coxaMin": robot.coxaMin,
                # "femurMax": robot.femurMax,
                # "femurMin": robot.femurMin,
                # "tibiaMax": robot.tibiaMax,
                # "tibiaMin": robot.tibiaMin,
                "urdf_path": str(urdf_path),
            }
        ],
    )
    for leg in range(LegCount)
]

nodeList = ik_node_list


def generate_launch_description():
    return LaunchDescription(nodeList)
