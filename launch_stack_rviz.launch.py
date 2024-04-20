from launch import LaunchDescription, LaunchContext
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
import sys
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

NAMESPACE = "r1"


def getLauncherFromPKG(pkgName: str, launchFileName: str) -> list:
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(get_package_share_directory(pkgName), "launch"),
                    f"/{launchFileName}",
                ]
            )
        )
    ]


mover_launch_desc = getLauncherFromPKG("easy_robot_control", "lvl_04_mover.py")

rviz_launch_desc = getLauncherFromPKG("rviz_basic", "rviz.launch.py")

fused_launch_desc = rviz_launch_desc + mover_launch_desc

with_namespace = [
    GroupAction(actions=[PushRosNamespace(NAMESPACE), description])
    for description in fused_launch_desc
]


def generate_launch_description():
    return LaunchDescription(with_namespace)
