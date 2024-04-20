from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def getLauncherFromPKG(pkgName: str, launchFileName: str):
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


def generate_launch_description():
    return LaunchDescription(rviz_launch_desc + mover_launch_desc)
