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

NAMESPACES = [""]
# NAMESPACES = [f"r{i+1}" for i in range(8)]  # use this to launch several robots


def getLauncherFromPKG(pkgName: str, launchFileName: str, prefix: str) -> list:
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(get_package_share_directory(pkgName), "launch"),
                    f"/{launchFileName}",
                ]
            ),
            launch_arguments={"prefix": str(prefix)}.items(),
        )
    ]


all_launch_descriptions = []
for namespace in NAMESPACES:
    prefix = f"{namespace}/" if namespace != "" else ""
    stack = getLauncherFromPKG("easy_robot_control", "lvl_04_mover.py", prefix)
    rviz = getLauncherFromPKG("rviz_basic", "rviz.launch.py", prefix)
    # rviz = []
    fused_launch_desc = rviz + stack

    with_namespace = [
        GroupAction(actions=[PushRosNamespace(namespace), description])
        for description in fused_launch_desc
    ]
    all_launch_descriptions += with_namespace


def generate_launch_description():
    return LaunchDescription(all_launch_descriptions)
