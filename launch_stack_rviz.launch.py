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

NAMESPACE = "r3"

prefix = f"{NAMESPACE}/" if NAMESPACE != "" else ""


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


all_launch_desc = []

for i in range(8):
    namespace =f"r{i+1}" 
    prefix = f"{namespace}/"
    mover_launch_desc = getLauncherFromPKG(
        "easy_robot_control", "lvl_04_mover.py", prefix
    )
    rviz_launch_desc = getLauncherFromPKG("rviz_basic", "rviz.launch.py", prefix)

    fused_launch_desc = rviz_launch_desc + mover_launch_desc

    with_namespace = [
        GroupAction(actions=[PushRosNamespace(namespace), description])
        for description in fused_launch_desc
    ]
    all_launch_desc += with_namespace


def generate_launch_description():
    return LaunchDescription(all_launch_desc)
