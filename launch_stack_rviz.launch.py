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
# NAMESPACES = [f"r{i+1}" for i in range(5)]  # use this to launch several robots
# NAMESPACES = [f"r{i+1}" for i in [0,1,2]]  # use this to launch several robots
# NAMESPACES = [f"r{i+1}" for i in [3,4]]  # use this to launch several robots
# NAMESPACES = [f"r{i+1}" for i in [5,6,7]]  # use this to launch several robots
ROBOTS = {
    1: "moonbot_7",
    2: "moonbot_45",
    3: "moonbot_hero",
    4: "moonbot_hero2",
    5: "hero_3wheel_1hand",
}
ROBOT_NAME = ROBOTS[4]


def getLauncherFromPKG(pkgName: str, launchFileName: str, launchArguments: dict) -> list:
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(get_package_share_directory(pkgName), "launch"),
                    f"/{launchFileName}",
                ]
            ),
            # launch_arguments={"prefix": str(prefix)}.items(),
            launch_arguments=launchArguments.items(),
        )
    ]


def add_namespace(launch_desc: list, namespace: str) -> list:
    return [
        GroupAction(actions=[PushRosNamespace(namespace), description])
        for description in launch_desc
    ]


all_launch_descriptions = []
stack_list = []
interface_list = []
for namespace in NAMESPACES:
    prefix = f"{namespace}/" if namespace != "" else ""
    launchArgs = {"prefix": str(prefix), "robot": ROBOT_NAME}
    stack = []
    stack += getLauncherFromPKG("easy_robot_control", "lvl_02_ik.py", launchArgs)
    stack += getLauncherFromPKG("easy_robot_control", "lvl_03_leg.py", launchArgs)
    stack += getLauncherFromPKG("easy_robot_control", "lvl_04_mover.py", launchArgs)
    rviz = getLauncherFromPKG("rviz_basic", "rviz.launch.py", launchArgs)

    stack_list += add_namespace(stack, namespace)
    interface_list += add_namespace(rviz, namespace)

environment = getLauncherFromPKG("pcl_reader", "pcl_reader.launch.py", {})
all_launch_descriptions += interface_list
all_launch_descriptions += stack_list
# all_launch_descriptions += environment


def generate_launch_description():
    return LaunchDescription(all_launch_descriptions)
