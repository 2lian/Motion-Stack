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

# gets this python file path to load neighboring setting.py
current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
sys.path.append(current_directory)
from general_launch_settings import *


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
    launchArgs = {"prefix": str(prefix), "robot": RobotName}
    stack = []

    for keys, values in MOTION_STACK_LEVEL_LAUNCHERS.items():
        lvlIsTooHigh = keys > LAUNCH_UP_TO_LVL
        if lvlIsTooHigh:
            continue  # skips
        stack += getLauncherFromPKG(MOTION_STACK_PKG_NAME, values, launchArgs)

    for pkgName, launcherName in INTERFACES:
        rviz = getLauncherFromPKG(pkgName, launcherName, launchArgs)
        interface_list += add_namespace(rviz, namespace)

    stack_list += add_namespace(stack, namespace)

environment = getLauncherFromPKG("pcl_reader", "pcl_reader.launch.py", {})
all_launch_descriptions += interface_list
all_launch_descriptions += stack_list
# all_launch_descriptions += environment


def generate_launch_description():
    return LaunchDescription(all_launch_descriptions)
