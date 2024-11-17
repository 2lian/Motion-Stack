import importlib.util
import os
import sys
from os.path import join
from typing import Iterable, List, Optional, Union

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

# gets this python file path to load neighboring setting.py
current_file_path = os.path.abspath(__file__)
current_directory = os.path.dirname(current_file_path)
sys.path.append(current_directory)
from general_launch_settings import *

def import_module_from_path(module_name, file_path):
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module  # Add the module to sys.modules
    spec.loader.exec_module(module)
    return module

robot_settings = import_module_from_path(LAUNCHPY, join(node_maker, f"{LAUNCHPY}.py"))

try:
    p = robot_settings.params["urdf_path"]
    p = robot_settings.params["urdf_path"]
    levels: List[List[Node]] = robot_settings.levels
    for lvl in levels:
        if isinstance(lvl, list):
            for no in lvl:
                if isinstance(no, Node):
                    pass
                else:
                    raise TypeError(
                        "After importing {LAUNCHPY}.py, type error, "
                        f"levels must be a List[List[Node]]"
                    )
except NameError:
    raise NameError(
        f"Error, {LAUNCHPY}.py does not define 'params['urdf_path']' or 'levels' variable"
    )


def get_nodes_from_levels(
    levels: List[List[Node]], to_include: Optional[Iterable[int]] = None
) -> List[Node]:
    if to_include is None:
        selected_levels = levels
    else:
        selected_levels = [levels[i] for i in to_include]
    nodes: List[Node] = []
    for lvl in selected_levels:
        for node in lvl:
            nodes.append(node)
    return nodes


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


def add_namespace(launch_desc: List[Union[Action, Node]], namespace: str) -> list:
    """sets a namespace"""
    return [
        GroupAction(actions=[PushRosNamespace(namespace), description])
        for description in launch_desc
    ]


all_launch_descriptions = []
stack_list = []
interface_list = []
for namespace in NAMESPACES:
    prefix = f"{namespace}/" if namespace != "" else ""
    launchArgs = {
        "prefix": str(prefix),
        "xacro_path": str(robot_settings.params["urdf_path"]),
    }
    stack = get_nodes_from_levels(robot_settings.levels, LIST_OF_LVL)

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
