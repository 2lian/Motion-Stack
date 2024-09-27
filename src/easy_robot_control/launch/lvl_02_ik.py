from typing import Any, Dict, List
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription

PACKAGE_NAME = "easy_robot_control"

from launch_setting import LEG_EE_LIST
from launch_setting import params

parameters: Dict[str, Any] = params.copy()
paramList: List[Dict[str, Any]] = [parameters] * len(LEG_EE_LIST)
for leg_index, ee_name in enumerate(LEG_EE_LIST):
    p = paramList[leg_index]
    p["leg_number"] = leg_index
    p["end_effector_name"] = str(ee_name)

ik_node_list = [
    Node(
        package=PACKAGE_NAME,
        namespace="",  # Default namespace
        executable="ik_heavy_node",
        name=f"""ik_{p["leg_number"]}""",
        arguments=["--ros-args", "--log-level", "info"],
        emulate_tty=True,
        output="screen",
        parameters=[p],
    )
    for p in paramList
]

nodeList = ik_node_list


def generate_launch_description():
    return LaunchDescription(nodeList)
