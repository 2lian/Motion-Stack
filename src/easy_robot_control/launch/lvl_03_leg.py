from typing import Any, Dict, List
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

movement_node_list = [
    Node(
        package=PACKAGE_NAME,
        namespace="",  # Default namespace
        executable="leg_node",
        name=f"""leg_{p["leg_number"]}""",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[p],
    )
    for p in paramList
]


def generate_launch_description():
    return LaunchDescription(movement_node_list)
