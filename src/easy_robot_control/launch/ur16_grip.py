from typing import Any, Dict, Iterable
from typing import List, Union
from launch.launch_description import DeclareLaunchArgument
from launch_ros.actions import Node
import numpy as np
from default_params import *


params = default_params.copy()  # params loaded from default_params

# V Change default parameters here V
#   \  /   #
#    \/    #
ROBOT_NAME = "ur16_3f"
xacro_path = get_xacro_path(ROBOT_NAME)

overwrite_default = {
    "robot_name": ROBOT_NAME,
    "urdf_path": xacro_path,
    "number_of_legs": 1,
    "start_effector_name": "base_link",
    "pure_topic_remap": False,  # activates the pure_remap.py remapping
    "speed_mode": False,
    "ignore_limits": True,
}
params.update(overwrite_default)

LEG_END_EFF: Iterable[str] = ["palm"]
#    /\    #
#   /  \   #
# ^ Change default parameters here ^

enforce_params_type(params)

# V LVL1 node setup V
#   \  /   #
#    \/    #

# changes parameters for lvl1
this_node_param: Dict[str, Any] = params.copy()
# prepares the node
lvl1: List[Node] = []
for leg_index, ee_name in enumerate(LEG_END_EFF):
    this_node_param["leg_number"] = leg_index
    this_node_param["end_effector_name"] = str(ee_name)
    lvl1.append(
        Node(
            package=THIS_PACKAGE_NAME,
            namespace=f"leg{leg_index}",
            executable="joint_node",
            name=f"joint_node",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[this_node_param],
            remappings=[
                ("joint_states", "/joint_states"),
                ("joint_commands", "/joint_commands"),
                ("smooth_body_rviz", "/smooth_body_rviz"),
                ("robot_body", "/robot_body"),
                ("rviz_interface_alive", "/rviz_interface_alive"),
            ],
        )
    )
#    /\    #
#   /  \   #
# ^ LVL1 node setup ^

# V LVL2 node setup V
#   \  /   #
#    \/    #

# Makes a level composed of several nodes
lvl2: List[Node] = []
for leg_index, ee_name in enumerate(LEG_END_EFF):
    # changes parameters for this node
    this_node_param: Dict[str, Any] = params.copy()
    this_node_param["leg_number"] = leg_index
    this_node_param["end_effector_name"] = str(ee_name)
    # prepares the node
    lvl2.append(
        Node(
            package=THIS_PACKAGE_NAME,
            namespace=f"leg{leg_index}",
            executable="ik_heavy_node",
            name=f"""ik_{this_node_param["leg_number"]}""",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[this_node_param],
            remappings=[],
        )
    )
#    /\    #
#   /  \   #
# ^ LVL2 node setup ^

# V LVL3 node setup V
#   \  /   #
#    \/    #

# Makes a level composed of several nodes
lvl3: List[Node] = []
for leg_index, ee_name in enumerate(LEG_END_EFF):
    # changes parameters for this node
    this_node_param: Dict[str, Any] = params.copy()
    this_node_param["leg_number"] = leg_index
    this_node_param["end_effector_name"] = str(ee_name)  # not used ?
    # prepares the node
    lvl3.append(
        Node(
            package=THIS_PACKAGE_NAME,
            namespace=f"leg{leg_index}",
            executable="leg_node",
            name=f"leg",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[this_node_param],
            remappings=[],
        )
    )
#    /\    #
#   /  \   #
# ^ LVL3 node setup ^

# V LVL4 node setup V
#   \  /   #
#    \/    #
this_node_param: Dict[str, Any] = params.copy()
lvl4: Node = Node(
    package=THIS_PACKAGE_NAME,
    namespace="",
    executable="mover_node",
    name="mover",
    arguments=["--ros-args", "--log-level", "info"],
    emulate_tty=True,
    output="screen",
    parameters=[this_node_param],
    remappings=[],
)
#    /\    #
#   /  \   #
# ^ LVL4 node setup ^

# V LVL5 node setup V
#   \  /   #
#    \/    #
this_node_param: Dict[str, Any] = params.copy()
lvl5: Node = Node(
    package=THIS_PACKAGE_NAME,
    namespace="",
    executable="gait_node",
    name="gait_node",
    arguments=["--ros-args", "--log-level", "info"],
    emulate_tty=True,
    output="screen",
    parameters=[this_node_param],
    remappings=[],
)
#    /\    #
#   /  \   #
# ^ LVL5 node setup ^

# finally we make a list where index 0 is lvl1 ect..
# each level is composed of a list of node
levels: List[List[Node]] = [
    # [lvl1],
    # lvl2,
    # lvl3,
    # [lvl4],
    # [lvl5],
]
for lvl in [lvl1, lvl2, lvl3, lvl4, lvl5]:
    if isinstance(lvl, Node):
        levels.append([lvl])
    elif isinstance(lvl, list):
        levels.append(lvl)
    else:
        raise TypeError("lvl 1-2-3-4-5 ... must be either a Node or a List[Node]")
