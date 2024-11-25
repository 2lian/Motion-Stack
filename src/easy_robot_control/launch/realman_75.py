"""
launchpy file for RealMan-75 robot arm
"""

from typing import Any, Dict, Iterable, List, Union

import numpy as np
from default_params import *
from launch_ros.actions import Node

params = default_params.copy()  # params loaded from default_params

# V Change default parameters here V
#   \  /   #
#    \/    #
ROBOT_NAME = "realman_75"  # name of the xacro to load
xacro_path = get_xacro_path(ROBOT_NAME)
# remaplvl1 = RVIZ_REMAP
remaplvl1 = []

# leg number -> end effector (number or link name)
LEGS_DIC: Dict[int, Union[str, int]] = {
    75: 0,
}

overwrite_default = {
    "robot_name": ROBOT_NAME,
    "urdf_path": xacro_path,
    "number_of_legs": 1,
    "pure_topic_remap": False,  # activates the pure_remap.py remapping
    "speed_mode": False,
    "ignore_limits": False,
}
params.update(overwrite_default)
#    /\    #
#   /  \   #
# ^ Change default parameters here ^

enforce_params_type(params)

# V LVL1 node setup V
#   \  /   #
#    \/    #
lvl1: List[Node] = []
for leg_index, ee_name in LEGS_DIC.items():
    # there is one lvl1 node per leg
    # (one joint node can also handle several legs/joints if end effector is None,
    # but let's not complicate)
    this_node_param: Dict[str, Any] = params.copy()
    # each node has different parameters,
    # we need to cahnge the leg number and end effector
    this_node_param["leg_number"] = leg_index
    this_node_param["end_effector_name"] = str(ee_name)
    lvl1.append(
        Node(
            package="realman_interface",
            namespace=f"leg{leg_index}",  # separates the topics between different legs
            # with one namespace per leg
            executable="interface_node",
            arguments=["--ros-args", "--log-level", "info"],
            parameters=[this_node_param],
            remappings=remaplvl1 
            # rviz is in global namespace so we remap the output
            # of lvl1 from local namespace (=/.../legX) to global namespace (=/)
            # depending on your stack structure you'll need to change this remap to send
            # the output on the right topic
            ,
        )
    )
#    /\    #
#   /  \   #
# ^ LVL1 node setup ^

# V LVL2 node setup V
#   \  /   #
#    \/    #
lvl2: List[Node] = []
for leg_index, ee_name in LEGS_DIC.items():
    # same process as lvl1
    this_node_param: Dict[str, Any] = params.copy()
    this_node_param["leg_number"] = leg_index
    this_node_param["end_effector_name"] = str(ee_name)
    lvl2.append(
        Node(
            package=THIS_PACKAGE_NAME,
            namespace=f"leg{leg_index}",
            executable="ik_heavy_node",
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
lvl3: List[Node] = []
for leg_index, ee_name in LEGS_DIC.items():
    # same process as lvl1
    this_node_param: Dict[str, Any] = params.copy()
    this_node_param["leg_number"] = leg_index
    this_node_param["end_effector_name"] = str(ee_name)  # not used ?
    lvl3.append(
        Node(
            package=THIS_PACKAGE_NAME,
            namespace=f"leg{leg_index}",
            executable="leg_node",
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
    arguments=["--ros-args", "--log-level", "info"],
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
    name="gait",
    arguments=["--ros-args", "--log-level", "info"],
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
