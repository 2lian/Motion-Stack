"""
Settings for Gustavo's arm. feel free to change anything.
See default_params.py for all settings available.
"""

from typing import Any, Dict, Iterable
from typing import List, Union
from launch_ros.actions import Node
from os import environ
from default_params import *

params = default_params.copy()  # params loaded from default_params

# Changes behavior depending on environenemnt variable
# Set USE_RVIZ=TRUE on your PC, FALSE on the robot for example
USE_RVIZ = str(environ.get("USE_RVIZ")) == "TRUE"
# This is used a lot on moonbot hero to change behavior on legs, wheels, ground station
# that's why I love pyhton launchfiles

# V Change default parameters here V
#   \  /   #
#    \/    #

ROBOT_NAME = "mglimb_7dof"  # just to get the file path
LEG_INDICES = [0]  # you only have 1 arm, will be leg #0
LEG_END_EFF = [0]  # automatic

xacro_path = get_xacro_path(ROBOT_NAME)
overwrite_default = {
    "robot_name": ROBOT_NAME,
    "urdf_path": xacro_path,
    "number_of_legs": len(LEG_INDICES),
    "leg_list": LEG_INDICES,
    "start_coord": [0 / 1000, 0 / 1000, 0 / 1000],
    # "ignore_limits": True,
    "pure_topic_remap": False,  # activates the pure_remap.py remapping
    "speed_mode": False,
    "limit_margin": 0.0,
}

# Remapping is better disabled when using Rviz,
# it works, but you might mess Rviz up.
if USE_RVIZ:
    overwrite_default["pure_topic_remap"] = False
else:
    overwrite_default["pure_topic_remap"] = True 

params.update(overwrite_default)
#    /\    #
#   /  \   #
# ^ Change default parameters here ^
enforce_params_type(params)

# V LVL1 node setup V
#   \  /   #
#    \/    #
remaplvl1 = []
if USE_RVIZ:
    remaplvl1 = RVIZ_REMAP

lvl1: List[Node] = []
for leg_index, ee_name in zip(LEG_INDICES, LEG_END_EFF):
    # changes parameters for this node
    this_node_param: Dict[str, Any] = params.copy()
    if this_node_param["speed_mode"] is True:
        this_node_param["control_rate"] = max(
            float(JOINT_SPEED_MODE_MIN_RATE), this_node_param["mvmt_update_rate"]
        )
    assert leg_index is not None
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
            remappings=remaplvl1,
        )
    )
#    /\    #
#   /  \   #
# ^ LVL1 node setup ^

# V LVL2 node setup V
#   \  /   #
#    \/    #
lvl2: List[Node] = []
for leg_index, ee_name in zip(LEG_INDICES, LEG_END_EFF):
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
            name=f"""ik""",
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
for leg_index, ee_name in zip(LEG_INDICES, LEG_END_EFF):
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
lvl4: List[Node] = []
lvl4.append(
    Node(
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
)
#    /\    #
#   /  \   #
# ^ LVL4 node setup ^

# V LVL5 node setup V
#   \  /   #
#    \/    #
this_node_param: Dict[str, Any] = params.copy()
lvl5: List[Node] = []
lvl5.append(
    Node(
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
