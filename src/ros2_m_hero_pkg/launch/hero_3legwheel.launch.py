from typing import Any, Dict, Final, Iterable
from typing import List, Union
from launch_ros.actions import Node
import numpy as np
from default_params import *
from mh_unified import *


# V Change default parameters here V
#   \  /   #
#    \/    #
# if MOONBOT_PC_NUMBER.isdigit():
#     MOONBOT_PC_NUMBER = int(MOONBOT_PC_NUMBER)
# else:
#     fallbacknumber = 3
#     print(
#         f"MOONBOT_PC_NUMBER must correspond to leg number. "
#         f"Using {fallbacknumber} instead."
#     )
#     MOONBOT_PC_NUMBER = fallbacknumber


if USE_RVIZ:  # onlly lauinch 1 leg
    LEGS_DIC: Dict[int, Union[str, int]] = {  # leg number -> end effector
        1: 0,
        2: 1,
        3: 2,
        # 4: 0,
        11: "base_link",
        12: "base_link",
        13: "base_link",
        # 14: "14wheel_in",
    }
else:  # tries them all
    LEGS_DIC: Dict[int, Union[str, int]] = {  # leg number -> end effector
        1: 0,
        2: 1,
        3: 2,
        # 4: 0,
        11: "base_link",
        12: "base_link",
        13: "base_link",
        # 14: "14wheel_in",
    }


def is_leg(ind):
    return ind >= 10


LEG_END_EFF: Iterable[Union[str, int]] = get_LEG_EE(LEGS_DIC)
leg_indices: Iterable[int] = get_LEG_IND(LEGS_DIC)

ROBOT_NAME = "hero_7dof"  # just to get the file path
xacro_path = get_xacro_path(ROBOT_NAME)
xacro_path_safe: Final[str] = str(xacro_path)
xacro_path = xacro_path_safe[: -len(f"{ROBOT_NAME}.xacro")] + f"hero_3legwheel.xacro"

ROBOT_NAME = "hero_3legwheel"

params = change_param(default_params)  # params loaded from default_params
overwrite_default = {
    "robot_name": ROBOT_NAME,
    "urdf_path": xacro_path,
    "number_of_legs": len([i for i in leg_indices if not is_leg(i)]),
    "leg_list": [i for i in leg_indices if not is_leg(i)],
    "start_coord": [0 / 1000, 0 / 1000, 500 / 1000],
    "ignore_limits": True,
    "limit_margin": 0.0,
}
params.update(overwrite_default)
print(leg_indices)
print(LEG_END_EFF)
#    /\    #
#   /  \   #
# ^ Change default parameters here ^
enforce_params_type(params)

# V LVL1 node setup V
#   \  /   #
#    \/    #

lvl1: List[Node] = []
someone_handling_baselink = False
for leg_index, ee_name in zip(leg_indices, LEG_END_EFF):
    if 1 not in CASE.lvl_to_launch:
        break
    # changes parameters for this node
    this_node_param: Dict[str, Any] = params.copy()
    if this_node_param["speed_mode"] is True:
        this_node_param["control_rate"] = max(
            float(JOINT_SPEED_MODE_MIN_RATE), this_node_param["mvmt_update_rate"]
        )
    assert leg_index is not None
    this_node_param["leg_number"] = leg_index
    this_node_param["end_effector_name"] = str(ee_name)
    if is_leg(leg_index):
        this_node_param["add_joints"] = [
            f"{leg_index}wheel_left_joint",
            f"{leg_index}wheel_right_joint",
        ]
        this_node_param["start_effector_name"] = ee_name
        this_node_param["speed_mode"] = True

    else:
        this_node_param["add_joints"] = [f"leg{leg_index}grip1", f"leg{leg_index}grip2"]
    if someone_handling_baselink:
        this_node_param["start_coord"] = [np.nan, np.nan, np.nan]
    else:
        someone_handling_baselink = True
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
for leg_index, ee_name in zip(leg_indices, LEG_END_EFF):
    if is_leg(leg_index):
        continue
    if 2 not in CASE.lvl_to_launch:
        break
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
for leg_index, ee_name in zip(leg_indices, LEG_END_EFF):
    if is_leg(leg_index):
        continue
    if 3 not in CASE.lvl_to_launch:
        break
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
this_node_param["leg_list"] = [i for i in this_node_param["leg_list"] if not is_leg(i)]
lvl4: List[Node] = []
if 4 in CASE.lvl_to_launch:
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
this_node_param["leg_list"] = [i for i in this_node_param["leg_list"] if not is_leg(i)]
lvl5: List[Node] = []
if 5 in CASE.lvl_to_launch:
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
