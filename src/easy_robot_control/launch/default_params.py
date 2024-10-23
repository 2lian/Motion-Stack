from typing import Any, Dict, Iterable
from typing import List, Union

# V Default parameters here V
#   \  /   #
#    \/    #
THIS_PACKAGE_NAME = "easy_robot_control"
ROS2_PACKAGE_WITH_URDF = "urdf_packer"

default_params: Dict[str, Any] = {
    "robot_name": None,  # set this in your own launcher
    "urdf_path": None,  # set this in your own launcher
    "number_of_legs": None,  # set this in your own launcher
    "leg_list": [0],  # set this in your own launcher
    "std_movement_time": 2,
    "mvmt_update_rate": 2.0,
    "control_rate": 30.0,
    "start_coord": [0 / 1000, 0 / 1000, 0 / 1000],
    "mirror_angle": False,
    "always_write_position": False,  # deprecated ?
    "start_effector_name": "",  # setting this, works for the IK, but not for Rviz and ros2's tf. In ros, the baselink must be the root and cannot have a parent and there can only be one baselink. Letting this empty and properly setting your URDF baselink is recommended.
    "wheel_size_mm": 230,
    "pure_topic_remap": False,  # activates the pure_remap.py remapping
    "speed_mode": False,
    "WAIT_FOR_LOWER_LEVEL": True,  # waits for nodes of lower level before initializing
    "ignore_limits": False,
    "limit_margin": 0.0,
}

# List link names. Those will be used as end effectors (EE) for each ik nodes
# if a integer number N is given, the last link of the Nth longest kinematic
# chain will be used as the EE of the IK node
LEG_END_EFF: Iterable[Union[str, int]]  # set this in you own launcher
# an easy way to do it is `range(default_params["number_of_legs"])` >> [0,1,2,3]

# the refresh rate of the joint node will not fall below this value if speed_mode = True
JOINT_SPEED_MODE_MIN_RATE = 30.0
#    /\    #
#   /  \   #
# ^ Default parameters here ^


def get_xacro_path(robot_name: str):
    """retieves the .urdf/.xacro path in the install share folder

    Args:
        robot_name: corresponds to <robot_name>.xacro

    Returns:

    """
    from os.path import join
    from ament_index_python.packages import get_package_share_directory

    return join(
        get_package_share_directory(ROS2_PACKAGE_WITH_URDF),
        "urdf",
        f"{robot_name}",
        f"{robot_name}.xacro",
    )


def enforce_params_type(parameters: Dict[str, Any]) -> None:
    """enforces types to dic in place

    Args:
        parameters: ros2 parameters dictinary
    """
    parameters["std_movement_time"] = float(parameters["std_movement_time"])
    parameters["mvmt_update_rate"] = float(parameters["mvmt_update_rate"])
    parameters["start_coord"] = [float(x) for x in parameters["start_coord"]]
    parameters["mirror_angle"] = bool(parameters["mirror_angle"])
    parameters["robot_name"] = str(parameters["robot_name"])
    parameters["urdf_path"] = str(parameters["urdf_path"])
    parameters["always_write_position"] = bool(parameters["always_write_position"])
    parameters["start_effector_name"] = str(parameters["start_effector_name"])
    parameters["wheel_size_mm"] = float(parameters["wheel_size_mm"])
    parameters["number_of_legs"] = int(parameters["number_of_legs"])
    parameters["pure_topic_remap"] = bool(parameters["pure_topic_remap"])
    parameters["speed_mode"] = bool(parameters["speed_mode"])
    parameters["WAIT_FOR_LOWER_LEVEL"] = bool(parameters["WAIT_FOR_LOWER_LEVEL"])
    parameters["ignore_limits"] = bool(parameters["ignore_limits"])
    parameters["limit_margin"] = float(parameters["limit_margin"])
    parameters["control_rate"] = float(parameters["control_rate"])


RVIZ_REMAP = [
    ("joint_states", "/joint_states"),
    ("joint_commands", "/joint_commands"),
    # ("smooth_body_rviz", "/smooth_body_rviz"),
    # ("robot_body", "/robot_body"),
    ("rviz_interface_alive", "/rviz_interface_alive"),
]
