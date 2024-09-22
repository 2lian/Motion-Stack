from typing import List, Union
from launch.launch_description import DeclareLaunchArgument
import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

# from ament_index_python.packages import get_package_share_directory

MOVEMENT_TIME = 2  # seconds
MOVEMENT_RATE = 2.0  # Hz
SPEED_MODE = True
JOINT_SPEED_MODE_MIN_RATE = 60
LEG_COUNT: int = 1  # Optional
# List link names. Those will be used as end effectors (EE) for each ik nodes
# if a number N is given, the last link of the Nth longest kinematic chain will be used
# as the EE of the IK node
LEG_EE_LIST: List[Union[int, str]] = list(range(LEG_COUNT))
LEG_COUNT: int = len(LEG_EE_LIST)
WHEEL_SIZE = float(230)  # mm

# leave empty for automatic
BASE_LINK = ""

ROS2_PACKAGE_WITH_URDF = "urdf_packer"
ROBOT_NAME_DEFAULT = "moonbot_hero"

MIRROR_ANGLE: bool = False
START_COORD: List[float] = [
    0 / 1000,
    0 / 1000,
    300 / 1000,
]



def make_xacro_path(launchArgName: str = "robot") -> PathJoinSubstitution:
    """
    Basically does this, but using ros2 parameter substitution on launch
    xacro_path = (
        get_package_share_directory(PACKAGE_NAME)
        + f"/urdf/{ROBOT_NAME}/{ROBOT_NAME}.xacro"
    )"""
    robot_name_arg = LaunchConfiguration(launchArgName, default=ROBOT_NAME_DEFAULT)
    robot_name_val = DeclareLaunchArgument(
        launchArgName, default_value=ROBOT_NAME_DEFAULT
    )

    xacro_file_path = PathJoinSubstitution(
        [
            get_package_share_directory(ROS2_PACKAGE_WITH_URDF),
            "urdf",
            robot_name_arg,
            PythonExpression(["'", robot_name_arg, ".xacro'"]),
            # very unsecure, but who is gonna hack you with a robot name ??
        ]
    )
    return xacro_file_path


xacro_path: PathJoinSubstitution = make_xacro_path()

params = {
    "std_movement_time": float(MOVEMENT_TIME),
    "mvmt_update_rate": MOVEMENT_RATE,
    "start_coord": START_COORD,
    "mirror_angle": MIRROR_ANGLE,
    "urdf_path": xacro_path,
    "always_write_position": False,  # deprecated ?
    "start_effector_name": BASE_LINK,
    # "end_effector_name": str(leg),
    "wheel_size_mm": float(WHEEL_SIZE),
    "number_of_legs": int(LEG_COUNT),
    "number_of_legs": int(LEG_COUNT),
    "speed_mode": SPEED_MODE,
}
