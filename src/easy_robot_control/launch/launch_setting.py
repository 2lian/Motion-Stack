from launch.launch_description import DeclareLaunchArgument
import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
)
# from ament_index_python.packages import get_package_share_directory

std_movement_time = 2  # seconds
movement_update_rate = 30.0  # Hz
number_of_legs = 4
wheel_size = float(1000)

ROS2_PACKAGE_WITH_URDF = "rviz_basic"
# ROBOT_NAME_DEFAULT = "moonbot_7"
# ROBOT_NAME_DEFAULT = "moonbot_45"
ROBOT_NAME_DEFAULT = "moonbot_hero"
# ROBOT_NAME_DEFAULT = "hero_3wheel_1hand"

def make_xacro_path(launchArgName: str = "robot") -> PathJoinSubstitution:
    """
    Basically does this, but using ros2 parameter substitution on launch
    xacro_path = (
        get_package_share_directory(PACKAGE_NAME)
        + f"/urdf/{ROBOT_NAME}/{ROBOT_NAME}.xacro"
    )"""
    robot_name_arg = LaunchConfiguration(launchArgName, default=ROBOT_NAME_DEFAULT)
    robot_name_val = DeclareLaunchArgument(launchArgName, default_value=ROBOT_NAME_DEFAULT)

    xacro_file_path = PathJoinSubstitution(
        [
            get_package_share_directory(ROS2_PACKAGE_WITH_URDF),
            "urdf",
            robot_name_arg,
            PythonExpression(["'", robot_name_arg, ".xacro'"]),
        ]
    )
    return xacro_file_path

xacro_path = make_xacro_path()
