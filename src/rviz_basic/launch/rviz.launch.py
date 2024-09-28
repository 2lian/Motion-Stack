from typing import List
from launch import LaunchDescription
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
)
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import (
    get_package_share_directory,
)
from launch_ros.parameter_descriptions import ParameterValue

REFRESH_RATE = float(60)
MOVEMENT_TIME = float(2)
ALWAYS_WRITE_POSITION: bool = True
SEND_BACK_ANGLES: bool = True
START_COORD: List[float] = [
    0 / 1000,
    0 / 1000,
    0 / 1000,
]


PACKAGE_NAME = "urdf_packer"
ROBOT_NAME_DEFAULT = "moonbot_hero"


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
            get_package_share_directory(PACKAGE_NAME),
            "urdf",
            robot_name_arg,
            PythonExpression(["'", robot_name_arg, ".xacro'"]),
        ]
    )
    return xacro_file_path


def generate_launch_description():
    xacro_path = make_xacro_path()

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    prefix_value = LaunchConfiguration("prefix", default="")
    prefix_arg = DeclareLaunchArgument("prefix", default_value="")

    # this will result in f"{prefix_value}base_link"
    baselink_with_prefix_arg = DeclareLaunchArgument(
        "baselink_with_prefix", default_value=[prefix_value, "base_link"]
    )
    baselink_with_prefix_value = LaunchConfiguration("baselink_with_prefix")

    compiled_xacro = Command([f"xacro ", xacro_path])

    return LaunchDescription(
        [
            prefix_arg,
            baselink_with_prefix_arg,
            Node(
                package="rviz_basic",
                executable="rviz_interface",
                name="rviz_interface",
                emulate_tty=True,
                output = "screen",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[{
                    "mirror_angles": True,
                    "init_at_zero": True,
                    "refresh_rate": float(REFRESH_RATE),
                    }],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "frame_prefix": prefix_value,
                        "robot_description": ParameterValue(
                            compiled_xacro, value_type=str
                        ),
                        "publish_frequency": REFRESH_RATE,
                    }
                ],
                remappings=[
                    # (intside node, outside node),
                    # ("/joint_states", "/rviz_commands"),
                    ("/joint_states", "/rviz_commands"),
                ],  # will listen to joint_command not joint_state
                # not tested with multi robot, will break
                # arguments=[urdf],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base_link",
                output="screen",
                arguments=[
                    f"{START_COORD[0]}",
                    f"{START_COORD[1]}",
                    f"{START_COORD[2]}",
                    "0",
                    "0",
                    "0",
                    "world",
                    baselink_with_prefix_value,
                ],
            ),
        ]  # all nodes in this list will run in their own thread
    )
