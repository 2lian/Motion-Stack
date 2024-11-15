from typing import List

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

REFRESH_RATE = float(30)
SEND_BACK_ANGLES: bool = True  # /joint_commands messages will be send back on
# /joint_states, also integrating the angular speed
# You must disable this or not launch this file when
# using another interface or the real robot
START_COORD: List[float] = [
    0 / 1000,
    0 / 1000,
    0 / 1000,
]


PACKAGE_NAME = "urdf_packer"
ROBOT_NAME_DEFAULT = "moonbot_hero"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    prefix_value = LaunchConfiguration("prefix", default="")
    prefix_arg = DeclareLaunchArgument("prefix", default_value="")

    xacro_path_val = DeclareLaunchArgument("xacro_path")
    xacro_path_arg = LaunchConfiguration("xacro_path")

    # this will result in f"{prefix_value}base_link"
    baselink_with_prefix_arg = DeclareLaunchArgument(
        "baselink_with_prefix", default_value=[prefix_value, "base_link"]
    )
    baselink_with_prefix_value = LaunchConfiguration("baselink_with_prefix")

    compiled_xacro = Command([f"xacro ", xacro_path_arg])

    return LaunchDescription(
        [
            prefix_arg,
            baselink_with_prefix_arg,
            xacro_path_val,
            Node(
                package="rviz_basic",
                executable="rviz_interface",
                name="rviz_interface",
                emulate_tty=True,
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {
                        "mirror_angles": bool(SEND_BACK_ANGLES),
                        "init_at_zero": True,
                        "refresh_rate": float(REFRESH_RATE),
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                arguments=["--ros-args", "--log-level", "warn"],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "frame_prefix": prefix_value,
                        "robot_description": ParameterValue(
                            compiled_xacro, value_type=str
                        ),
                        # "publish_frequency": REFRESH_RATE,
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
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     name="world_to_base_link",
            #     output="screen",
            #     arguments=[
            #         f"{START_COORD[0]}",
            #         f"{START_COORD[1]}",
            #         f"{START_COORD[2]}",
            #         "0",
            #         "0",
            #         "0",
            #         "world",
            #         baselink_with_prefix_value,
            #     ],
            # ),
        ]  # all nodes in this list will run in their own thread
    )
