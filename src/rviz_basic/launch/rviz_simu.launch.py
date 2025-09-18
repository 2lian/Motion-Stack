from launch_ros.actions import Node
from launch_ros.substitutions.find_package import get_package_share_directory

from launch import LaunchDescription

REFRESH_RATE = float(200)
SEND_BACK_ANGLES: bool = True  # /joint_commands messages will be send back on
# /joint_states, also integrating the angular speed
# You must disable this or not launch this file when
# using another interface or the real robot


PACKAGE_NAME = "rviz_basic"
ROBOT_NAME_DEFAULT = "moonbot_hero"

setting_path = f"{get_package_share_directory(PACKAGE_NAME)}/launch/rviz_settings.rviz"


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rviz_basic",
                executable="rviz_interface",
                name="rviz_interface",
                emulate_tty=True,
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                remappings=[
                    # ("rviz_sim", "joint_states"),
                ],  # will listen to joint_command not joint_state
                parameters=[
                    {
                        "refresh_rate": float(REFRESH_RATE),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    f"-d",
                    setting_path,
                ],
            ),
        ]  # all nodes in this list will run in their own thread
    )
