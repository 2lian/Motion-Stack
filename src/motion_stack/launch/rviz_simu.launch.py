from launch_ros.actions import Node
from launch_ros.substitutions.find_package import get_package_share_directory

from launch import LaunchDescription

REFRESH_RATE = float(60)
MAX_JIONT_SPEED = float(0.3)  # ignored when position control ???

PACKAGE_NAME = "motion_stack"

setting_path = f"{get_package_share_directory(PACKAGE_NAME)}/launch/rviz_settings.rviz"


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package=PACKAGE_NAME,
                executable="mini_sim",
                name="mini_sim",
                emulate_tty=True,
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {
                        "refresh_rate": float(REFRESH_RATE),
                        "max_speed": float(MAX_JIONT_SPEED),
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
        ]
    )
