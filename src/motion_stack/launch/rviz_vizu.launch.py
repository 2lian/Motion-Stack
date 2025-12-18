from launch_ros.actions import Node
from launch_ros.substitutions.find_package import get_package_share_directory

from launch import LaunchDescription

PACKAGE_NAME = "motion_stack"

setting_path = f"{get_package_share_directory(PACKAGE_NAME)}/launch/rviz_settings.rviz"


def generate_launch_description():
    return LaunchDescription(
        [
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
