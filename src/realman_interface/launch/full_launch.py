import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rm_driver_launch = os.path.join(
        get_package_share_directory("rm_driver"), "launch", "rm_75_driver.launch.py"
    )

    realman_interface_launch = os.path.join(
        get_package_share_directory("realman_interface"),
        "launch",
        "realman_interface_launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(rm_driver_launch)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realman_interface_launch)
            ),
        ]
    )
