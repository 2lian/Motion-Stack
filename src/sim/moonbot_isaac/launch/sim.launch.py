import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Run Isaac Sim in headless mode",
    )

    package_share_directory = get_package_share_directory("moonbot_isaac")
    environment_script = os.path.join(
        package_share_directory, "environments", "run_sim.py"
    )

    # Isaac Sim environment
    sim_environment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("isaacsim"),
                "launch",
                "run_isaacsim.launch.py",
            )
        ),
        launch_arguments={
            "standalone": environment_script,
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    #  Convert beteen Isaac and ROS joint names
    joint_state_converter = Node(
        package="moonbot_isaac",
        executable="isaac_joint_name_converter",
        name="isaac_joint_name_converter",
        output="screen",
    )

    interface_alive_service = Node(
        package="moonbot_isaac",
        executable="interface_alive_service",
        name="interface_alive_service",
        output="screen",
    )
    

    return LaunchDescription(
        [
            SetParameter(name='use_sim_time', value=True),
            headless_arg,
            sim_environment,
            joint_state_converter,
            interface_alive_service,
        ]
    )
