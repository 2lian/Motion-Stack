import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Run Isaac Sim in headless mode",
    )
    env_script = DeclareLaunchArgument(
        "env_script",
        default_value="run_sim.py",
        description="Environment script to load. Default is run_sim.py",
    )
    env_args = DeclareLaunchArgument(
        "env_args",
        default_value="",
        description="Additional arguments to pass to the environment script",
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
            "standalone": PathJoinSubstitution([
                FindPackageShare("moonbot_isaac"),
                "environments", 
                LaunchConfiguration("env_script")
            ]),
            "headless": LaunchConfiguration("headless"),
            "env_args": LaunchConfiguration("env_args"),
        }.items(),
    )

    #  Convert beteen Isaac and ROS joint names
    isaac_motion_stack_interface = Node(
        package="moonbot_isaac",
        executable="isaac_motion_stack_interface",
        name="isaac_motion_stack_interface",
        output="screen",
    )

    interface_alive_service = Node(
        package="moonbot_isaac",
        executable="interface_alive_service",
        name="interface_alive_service",
        output="screen",
    )

    tf_ground_truth_republisher = Node(
        package="moonbot_isaac",
        executable="tf_ground_truth_republisher",
        name="tf_ground_truth_republisher",
        output="screen",
    )

    return LaunchDescription(
        [
            SetParameter(name='use_sim_time', value=True),
            env_script,
            env_args,
            headless_arg,
            sim_environment,
            isaac_motion_stack_interface,
            interface_alive_service,
            tf_ground_truth_republisher,
        ]
    )
