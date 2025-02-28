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
        "sim_script",
        default_value="run_sim.py",
        description="Environment script to load. Default is run_sim.py",
    )
    sim_config = DeclareLaunchArgument(
        "sim_config",
        default_value="default.toml",
        description="Path to the environment configuration file (relative to the config folder)",
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
                LaunchConfiguration("sim_script")
            ]),
            "headless": LaunchConfiguration("headless"),
            "env_args": ["--sim-config-path=", LaunchConfiguration("sim_config")],
        }.items(),
    )

    #  Convert beteen Isaac and ROS joint names
    isaac_motion_stack_interface = Node(
        package="moonbot_isaac",
        executable="isaac_motion_stack_interface",
        name="isaac_motion_stack_interface",
        output="screen",
    )

    isaac_robot_description_publisher = Node(
        package="moonbot_isaac",
        executable="isaac_robot_description_publisher",
        name="isaac_robot_description_publisher",
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
            sim_config,
            headless_arg,
            sim_environment,
            isaac_motion_stack_interface,
            isaac_robot_description_publisher,
            interface_alive_service,
            tf_ground_truth_republisher,
        ]
    )
