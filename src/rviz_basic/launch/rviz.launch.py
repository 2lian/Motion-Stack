from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import (
    get_package_share_directory,
)
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = "rviz_basic"
# ROBOT_NAME = "moonbot_7"
ROBOT_NAME = "moonbot_hero"
# ROBOT_NAME = "hero_3wheel_1hand"


def generate_launch_description():
    # urdf_file_name = f"{URDF_NAME}.urdf"
    # urdf = os.path.join("./src/rviz_basic/urdf/", urdf_file_name)
    # urdf_path = get_package_share_directory(PACKAGE_NAME) + f"/urdf/{URDF_NAME}.urdf"

    xacro_path = (
        get_package_share_directory(PACKAGE_NAME)
        + f"/urdf/{ROBOT_NAME}/{ROBOT_NAME}.xacro"
    )

    # xacro_out_path = (
        # get_package_share_directory(PACKAGE_NAME)
        # + f"/urdf/{ROBOT_NAME}/{ROBOT_NAME}.urdf"
    # )

    # with open(urdf, "r") as file:
        # urdf_content = file.read()

    rviz_config_file_name = "urdf_vis.rviz"
    rviz_config = os.path.join("./src/rviz_basic/rviz2/", rviz_config_file_name)

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    prefix_value = LaunchConfiguration("prefix", default="")
    prefix_arg = DeclareLaunchArgument("prefix", default_value="")

    # this will result in f"{prefix_value}base_link"
    baselink_with_prefix_arg = DeclareLaunchArgument(
        "baselink_with_prefix", default_value=[prefix_value, "base_link"]
    )
    baselink_with_prefix_value = LaunchConfiguration("baselink_with_prefix")

    # c = ExecuteProcess(
    #     cmd=[
    #         # "ls",
    #         f"xacro", f"{ROBOT_NAME}.xacro", " | tee output.txt"
    #         # f"xacro {ROBOT_NAME}.xacro",
    #         # f"xacro {str(xacro_path)} > {str(xacro_out_path)}",
    #     ],
    #     cwd=get_package_share_directory(PACKAGE_NAME) + f"/urdf/{ROBOT_NAME}",
    #     output="both",
    # )
    # c = Command([f"""xacro {str(xacro_path)} > {str(xacro_out_path)}""", ""])
    compiled_xacro = Command([f"""xacro {str(xacro_path)}"""])

    return LaunchDescription(
        [
            prefix_arg,
            baselink_with_prefix_arg,
            Node(
                package="rviz_basic",
                executable="rviz_interface",
                name="rviz_interface",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {
                        "std_movement_time": float(1),
                        "frame_prefix": prefix_value,
                        "urdf_path": str(xacro_path),
                        # "nothing": ParameterValue(c, value_type=str),
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "frame_prefix": prefix_value,
                        "robot_description": compiled_xacro,
                    }
                ],
                # arguments=[urdf],
            ),
            # Node(
            #     package='rviz2',
            #     executable='rviz2',
            #     name='rviz2',
            #     output='screen',
            #     arguments=['-d', rviz_config],
            # ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base_link",
                output="screen",
                arguments=[
                    "0",
                    "0",
                    # "0.0",
                    "0.200",
                    "0",
                    "0",
                    "0",
                    "world",
                    baselink_with_prefix_value,
                ],
            ),
        ]  # all nodes in this list will run in their own thread
    )
