from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_name = "moonbot7"
    urdf_file_name = f"{robot_name}.urdf"
    urdf = os.path.join("./src/rviz_basic/urdf/", urdf_file_name)

    with open(urdf, "r") as file:
        urdf_content = file.read()

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
                        "robot_description": urdf_content,
                    }
                ],
                arguments=[urdf],
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
                # arguments=["0", "0", "0.200", "0", "0", "0", "world", f"{prefix_value}base_link"],
                # arguments=["0", "0", "0.200", "0", "0", "0", "world", prefix_value + baselink_value],
                arguments=[
                    "0",
                    "0",
                    "0.200",
                    "0",
                    "0",
                    "0",
                    "world",
                    baselink_with_prefix_value,
                ],
            ),
        ],  # all nodes in this list will run in their own thread
    )
