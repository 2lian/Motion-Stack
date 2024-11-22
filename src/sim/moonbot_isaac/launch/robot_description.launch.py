from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    prefix_value = LaunchConfiguration("prefix", default="")
    xacro_path_arg = LaunchConfiguration("xacro_path")
    compiled_xacro = Command([f"xacro ", xacro_path_arg])

    return LaunchDescription(
        [
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
                    }
                ],
            ),
        ]
    )
