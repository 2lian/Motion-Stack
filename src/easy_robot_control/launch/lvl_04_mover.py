from typing import Any, Dict
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription

PACKAGE_NAME = "easy_robot_control"

# Add the launch directory to the Python path to import the settings without rebuilding
# directory_to_add = f"./src/{PACKAGE_NAME}/launch"
# sys.path.append(directory_to_add)
from launch_setting import params
parameters: Dict[str, Any] = params.copy()

other_nodes = [
    Node(
        package=PACKAGE_NAME,
        namespace="",  # Default namespace
        executable="mover_node",
        name="mover",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[parameters],
    )
]

def generate_launch_description():
    return LaunchDescription( other_nodes)
