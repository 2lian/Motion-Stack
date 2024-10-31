from launch import LaunchDescription
from launch_ros.actions import Node

ns = "/elian"

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                namespace=ns,
                parameters=[{
                    "device_id": int(1),
                    "dead_zone":0.01,
                    "autorepeat_rate": 0.0,
                    }],
            ),
            Node(
                package="easy_robot_control",
                namespace=ns,
                executable="keygait_node",
            ),
            Node(
                package="keyboard",
                namespace=ns,
                executable="keyboard",
            ),
        ]  # all nodes in this list will run in their own thread
    )
