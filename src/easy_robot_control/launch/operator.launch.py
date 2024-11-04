from os import environ
from launch import LaunchDescription
from launch_ros.actions import Node

operator = str(environ.get("OPERATOR"))
ns = f"/{operator}"

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                namespace=ns,
                parameters=[{
                    "device_id": int(0),
                    "dead_zone":0.1,
                    "autorepeat_rate": 0.0,
                    }],
            ),
            Node(
                package="easy_robot_control",
                namespace="",
                executable="keygait_node",
            ),
            Node(
                package="keyboard",
                namespace=ns,
                executable="keyboard",
            ),
        ]  # all nodes in this list will run in their own thread
    )
