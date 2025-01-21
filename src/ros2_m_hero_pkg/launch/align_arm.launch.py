from os import environ

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

operator = str(environ.get("OPERATOR"))
ns = f"/{operator}"


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_m_hero_pkg",
                executable="align_arm_to_wheel",
                name="align_arm_to_wheel_node",
                # emulate_tty=True,
                prefix="xterm -hold -e",
                output="screen",
                parameters=[
                    {
                        "world_frame": LaunchConfiguration(
                            "world_frame", default="world"
                        ),
                        "safety_offset_z": LaunchConfiguration(
                            "safety_offset_z", default=0.0
                        ),
                        "distance_threshold": LaunchConfiguration(
                            "distance_threshold", default=0.05
                        ),
                        # Arm parameters
                        "arm_end_effector_frame": LaunchConfiguration(
                            "arm_end_effector_frame", default="leg3gripper2_straight"
                        ),
                        # Wheel parameters
                        "wheel_frame": LaunchConfiguration(
                            "wheel_frame", default="wheel11_body"
                        ),
                    }
                ],
            ),
            Node(
                package="keyboard",
                namespace=ns,
                executable="keyboard",
            ),
        ]
    )
