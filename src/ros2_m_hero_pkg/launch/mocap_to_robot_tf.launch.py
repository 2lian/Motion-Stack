from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_m_hero_pkg",
                executable="mocap_to_robot_tf",
                name="mocap_to_robot_tf_node",
                output="screen",
                parameters=[
                    {
                        "simulation_mode": LaunchConfiguration(
                            "simulation_mode", default=False
                        ),
                        "mocap_frame": LaunchConfiguration(
                            "mocap_frame", default="mocap3gripper1"
                        ),
                        "robot_frame": LaunchConfiguration(
                            "robot_frame", default="leg3gripper1"
                        ),
                        "offset_translation": LaunchConfiguration(
                            "offset_translation", default="[0.0, 0.0, 0.0]"
                        ),
                        "offset_rotation_euler": LaunchConfiguration(
                            "offset_rotation_euler", default="[0.0, 0.0, 0.0]"
                        ),
                    }
                ],
            )
        ]
    )
