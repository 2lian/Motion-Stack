from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_m_hero_pkg",
                executable="mocap_to_arm_wheel_tf",
                name="mocap_to_arm_wheel_tf_node",
                output="screen",
                parameters=[
                    {
                        "simulation_mode": LaunchConfiguration(
                            "simulation_mode", default="false"
                        ),
                        # Arm parameters
                        "arm_mocap_frame": LaunchConfiguration(
                            "arm_mocap_frame", default="mocap3gripper1"
                        ),
                        "arm_frame": LaunchConfiguration(
                            "arm_frame", default="leg3gripper1"
                        ),
                        "arm_offset_translation": LaunchConfiguration(
                            "arm_offset_translation", default="[0.0, 0.0, 0.0]"
                        ),
                        "arm_offset_rotation_euler": LaunchConfiguration(
                            "arm_offset_rotation_euler", default="[0.0, 0.0, 0.0]"
                        ),
                        # Wheel parameters
                        "wheel_mocap_frame": LaunchConfiguration(
                            "wheel_mocap_frame", default="mocap11_body"
                        ),
                        "wheel_frame": LaunchConfiguration(
                            "wheel_frame", default="wheel11_body"
                        ),
                        "wheel_offset_translation": LaunchConfiguration(
                            "wheel_offset_translation", default="[0.0, 0.0, 0.0]"
                        ),
                        "wheel_offset_rotation_euler": LaunchConfiguration(
                            "wheel_offset_rotation_euler", default="[0.0, 0.0, 0.0]"
                        ),
                    }
                ],
            )
        ]
    )
