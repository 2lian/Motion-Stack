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
                output="screen",
                parameters=[
                    {
                        # "ee_mocap_frame": LaunchConfiguration(
                        #     "ee_mocap_frame", default="mocap4gripper2_straight"
                        # ),
                        # "wheel_mocap_frame": LaunchConfiguration(
                        #     "wheel_mocap_frame", default="mocap14_body_offset"
                        # ),
                        # "world_frame": LaunchConfiguration(
                        #     "world_frame", default="world"
                        # ),
                        # "coarse_threshold": LaunchConfiguration(
                        #     "coarse_threshold", default="0.01"
                        # ),
                        # "fine_threshold": LaunchConfiguration(
                        #     "fine_threshold", default="0.002"
                        # ),
                        # "orient_threshold_coarse": LaunchConfiguration(
                        #     "orient_threshold_coarse", default="0.1"
                        # ),
                        # "orient_threshold_fine": LaunchConfiguration(
                        #     "orient_threshold_fine", default="0.05"
                        # ),
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
