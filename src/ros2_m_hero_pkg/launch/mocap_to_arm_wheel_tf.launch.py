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
                        # "simulation_mode": LaunchConfiguration(
                        #     "simulation_mode", default="False"
                        # ),
                        # "arm_mocap_frame": LaunchConfiguration(
                        #     "arm_mocap_frame", default="mocap4gripper1"
                        # ),
                        # "arm_frame": LaunchConfiguration(
                        #     "arm_frame", default="leg4gripper1"
                        # ),
                        # "arm_offset_translation": LaunchConfiguration(
                        #     "arm_offset_translation", default="[0.0, 0.0, 0.0]"
                        # ),
                        # "arm_offset_rotation_rvec": LaunchConfiguration(
                        #     "arm_offset_rotation_rvec", default="[0.0, 0.0, np.pi/2]"
                        # ),
                        # Wheel parameters
                        # "wheel_mocap_frame": LaunchConfiguration(
                        #     "wheel_mocap_frame", default="mocap14_body"
                        # ),
                        # "wheel_frame": LaunchConfiguration(
                        #     "wheel_frame", default="wheel14_body"
                        # ),
                        # "wheel_offset_translation": LaunchConfiguration(
                        #     "wheel_offset_translation", default="[0.0, 0.0, 0.0]"
                        # ),
                        # "wheel_offset_rotation_rvec": LaunchConfiguration(
                        #     "wheel_offset_rotation_rvec", default="[0.0, 0.0, np.pi/2]"
                        # ),
                        # Additional anchor offset (mocap11_body_offset)
                        # "wheel_offset_anchor_frame": LaunchConfiguration(
                        #     "wheel_offset_anchor_frame", default="mocap14_body_offset"
                        # ),
                        # "wheel_offset_anchor_translation": LaunchConfiguration(
                        #     "wheel_offset_anchor_translation", default="[0.0, 0.0, 0.2]"
                        # ),
                        # "wheel_offset_anchor_rotation_rvec": LaunchConfiguration(
                        #     "wheel_offset_anchor_rotation_rvec",
                        #     default="[0.0, 0.0, 0.0]",
                        # ),
                        # End-Effector MOCAP in simulation
                        # "eef_mocap_frame": LaunchConfiguration(
                        #     "eef_mocap_frame", default="mocap4gripper2_straight"
                        # ),
                        # "eef_urdf_frame": LaunchConfiguration(
                        #     "eef_urdf_frame", default="leg4gripper2_straight"
                        # ),
                        # "eef_offset_translation": LaunchConfiguration(
                        #     "eef_offset_translation", default="[0.2, 0.0, 0.0]"
                        # ),
                        # "eef_offset_rotation_rvec": LaunchConfiguration(
                        #     "eef_offset_rotation_rvec", default="[0.0, 0.0, 0.0]"
                        # ),
                    }
                ],
            )
        ]
    )
