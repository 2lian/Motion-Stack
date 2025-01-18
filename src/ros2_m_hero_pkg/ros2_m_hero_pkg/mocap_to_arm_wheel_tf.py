#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from easy_robot_control.EliaNode import EliaNode, myMain, np2tf, tf2np
from easy_robot_control.utils.math import Quaternion, qt
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class MocapToArmAndWheelTF(EliaNode):
    def __init__(self):
        super().__init__("mocap_to_arm_wheel_tf_node")

        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -----------------------------------------------------------------
        # 1) Parameters for Arm
        # -----------------------------------------------------------------
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("arm_mocap_frame", "mocap3gripper1")  # Arm’s mocap frame
        self.declare_parameter("arm_frame", "leg3gripper1")  # Arm’s URDF frame
        self.declare_parameter("arm_offset_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("arm_offset_rotation_euler", [0.0, 0.0, 0.0])

        # -----------------------------------------------------------------
        # 2) Parameters for Wheel
        # -----------------------------------------------------------------
        self.declare_parameter("wheel_mocap_frame", "mocap11_body")
        self.declare_parameter("wheel_frame", "wheel11_body")
        self.declare_parameter("wheel_offset_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("wheel_offset_rotation_euler", [0.0, 0.0, 0.0])

        self.simulation_mode = self.get_parameter("simulation_mode").value

        self.arm_mocap_frame = self.get_parameter("arm_mocap_frame").value
        self.arm_frame = self.get_parameter("arm_frame").value
        self.arm_offset_translation = self.get_parameter("arm_offset_translation").value
        self.arm_offset_rotation_euler = self.get_parameter(
            "arm_offset_rotation_euler"
        ).value

        self.wheel_mocap_frame = self.get_parameter("wheel_mocap_frame").value
        self.wheel_frame = self.get_parameter("wheel_frame").value
        self.wheel_offset_translation = self.get_parameter(
            "wheel_offset_translation"
        ).value
        self.wheel_offset_rotation_euler = self.get_parameter(
            "wheel_offset_rotation_euler"
        ).value

        # convert Euler angles to offset quaternions
        self.arm_offset_quat = self.euler_to_quaternion(*self.arm_offset_rotation_euler)
        self.wheel_offset_quat = self.euler_to_quaternion(
            *self.wheel_offset_rotation_euler
        )

        self.arm_update_timer = self.create_timer(0.1, self.update_arm_tf)
        self.wheel_update_timer = self.create_timer(0.1, self.update_wheel_tf)

        # -------------------------------
        # Simulation / mock data
        # -------------------------------
        if self.simulation_mode:
            self.arm_mock_timer = self.create_timer(0.1, self.mock_arm_mocap_tf)
            self.wheel_mock_timer = self.create_timer(0.1, self.mock_wheel_mocap_tf)

            # subscriptions to control mock transforms
            self.pose_subscription_arm = self.create_subscription(
                Pose, "/mock_mocap_control_arm", self.pose_callback_arm, 10
            )
            self.pose_subscription_wheel = self.create_subscription(
                Pose, "/mock_mocap_control_wheel", self.pose_callback_wheel, 10
            )

        # initial mock positions/orientations
        self.mock_arm_position = [0.5, 0.5, 0.05]
        self.mock_arm_orientation = [0.0, 0.0, 0.0]

        self.mock_wheel_position = [0.6, 1.3, 0.3]
        self.mock_wheel_orientation = [0.0, 0.0, 0.0]

        self.pinfo("MocapToArmAndWheelTF node initialized.")

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Convert Euler angles (roll, pitch, yaw) to a quaternion."""
        qx = qt.from_rotation_vector(np.array([roll, 0, 0]))
        qy = qt.from_rotation_vector(np.array([0, pitch, 0]))
        qz = qt.from_rotation_vector(np.array([0, 0, yaw]))
        return qz * qy * qx

    # ------------------------------------------------------------------------
    # Arm MOCAP Simulation
    # ------------------------------------------------------------------------
    def mock_arm_mocap_tf(self):
        """Simulate a 'world -> mocap3gripper1' transform (arm) in simulation."""
        pos_np = np.array(self.mock_arm_position, dtype=float)
        orientation_quat = self.euler_to_quaternion(*self.mock_arm_orientation)

        tf_generic = np2tf(pos_np, orientation_quat)

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "world"
        transform_stamped.child_frame_id = self.arm_mocap_frame
        transform_stamped.transform = tf_generic

        self.tf_broadcaster.sendTransform(transform_stamped)

    def pose_callback_arm(self, msg: Pose):
        """Update the arm's mock position/orientation from a Pose message."""
        self.mock_arm_position = [msg.position.x, msg.position.y, msg.position.z]
        quat = qt.quaternion(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )
        self.mock_arm_orientation = qt.as_euler_angles(quat)

    def update_arm_tf(self):
        """Link 'mocap3gripper1' to 'leg3gripper1' (arm) with the specified offset."""
        try:
            # 1) Lookup transform from 'world' -> 'mocap3gripper1'
            transform = self.tf_buffer.lookup_transform(
                "world", self.arm_mocap_frame, rclpy.time.Time()
            )

            # 2) Convert to NumPy + quaternion using tf2np
            mocap_pos, mocap_quat = tf2np(transform.transform)

            # 3) Combine orientation
            result_quat = mocap_quat * self.arm_offset_quat

            # 4) Build the new transform from 'mocap3gripper1' to 'leg3gripper1'
            #    The translation is purely the offset from parameters, orientation is combined.
            offset_pos = np.array(self.arm_offset_translation, dtype=float)

            new_tf = np2tf(offset_pos, result_quat)

            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = self.arm_mocap_frame
            transform_stamped.child_frame_id = self.arm_frame
            transform_stamped.transform = new_tf

            self.tf_broadcaster.sendTransform(transform_stamped)

        except Exception as e:
            self.pwarn(f"Arm TF error: {e}")

    # ------------------------------------------------------------------------
    # Wheel MOCAP Simulation
    # ------------------------------------------------------------------------
    def mock_wheel_mocap_tf(self):
        """Simulate 'world -> mocap11_body' (wheel) in simulation."""
        pos_np = np.array(self.mock_wheel_position, dtype=float)
        orientation_quat = self.euler_to_quaternion(*self.mock_wheel_orientation)

        tf_generic = np2tf(pos_np, orientation_quat)

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "world"
        transform_stamped.child_frame_id = self.wheel_mocap_frame
        transform_stamped.transform = tf_generic

        self.tf_broadcaster.sendTransform(transform_stamped)

    def pose_callback_wheel(self, msg: Pose):
        """Update the wheel's mock position/orientation from a Pose message."""
        self.mock_wheel_position = [msg.position.x, msg.position.y, msg.position.z]
        quat = qt.quaternion(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )
        self.mock_wheel_orientation = qt.as_euler_angles(quat)

    def update_wheel_tf(self):
        """Link 'mocap11_body' to 'wheel11_body' with the specified offset."""
        try:
            # 1) Lookup transform from 'world' -> 'mocap11_body'
            transform = self.tf_buffer.lookup_transform(
                "world", self.wheel_mocap_frame, rclpy.time.Time()
            )

            # 2) Convert with tf2np
            mocap_pos, mocap_quat = tf2np(transform.transform)

            # 3) Combine orientation
            result_quat = mocap_quat * self.wheel_offset_quat

            # 4) Build new transform from 'mocap11_body' to 'wheel11_body'
            offset_pos = np.array(self.wheel_offset_translation, dtype=float)
            new_tf = np2tf(offset_pos, result_quat)

            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = self.wheel_mocap_frame
            transform_stamped.child_frame_id = self.wheel_frame
            transform_stamped.transform = new_tf

            self.tf_broadcaster.sendTransform(transform_stamped)

        except Exception as e:
            self.pwarn(f"Wheel TF error: {e}")


def main(args=None):
    myMain(MocapToArmAndWheelTF)


if __name__ == "__main__":
    main()
