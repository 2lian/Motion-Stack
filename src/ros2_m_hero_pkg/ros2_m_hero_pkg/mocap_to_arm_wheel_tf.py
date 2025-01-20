#!/usr/bin/env python3

import math

import numpy as np
import rclpy

# From EliaNode, we get the Node class, utilities (tf2np, np2tf), and myMain
from easy_robot_control.EliaNode import EliaNode, myMain, np2tf, tf2np
from easy_robot_control.utils.math import Quaternion, qt
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class MocapToArmAndWheelTF(EliaNode):
    def __init__(self):
        super().__init__("mocap_to_arm_wheel_tf_node")

        # For publishing TF frames
        self.tf_broadcaster = TransformBroadcaster(self)

        # TF buffer & listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -----------------------------------------------------------------
        # 1) Parameters for Arm (mocap -> URDF)
        # -----------------------------------------------------------------
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("arm_mocap_frame", "mocap3gripper1")  # Arm’s mocap frame
        self.declare_parameter("arm_frame", "leg3gripper1")  # Arm’s URDF frame
        self.declare_parameter("arm_offset_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("arm_offset_rotation_euler", [0.0, 0.0, 0.0])

        # -----------------------------------------------------------------
        # 2) Parameters for Wheel (mocap -> URDF)
        # -----------------------------------------------------------------
        self.declare_parameter("wheel_mocap_frame", "mocap11_body")
        self.declare_parameter("wheel_frame", "wheel11_body")
        self.declare_parameter("wheel_offset_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("wheel_offset_rotation_euler", [0.0, 0.0, 0.0])

        # -----------------------------------------------------------------
        # 3) Parameters for Wheel Offset Frame (mocap11_body_offset)
        # -----------------------------------------------------------------
        # We'll create "mocap11_body_offset" as a child of "mocap11_body"
        # that is translated +0.2 in z and possibly rotated.
        self.declare_parameter("wheel_offset_anchor_frame", "mocap11_body_offset")
        self.declare_parameter(
            "wheel_offset_anchor_translation", [0.0, 0.0, 0.2]
        )  # 20 cm up
        self.declare_parameter("wheel_offset_anchor_rotation_euler", [3.14, 1.57, 0.0])

        # -----------------------------------------------------------------
        # 4) Parameters for End-Effector Mocap
        # -----------------------------------------------------------------
        # For tracking the tip link (leg3gripper2_straight) in simulation
        self.declare_parameter("eef_mocap_frame", "mocap3gripper2_straight")
        self.declare_parameter("eef_urdf_frame", "leg3gripper2_straight")
        self.declare_parameter("eef_offset_translation", [0.2, 0.0, 0.0])
        self.declare_parameter("eef_offset_rotation_euler", [0.0, 0.0, -1.57])

        # Read them in
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

        # (New) Wheel offset anchor
        self.wheel_offset_anchor_frame = self.get_parameter(
            "wheel_offset_anchor_frame"
        ).value
        self.wheel_offset_anchor_translation = self.get_parameter(
            "wheel_offset_anchor_translation"
        ).value
        self.wheel_offset_anchor_rotation_euler = self.get_parameter(
            "wheel_offset_anchor_rotation_euler"
        ).value

        self.eef_mocap_frame = self.get_parameter("eef_mocap_frame").value
        self.eef_urdf_frame = self.get_parameter("eef_urdf_frame").value
        self.eef_offset_translation = self.get_parameter("eef_offset_translation").value
        self.eef_offset_rotation_euler = self.get_parameter(
            "eef_offset_rotation_euler"
        ).value

        # Convert Euler angles to offset quaternions
        self.arm_offset_quat = self.euler_to_quaternion(*self.arm_offset_rotation_euler)
        self.wheel_offset_quat = self.euler_to_quaternion(
            *self.wheel_offset_rotation_euler
        )
        # For the new anchor frame
        self.wheel_offset_anchor_quat = self.euler_to_quaternion(
            *self.wheel_offset_anchor_rotation_euler
        )

        self.eef_offset_quat = self.euler_to_quaternion(*self.eef_offset_rotation_euler)

        # Timers for periodically updating transforms
        self.arm_update_timer = self.create_timer(0.1, self.update_arm_tf)
        self.wheel_update_timer = self.create_timer(0.1, self.update_wheel_tf)
        # new: anchor offset from mocap11_body -> mocap11_body_offset
        self.wheel_offset_anchor_timer = self.create_timer(
            0.1, self.update_wheel_offset_anchor_tf
        )

        # If simulation => update eef_mocap tf
        if self.simulation_mode:
            self.eef_update_timer = self.create_timer(0.1, self.update_eef_mocap_tf)

        # -------------------------------
        # Simulation / mock data for arm & wheel frames
        # -------------------------------
        if self.simulation_mode:
            self.arm_mock_timer = self.create_timer(0.1, self.mock_arm_mocap_tf)
            self.wheel_mock_timer = self.create_timer(0.1, self.mock_wheel_mocap_tf)

            # Subscriptions to control mock transforms
            self.pose_subscription_arm = self.create_subscription(
                Pose, "/mock_mocap_control_arm", self.pose_callback_arm, 10
            )
            self.pose_subscription_wheel = self.create_subscription(
                Pose, "/mock_mocap_control_wheel", self.pose_callback_wheel, 10
            )

        # Initial mock positions/orientations for arm & wheel
        self.mock_arm_position = [0.5, 0.5, 0.05]
        self.mock_arm_orientation = [0.0, 0.0, 0.0]

        self.mock_wheel_position = [0.6, 1.3, 0.3]
        self.mock_wheel_orientation = [0.0, 0.0, 0.0]

        self.pinfo(
            "mocap_to_arm_wheel_tf node initialized with offset anchor for wheel."
        )

    # ------------------------------------------------------------------------
    # Convert Euler angles -> Quaternion
    # ------------------------------------------------------------------------
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
        """Simulate 'world -> mocap3gripper1' transform (arm) in simulation."""
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
        """Link 'mocap3gripper1' -> 'leg3gripper1' with offset."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "world", self.arm_mocap_frame, rclpy.time.Time()
            )
            mocap_pos, mocap_quat = tf2np(transform.transform)

            # Combine orientation
            result_quat = mocap_quat * self.arm_offset_quat
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
        """Link 'mocap11_body' -> 'wheel11_body' with offset."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "world", self.wheel_mocap_frame, rclpy.time.Time()
            )
            mocap_pos, mocap_quat = tf2np(transform.transform)

            # Combine orientation
            result_quat = mocap_quat * self.wheel_offset_quat
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

    # ------------------------------------------------------------------------
    # (NEW) Wheel MOCAP Anchor Offset Frame
    # ------------------------------------------------------------------------
    def update_wheel_offset_anchor_tf(self):
        """
        Publish a child frame "mocap11_body_offset" from "mocap11_body" with a
        20cm upward translation & possible rotation. So you can align the end-eff
        to this offset frame instead of the raw wheel frame.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                "world", self.wheel_mocap_frame, rclpy.time.Time()
            )
            base_pos, base_quat = tf2np(transform.transform)

            # Combine the base orientation with our anchor rotation
            result_quat = base_quat * self.wheel_offset_anchor_quat
            offset_pos = np.array(self.wheel_offset_anchor_translation, dtype=float)

            # Build transform
            new_tf = np2tf(offset_pos, result_quat)
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = self.wheel_mocap_frame
            transform_stamped.child_frame_id = self.wheel_offset_anchor_frame
            transform_stamped.transform = new_tf

            self.tf_broadcaster.sendTransform(transform_stamped)

        except Exception as e:
            self.pwarn(f"Wheel OFFSET anchor TF error: {e}")

    # ------------------------------------------------------------------------
    # End-Effector MOCAP Tracking in Simulation
    # ------------------------------------------------------------------------
    def update_eef_mocap_tf(self):
        """
        If simulation_mode==True, link 'leg3gripper2_straight' -> 'mocap3gripper2_straight'
        with an optional offset.
        """
        try:
            result_quat = qt.one
            offset_pos = np.array(self.eef_offset_translation, dtype=float)
            new_tf = np2tf(offset_pos, result_quat)

            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = self.eef_urdf_frame
            transform_stamped.child_frame_id = self.eef_mocap_frame
            transform_stamped.transform = new_tf

            self.tf_broadcaster.sendTransform(transform_stamped)

        except Exception as e:
            self.pwarn(f"EEF TF error: {e}")


def main(args=None):
    myMain(MocapToArmAndWheelTF, args=args)


if __name__ == "__main__":
    main()
