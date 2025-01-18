from typing import Optional

import numpy as np
import rclpy
from easy_robot_control.EliaNode import EliaNode, myMain
from easy_robot_control.utils.math import Quaternion, qt
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class MocapToArmAndWheelTF(EliaNode):
    def __init__(self):
        super().__init__("mocap_to_arm_wheel_tf_node")

        # Create a TransformBroadcaster for publishing both arm and wheel transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a TF2 Buffer and Listener for subscribing to any published mocap frames
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------------------
        # 1) Parameters for Arm
        # ---------------------------
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("arm_mocap_frame", "mocap3gripper1")  # Arm’s mocap frame
        self.declare_parameter("arm_frame", "leg3gripper1")  # Arm’s URDF frame
        self.declare_parameter("arm_offset_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("arm_offset_rotation_euler", [0.0, 0.0, 0.0])

        # ---------------------------
        # 2) Parameters for Wheel
        # ---------------------------
        self.declare_parameter("wheel_mocap_frame", "mocap11_body")
        self.declare_parameter("wheel_frame", "wheel11_body")
        self.declare_parameter("wheel_offset_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("wheel_offset_rotation_euler", [0.0, 0.0, 0.0])

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

        # Convert Euler angles to offset quaternions
        self.arm_offset_quat = self.euler_to_quaternion(*self.arm_offset_rotation_euler)
        self.wheel_offset_quat = self.euler_to_quaternion(
            *self.wheel_offset_rotation_euler
        )

        # Timers for periodically updating transforms
        self.arm_update_timer = self.create_timer(0.1, self.update_arm_tf)
        self.wheel_update_timer = self.create_timer(0.1, self.update_wheel_tf)

        # -------------------------------------
        # Simulation / mock data for both frames
        # -------------------------------------
        if self.simulation_mode:
            # Timers for publishing mock "world -> mocap" transforms
            self.arm_mock_timer = self.create_timer(0.1, self.mock_arm_mocap_tf)
            self.wheel_mock_timer = self.create_timer(0.1, self.mock_wheel_mocap_tf)

            # Subscriptions to control mock transforms (optional)
            self.pose_subscription_arm = self.create_subscription(
                Pose, "/mock_mocap_control_arm", self.pose_callback_arm, 10
            )
            self.pose_subscription_wheel = self.create_subscription(
                Pose, "/mock_mocap_control_wheel", self.pose_callback_wheel, 10
            )

        # Initial mock positions/orientations
        self.mock_arm_position = [0.5, 0.5, 0.05]
        self.mock_arm_orientation = [0.0, 0.0, 0.0]

        self.mock_wheel_position = [0.6, 1.3, 0.3]
        self.mock_wheel_orientation = [0.0, 0.0, 0.0]

    # ----------------------------------------
    # Utility: Convert Euler angles to Quaternion
    # ----------------------------------------
    def euler_to_quaternion(
        self, roll: float, pitch: float, yaw: float
    ) -> Optional[Quaternion]:
        """Convert Euler angles to a quaternion."""
        qx = qt.from_rotation_vector(np.array([roll, 0, 0]))
        qy = qt.from_rotation_vector(np.array([0, pitch, 0]))
        qz = qt.from_rotation_vector(np.array([0, 0, yaw]))
        return qz * qy * qx

    # ----------------------------------------
    # Arm MOCAP Simulation
    # ----------------------------------------
    def mock_arm_mocap_tf(self):
        """Simulate 'world -> mocap3gripper1' (arm) in simulation."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = self.arm_mocap_frame

        transform.transform.translation.x = self.mock_arm_position[0]
        transform.transform.translation.y = self.mock_arm_position[1]
        transform.transform.translation.z = self.mock_arm_position[2]

        sim_quat = self.euler_to_quaternion(*self.mock_arm_orientation)
        transform.transform.rotation.w = sim_quat.w
        transform.transform.rotation.x = sim_quat.x
        transform.transform.rotation.y = sim_quat.y
        transform.transform.rotation.z = sim_quat.z

        self.tf_broadcaster.sendTransform(transform)

    def pose_callback_arm(self, msg: Pose):
        """Update the arm's mock position/orientation from a Pose message."""
        self.mock_arm_position = [msg.position.x, msg.position.y, msg.position.z]
        # Convert quaternion to Euler
        quat = qt.quaternion(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )
        self.mock_arm_orientation = qt.as_euler_angles(quat)

    def update_arm_tf(self):
        """Link 'mocap3gripper1' to 'leg3gripper1' (arm) with the specified offset."""
        try:
            # Lookup transform from 'world' -> 'mocap3gripper1'
            transform = self.tf_buffer.lookup_transform(
                "world", self.arm_mocap_frame, rclpy.time.Time()
            )

            mocap_quat = qt.quaternion(
                transform.transform.rotation.w,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
            )

            # Combine orientation
            result_quat = mocap_quat * self.arm_offset_quat

            # Build the new transform: 'mocap3gripper1' -> 'leg3gripper1'
            new_tf = TransformStamped()
            new_tf.header.stamp = self.get_clock().now().to_msg()
            new_tf.header.frame_id = self.arm_mocap_frame
            new_tf.child_frame_id = self.arm_frame

            # Translation = offset only
            new_tf.transform.translation.x = self.arm_offset_translation[0]
            new_tf.transform.translation.y = self.arm_offset_translation[1]
            new_tf.transform.translation.z = self.arm_offset_translation[2]

            new_tf.transform.rotation.w = result_quat.w
            new_tf.transform.rotation.x = result_quat.x
            new_tf.transform.rotation.y = result_quat.y
            new_tf.transform.rotation.z = result_quat.z

            self.tf_broadcaster.sendTransform(new_tf)

        except Exception as e:
            self.pwarn(f"Arm TF error: {e}")

    # ----------------------------------------
    # Wheel MOCAP Simulation
    # ----------------------------------------
    def mock_wheel_mocap_tf(self):
        """Simulate 'world -> mocap11_body' (wheel) in simulation."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = self.wheel_mocap_frame

        transform.transform.translation.x = self.mock_wheel_position[0]
        transform.transform.translation.y = self.mock_wheel_position[1]
        transform.transform.translation.z = self.mock_wheel_position[2]

        sim_quat = self.euler_to_quaternion(*self.mock_wheel_orientation)
        transform.transform.rotation.w = sim_quat.w
        transform.transform.rotation.x = sim_quat.x
        transform.transform.rotation.y = sim_quat.y
        transform.transform.rotation.z = sim_quat.z

        self.tf_broadcaster.sendTransform(transform)

    def pose_callback_wheel(self, msg: Pose):
        """Update the wheel's mock position/orientation from a Pose message."""
        self.mock_wheel_position = [msg.position.x, msg.position.y, msg.position.z]
        # Convert quaternion to Euler
        quat = qt.quaternion(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )
        self.mock_wheel_orientation = qt.as_euler_angles(quat)

    def update_wheel_tf(self):
        """Link 'mocap11_body' to 'wheel11_body' with the specified offset."""
        try:
            # Lookup transform from 'world' -> 'mocap11_body'
            transform = self.tf_buffer.lookup_transform(
                "world", self.wheel_mocap_frame, rclpy.time.Time()
            )

            mocap_quat = qt.quaternion(
                transform.transform.rotation.w,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
            )

            # Combine orientation
            result_quat = mocap_quat * self.wheel_offset_quat

            # Build the new transform: 'mocap11_body' -> 'wheel11_body'
            new_tf = TransformStamped()
            new_tf.header.stamp = self.get_clock().now().to_msg()
            new_tf.header.frame_id = self.wheel_mocap_frame
            new_tf.child_frame_id = self.wheel_frame

            # Translation = offset only
            new_tf.transform.translation.x = self.wheel_offset_translation[0]
            new_tf.transform.translation.y = self.wheel_offset_translation[1]
            new_tf.transform.translation.z = self.wheel_offset_translation[2]

            new_tf.transform.rotation.w = result_quat.w
            new_tf.transform.rotation.x = result_quat.x
            new_tf.transform.rotation.y = result_quat.y
            new_tf.transform.rotation.z = result_quat.z

            self.tf_broadcaster.sendTransform(new_tf)

        except Exception as e:
            self.pwarn(f"Wheel TF error: {e}")


def main(args=None):
    myMain(MocapToArmAndWheelTF)


if __name__ == "__main__":
    main()
