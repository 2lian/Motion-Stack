from typing import Optional

import numpy as np
import rclpy
from easy_robot_control.EliaNode import EliaNode, myMain
from easy_robot_control.utils.math import Quaternion, qt
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class MocapToRobotTF(EliaNode):
    def __init__(self):
        super().__init__("mocap_to_robot_tf_node")

        # Create a TransformBroadcaster for publishing the link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a TF2 Buffer and Listener for subscribing to mocap3gripper1
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameters
        self.declare_parameter("simulation_mode", True)  # Simulation mode flag
        self.declare_parameter("mocap_frame", "mocap3gripper1")  # Mocap frame
        self.declare_parameter("robot_frame", "leg3gripper1")  # Robot frame
        self.declare_parameter(
            "offset_translation", [0.0, 0.0, 0.0]
        )  # Translation offset [x, y, z]
        self.declare_parameter(
            "offset_rotation_euler", [0.0, 0.0, 0.0]
        )  # Rotation offset (roll, pitch, yaw)

        self.simulation_mode = self.get_parameter("simulation_mode").value
        self.mocap_frame = self.get_parameter("mocap_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.offset_translation = self.get_parameter("offset_translation").value
        self.offset_rotation_euler = self.get_parameter("offset_rotation_euler").value

        # Compute the offset quaternion using the method
        self.offset_rotation_quat = self.euler_to_quaternion(
            *self.offset_rotation_euler
        )

        # Timer for periodically updating the transform
        self.timer = self.create_timer(0.1, self.update_tf)  # Update at 10 Hz

        # Additional timer for simulating motion capture in simulation mode
        if self.simulation_mode:
            self.simulation_timer = self.create_timer(0.1, self.mock_mocap_tf)

            # Subscribe to a topic to dynamically control the mock transform
            self.pose_subscription = self.create_subscription(
                Pose, "/mock_mocap_control", self.pose_callback, 10
            )

        # Attributes for storing mock transform
        self.mock_position = [0.5, 0.5, 0.05]
        self.mock_orientation = [0.0, 0.0, 0.0]  # Euler angles

    def euler_to_quaternion(
        self, roll: float, pitch: float, yaw: float
    ) -> Optional[Quaternion]:
        """
        Convert Euler angles to a quaternion.
        """
        # Create quaternions for each rotation
        qx = qt.from_rotation_vector(np.array([roll, 0, 0]))
        qy = qt.from_rotation_vector(np.array([0, pitch, 0]))
        qz = qt.from_rotation_vector(np.array([0, 0, yaw]))

        # Combine them: Note that quaternion multiplication is not commutative
        q = qz * qy * qx
        return q

    def pose_callback(self, msg: Pose):
        """
        Update the mock position and orientation from received Pose messages.
        """
        # Update position
        self.mock_position = [msg.position.x, msg.position.y, msg.position.z]

        # Convert quaternion to Euler angles
        quat = qt.quaternion(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )
        self.mock_orientation = qt.as_euler_angles(quat)

    def mock_mocap_tf(self):
        """
        Mock the motion capture system by publishing simulated world and mocap3gripper1 TFs.
        """
        # Simulate the world to mocap3gripper1 transform
        mocap_transform = TransformStamped()
        mocap_transform.header.stamp = self.get_clock().now().to_msg()
        mocap_transform.header.frame_id = "world"
        mocap_transform.child_frame_id = self.mocap_frame

        # Use updated position and orientation
        mocap_transform.transform.translation.x = self.mock_position[0]
        mocap_transform.transform.translation.y = self.mock_position[1]
        mocap_transform.transform.translation.z = self.mock_position[2]

        # Convert updated orientation to quaternion
        simulated_quat = self.euler_to_quaternion(*self.mock_orientation)
        mocap_transform.transform.rotation.w = simulated_quat.w
        mocap_transform.transform.rotation.x = simulated_quat.x
        mocap_transform.transform.rotation.y = simulated_quat.y
        mocap_transform.transform.rotation.z = simulated_quat.z

        # Broadcast the simulated transform
        self.tf_broadcaster.sendTransform(mocap_transform)

    def update_tf(self):
        """
        Update and link the mocap TF to the robot TF with offsets.
        """
        try:
            # Lookup the transform from 'world' to 'mocap3gripper1'
            transform = self.tf_buffer.lookup_transform(
                "world", self.mocap_frame, rclpy.time.Time()
            )

            # Extract the quaternion from the mocap transform
            mocap_quat = qt.quaternion(
                transform.transform.rotation.w,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
            )

            # ---------------------------------------------
            # Orientation offset: Multiply mocap orientation by the offset quaternion
            # so that if offset = identity, the frames match exactly
            # ---------------------------------------------
            result_quat = mocap_quat * self.offset_rotation_quat

            # ---------------------------------------------
            # Translation offset: Use ONLY the offset values
            # rather than re-adding the mocap's translation.
            # ---------------------------------------------
            new_transform = TransformStamped()
            new_transform.header.stamp = self.get_clock().now().to_msg()
            new_transform.header.frame_id = self.mocap_frame  # Parent frame
            new_transform.child_frame_id = self.robot_frame  # Child frame

            new_transform.transform.translation.x = self.offset_translation[0]
            new_transform.transform.translation.y = self.offset_translation[1]
            new_transform.transform.translation.z = self.offset_translation[2]

            # Set the resulting orientation
            new_transform.transform.rotation.w = result_quat.w
            new_transform.transform.rotation.x = result_quat.x
            new_transform.transform.rotation.y = result_quat.y
            new_transform.transform.rotation.z = result_quat.z

            # Broadcast the transform
            self.tf_broadcaster.sendTransform(new_transform)

        except Exception as e:
            self.pwarn(f"Could not get transform: {e}")


def main(args=None):
    myMain(MocapToRobotTF)


if __name__ == "__main__":
    main()
