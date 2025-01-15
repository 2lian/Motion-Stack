import numpy as np
import quaternion as qt  # Import the quaternion library
import rclpy
from easy_robot_control.EliaNode import EliaNode, myMain
from geometry_msgs.msg import TransformStamped
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
        self.declare_parameter("mocap_frame", "mocap3gripper1")  # Mocap frame
        self.declare_parameter("robot_frame", "leg3gripper1")  # Robot frame
        self.declare_parameter(
            "offset_translation", [0.0, 0.0, 0.0]
        )  # Translation offset [x, y, z]
        self.declare_parameter(
            "offset_rotation_euler", [0.0, 0.0, 0.0]
        )  # Rotation offset (roll, pitch, yaw)

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

    def euler_to_quaternion(
        self, roll: float, pitch: float, yaw: float
    ) -> qt.quaternion:
        """
        Convert Euler angles to a quaternion.

        Args:
            roll (float): Rotation around the X-axis in radians.
            pitch (float): Rotation around the Y-axis in radians.
            yaw (float): Rotation around the Z-axis in radians.

        Returns:
            qt.quaternion: The resulting quaternion.
        """
        # Create quaternions for each rotation
        qx = qt.from_rotation_vector(np.array([roll, 0, 0]))
        qy = qt.from_rotation_vector(np.array([0, pitch, 0]))
        qz = qt.from_rotation_vector(np.array([0, 0, yaw]))

        # Combine them: Note that quaternion multiplication is not commutative
        q = qz * qy * qx
        return q

    def update_tf(self):
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

            # Apply the rotation offset using quaternion multiplication
            result_quat = mocap_quat * self.offset_rotation_quat

            # Apply the translation offset
            new_transform = TransformStamped()
            new_transform.header.stamp = self.get_clock().now().to_msg()
            new_transform.header.frame_id = self.mocap_frame  # Parent frame
            new_transform.child_frame_id = self.robot_frame  # Child frame

            new_transform.transform.translation.x = (
                transform.transform.translation.x + self.offset_translation[0]
            )
            new_transform.transform.translation.y = (
                transform.transform.translation.y + self.offset_translation[1]
            )
            new_transform.transform.translation.z = (
                transform.transform.translation.z + self.offset_translation[2]
            )

            # Set the resulting quaternion back to the new transform
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
