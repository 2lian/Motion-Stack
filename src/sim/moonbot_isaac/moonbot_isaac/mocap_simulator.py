import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R


class MocapSimulator(Node):
    def __init__(self):
        """
        Publishes a noisy transform between two frames.
        Used to simulate a motion capture system by adding noise to a ground truth transform.
        """
        super().__init__("mocap_simulator")

        # Parameters
        self.declare_parameter("ground_truth_frame", "base_link")
        self.declare_parameter("mocap_frame", "mocap_frame")
        self.declare_parameter("offset_translation", [0.0, 0.0, 0.035])
        self.declare_parameter("offset_rotation_rvec", [0.0, 0.0, 0.0])
        self.declare_parameter("noise_std_translation", 0.001)  # meters std
        self.declare_parameter("noise_std_rotation", 0.01)  # rad std
        self.declare_parameter("publish_frequency", 30.0)  # Hz

        # Get parameters
        self.gt_frame = self.get_parameter("ground_truth_frame").value
        self.mocap_frame = self.get_parameter("mocap_frame").value
        self.offset_trans = np.array(self.get_parameter("offset_translation").value)
        self.offset_rot = R.from_rotvec(
            self.get_parameter("offset_rotation_rvec").value
        )
        self.noise_std_trans = self.get_parameter("noise_std_translation").value
        self.noise_std_rot = self.get_parameter("noise_std_rotation").value

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create timer for periodic publishing
        period = 1.0 / self.get_parameter("publish_frequency").value
        self.timer = self.create_timer(period, self.publish_pose)

        self.get_logger().info(f"Publishing {self.mocap_frame} relative to {self.gt_frame}")

    def get_noisy_offset(self):
        # Add Gaussian noise to translation
        noisy_trans = self.offset_trans + np.random.normal(0, self.noise_std_trans, 3)

        # Add Gaussian noise to rotation
        noise_rot_vec = np.random.normal(0, self.noise_std_rot, 3)
        noise_rot = R.from_rotvec(noise_rot_vec)
        noisy_rot = noise_rot * self.offset_rot

        return noisy_trans, noisy_rot

    def publish_pose(self):
        # Add noise
        noisy_trans, noisy_rot = self.get_noisy_offset()

        # Create and publish transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.gt_frame
        t.child_frame_id = self.mocap_frame

        t.transform.translation.x = noisy_trans[0]
        t.transform.translation.y = noisy_trans[1]
        t.transform.translation.z = noisy_trans[2]

        quat = noisy_rot.as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MocapSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
