import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from realsense2_camera_msgs.msg import Extrinsics
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener


class RealsenseExtrinsicsNode(Node):
    """
    Simulate the Realsense specific depth-to-color-sensor extrinsics messages.
    It computes the extrinsics from the TF tree.
    """

    def __init__(self):
        super().__init__("realsense_extrinsics")

        self.depth_to_color_topic = "/camera/camera/extrinsics/depth_to_color"
        self.color_frame = "leg308_camera2_color_optical_frame"
        self.depth_frame = "leg308_camera2_depth_optical_frame"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS profile that publishes the the last message for each new subscriber
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_depth_to_color = self.create_publisher(
            Extrinsics, self.depth_to_color_topic, qos_profile
        )

        self.color_to_depth_tf = None

        self.timer = self.create_timer(1.0, self.query_transform)

    def query_transform(self):
        from_frame = self.depth_frame
        to_frame = self.color_frame

        try:
            # Get the transformation between the sensors
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                time=rclpy.time.Time(seconds=0),  # Use latest available transform
            )

            # Convert the transformation to realsense extrinsics message
            translation = transform.transform.translation
            translation = np.array([translation.x, translation.y, translation.z])
            rotation = transform.transform.rotation
            rotation_matrix = Rotation.from_quat(
                [rotation.x, rotation.y, rotation.z, rotation.w]
            ).as_matrix()

            depth_to_color = Extrinsics(
                translation=translation, rotation=rotation_matrix.flatten()
            )
            self.pub_depth_to_color.publish(depth_to_color)

            self.get_logger().info(
                f"Realsense depth to color transform found: {depth_to_color}"
            )

            # Stop querying the transform (it is static)
            self.timer.cancel()

        except Exception as e:
            self.get_logger().warning(
                f"Could not find transform from '{from_frame}' to '{to_frame}': {e}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseExtrinsicsNode()
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
