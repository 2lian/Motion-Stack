from queue import Queue
from typing import Optional
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TfTransformMonitor(Node):
    """
    Continously querying the transform between two frames (Used in the Isaac Sim environment)
    """

    def __init__(
        self,
        frame_id: str = "world",
        child_frame_id: str = "gt__wheel11_body",
        check_period_sec=0.04,
        queue: Optional[Queue] = None,
    ):
        super().__init__("tf_prefix_republisher_node")

        self.latest_transform = None
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        self.queue = queue
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(check_period_sec, self.on_timer)

    def on_timer(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id, self.child_frame_id, rclpy.time.Time()
            )
            self.latest_transform = transform

            if self.queue is not None:
                self.queue.put(transform)
        except TransformException as e:
            self.get_logger().error(
                f"Could not get transform between {self.frame_id} and {self.child_frame_id}: {e}"
            )

    def get_latest_transform(self) -> Optional[TransformStamped]:
        return self.latest_transform


def main(args=None):
    rclpy.init(args=args)
    node = TfTransformMonitor()
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
