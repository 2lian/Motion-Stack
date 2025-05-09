from queue import Queue
from typing import Optional

import rclpy
import yaml
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TfTransformMonitor(Node):
    """
    Track the relative transform from fixed_frame for all connected frames
    """

    def __init__(
        self,
        fixed_frame: str = "world",
        check_period_sec=0.04,
        queue: Optional[Queue] = None,
    ):
        super().__init__(f"tf_transform_monitor_from_{fixed_frame}")

        self.fixed_frame = fixed_frame
        self.queue = queue
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(check_period_sec, self.on_timer)

    def on_timer(self):
        # Get all frames
        frames = self.tf_buffer.all_frames_as_yaml()
        # Bail if no frames
        if frames == "[]":
            return

        frames_dict = yaml.safe_load(frames)

        try:
            parent_frames = set([frame["parent"] for frame in frames_dict.values()])
            frames = set(frames_dict.keys()).union(parent_frames)
        except Exception as e:
            self.get_logger().error(f"Error parsing frames '{frames}'\n{e}")
            return

        # Log error if fixed_frame is not in frames
        if self.fixed_frame not in frames:
            self.get_logger().error(
                f"Selected fixed frame '{self.fixed_frame}' is not an available TF"
            )
            return

        for frame in frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    source_frame=frame,
                    target_frame=self.fixed_frame,
                    time=rclpy.time.Time(seconds=0),  # Use latest available transform
                )
                self.latest_transform = transform
                if self.queue is not None:
                    self.queue.put(transform)
            except TransformException as _:
                pass


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
