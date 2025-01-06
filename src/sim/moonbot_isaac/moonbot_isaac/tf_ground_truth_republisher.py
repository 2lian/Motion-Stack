import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class TfGroundTruthRepublisherNode(Node):
    def __init__(self):
        super().__init__('tf_prefix_republisher_node')
        self.tf_gt_subscription = self.create_subscription(
            TFMessage,
            '/tf_gt',
            self.tf_gt_callback,
            10
        )
        self.tf_publisher = self.create_publisher(
            TFMessage,
            '/tf',
            10
        )

    def add_prefix_to_frame_id(self, frame_id: str) -> str:
        # Keep the "world" frame as is so the ground truth TF tree has the same root
        if frame_id == "world":
            return frame_id
        return f"gt__{frame_id}"

    def tf_gt_callback(self, msg: TFMessage):
        modified_transforms = []

        for transform in msg.transforms:
            # Create a copy of the transform and modify the frame names
            modified_transform = TransformStamped()
            modified_transform.header = transform.header
            modified_transform.child_frame_id = self.add_prefix_to_frame_id(transform.child_frame_id)
            modified_transform.transform = transform.transform

            # Prefix the frame_id if necessary (typically frame_id is in the header)
            modified_transform.header.frame_id = self.add_prefix_to_frame_id(transform.header.frame_id)

            modified_transforms.append(modified_transform)

        # Create a new TFMessage with modified transforms and publish it
        modified_tf_message = TFMessage(transforms=modified_transforms)
        self.tf_publisher.publish(modified_tf_message)

def main(args=None):
    rclpy.init(args=args)
    node = TfGroundTruthRepublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()