import numpy as np
import rclpy
from geometry_msgs.msg import Transform, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String

from motion_stack_msgs.srv import TFService


class LegMoveTestNode(Node):
    """
    This node waits for the robot to be available and still, then sends a move request to the leg 
    and checks if the leg moved to the expected position.
    """
    def __init__(self):
        super().__init__("leg_move_test_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_success = self.create_publisher(String, "leg_move_test_success", 10)

        # Parameters
        self.declare_parameter("tf_service_name", "/leg4/shift")
        self.declare_parameter("source_frame", "gt__leg4gripper2")
        self.declare_parameter("translate_x", 0.0)
        self.declare_parameter("translate_y", 0.0)
        self.declare_parameter("translate_z", 0.0)
        self.declare_parameter("rotate_x", 0.0)
        self.declare_parameter("rotate_y", 0.0)
        self.declare_parameter("rotate_z", 0.0)
        self.declare_parameter("rotate_w", 1.0)
        self.declare_parameter("initial_transform_wait_seconds", 300.0)
        self.declare_parameter("settle_wait_seconds", 12.0)
        self.declare_parameter("movement_completion_wait_seconds", 300.0)
        self.declare_parameter("traslation_threshold", 0.01)
        self.declare_parameter("rotation_threshold", 0.08)

        def get_param(name):
            return self.get_parameter(name).get_parameter_value()

        self.tf_service_name = get_param("tf_service_name").string_value
        self.source_frame = get_param("source_frame").string_value
        self.translate_x = get_param("translate_x").double_value
        self.translate_y = get_param("translate_y").double_value
        self.translate_z = get_param("translate_z").double_value
        self.rotate_x = get_param("rotate_x").double_value
        self.rotate_y = get_param("rotate_y").double_value
        self.rotate_z = get_param("rotate_z").double_value
        self.rotate_w = get_param("rotate_w").double_value
        self.initial_transform_wait_seconds = get_param(
            "initial_transform_wait_seconds"
        ).double_value
        self.settle_wait_seconds = get_param("settle_wait_seconds").double_value
        self.movement_completion_wait_seconds = get_param(
            "movement_completion_wait_seconds"
        ).double_value
        self.traslation_threshold = get_param("traslation_threshold").double_value
        self.rotation_threshold = get_param("rotation_threshold").double_value


        self.timer = self.create_timer(1.0, self.step_test)
        self.target_frame = "gt__base_link"

        self.client = self.create_client(TFService, self.tf_service_name)

        while not self.client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info("Service not available, waiting again...")

        # State
        self.initial_transform = None
        self.settles_at = None
        self.fail_time_limit = self.get_clock().now() + Duration(seconds=self.initial_transform_wait_seconds)

    def step_test(self):
        now = self.get_clock().now()
        current_transform = self.check_transform()

        if self.initial_transform is None:
            if current_transform is not None:
                # wait for the robot to settle
                if self.settles_at is None:
                    self.settles_at = now + Duration(seconds=self.settle_wait_seconds)
                    self.get_logger().info(f"Settling at {self.settles_at}")
                if self.settles_at > now:
                    self.get_logger().info("Waiting for robot to settle")
                    return
                
                self.initial_transform = current_transform
                self.send_move_request()
                self.fail_time_limit = now + Duration(seconds=self.movement_completion_wait_seconds)

            if self.fail_time_limit < now:
                raise RuntimeError("Could not get initial transform")

        elif current_transform is not None:
            # For now, we just check if the leg moved more than 8cm
            diff = np.linalg.norm(
                [
                    current_transform.x - self.initial_transform.x - self.translate_x,
                    current_transform.y - self.initial_transform.y - self.translate_y,
                    current_transform.z - self.initial_transform.z - self.translate_z,
                ]
            )
            self.get_logger().info(f"Diff: {diff}")
            if diff < self.traslation_threshold:
                self.get_logger().info("Test \"leg_move_test\" was successful!")
                self.pub_success.publish(String(data="success"))
                # Stop the node
                self.destroy_node()
                rclpy.shutdown()
            elif self.fail_time_limit < now:
                raise RuntimeError("Could not detect move")

    def send_move_request(self):
        request = TFService.Request()
        request.tf = Transform()
        # Convert meters to millimeters
        request.tf.translation.x = self.translate_x * 1000
        request.tf.translation.y = self.translate_y * 1000
        request.tf.translation.z = self.translate_z * 1000
        request.tf.rotation.x = self.rotate_x
        request.tf.rotation.y = self.rotate_y
        request.tf.rotation.z = self.rotate_z
        request.tf.rotation.w = self.rotate_w

        self.get_logger().info(f"Sending move request: {request}")

        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Response: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def check_transform(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time()
            )
            translation = transform.transform.translation

            self.get_logger().info(
                f"Translation: x={translation.x}, y={translation.y}, z={translation.z}"
            )

            return translation
        except Exception as e:
            self.get_logger().warn(
                f"Could not transform {self.source_frame} to {self.target_frame}: {e}"
            )

        return None


def main(args=None):
    rclpy.init(args=args)
    node = LegMoveTestNode()
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
