import threading
from typing import List, Optional, cast

import rclpy
from rclpy.node import Node
from rm_ros_interfaces.msg import Jointpos  # Ensure this is the correct import
from sensor_msgs.msg import JointState


class RealManInterface(Node):
    def __init__(self):
        super().__init__("realman_interface")

        # Declare and get parameters
        self.declare_parameter("joint_names", [])
        self.declare_parameter("publish_rate", 30.0)  # Hz
        self.declare_parameter("follow", False)
        self.declare_parameter("expand", 0.0)

        # Retrieve parameters with type casting
        self.joint_names: List[str] = cast(
            List[str],
            list(
                self.get_parameter("joint_names")
                .get_parameter_value()
                .string_array_value
            ),
        )
        self.publish_rate: float = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )
        self.follow: bool = (
            self.get_parameter("follow").get_parameter_value().bool_value
        )
        self.expand: float = (
            self.get_parameter("expand").get_parameter_value().double_value
        )

        # Validate parameters
        if len(self.joint_names) == 0:
            self.get_logger().error(
                'Parameter "joint_names" is empty. Please provide joint names.'
            )
            rclpy.shutdown()

        # Subscriber to joint_commands
        self.subscription = self.create_subscription(
            JointState, "joint_commands", self.joint_commands_callback, 10
        )
        self.subscription  # Prevent unused variable warning

        # Publisher to RealMan driver
        self.publisher = self.create_publisher(
            Jointpos, "/rm_driver/movej_canfd_cmd", 10
        )

        # Timer for publishing at fixed rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_jointpos)

        # Buffer to store the latest joint commands
        self.lock = threading.Lock()
        self.latest_joint_angles = [
            0.0 for _ in range(7)
        ]  # Adjust based on your robot's DOF

        self.get_logger().info("RealMan Interface Node has been started.")

    def joint_commands_callback(self, msg: JointState):
        with self.lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    index = self.joint_names.index(name)
                    if index < len(self.latest_joint_angles):
                        self.latest_joint_angles[index] = msg.position[i]
                    else:
                        self.get_logger().warn(
                            f'Joint index {index} for joint "{name}" out of range for joint_angles array.'
                        )

    def publish_jointpos(self):
        jointpos_msg = Jointpos()
        with self.lock:
            jointpos_msg.joint = self.latest_joint_angles.copy()
        jointpos_msg.follow = self.follow
        jointpos_msg.expand = self.expand
        jointpos_msg.dof = len(jointpos_msg.joint)

        self.publisher.publish(jointpos_msg)
        self.get_logger().debug(f"Published Jointpos message: {jointpos_msg}")


def main(args=None):
    rclpy.init(args=args)
    node = RealManInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("RealMan Interface Node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
