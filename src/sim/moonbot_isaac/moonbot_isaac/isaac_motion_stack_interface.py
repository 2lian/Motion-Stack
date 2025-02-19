import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


class IsaacMotionStackInterface(Node):
    """
    Bridge between Isaac Sim and Motion Stack
     - Relays joint state messages to Motion Stack
     - Merges and relays joint command messages to Isaac
     - Publishes the joint state errors
    """

    def __init__(self):
        super().__init__("joint_state_converter")

        self.get_logger().info("Starting joint state converter...")
        self.callback_group = ReentrantCallbackGroup()

        # Convert the joint names from Isaac and republish to ROS
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states_isaac",
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.joint_state_pub = self.create_publisher(
            JointState, "/joint_states", 10, callback_group=self.callback_group
        )

        # Convert the joint names from ROS and republish to Isaac
        self.joint_command_sub = self.create_subscription(
            JointState,
            "/joint_commands",
            self.joint_command_callback,
            10,
            callback_group=self.callback_group,
        )
        self.joint_command_pub = self.create_publisher(
            JointState, "/joint_commands_isaac", 10, callback_group=self.callback_group
        )

        # Publish the joint state errors in a separate topic
        self.joint_state_error_pub = self.create_publisher(
            JointState, "/joint_state_errors", 10, callback_group=self.callback_group
        )
        # save the last joint states and commands for comparision (using the ROS names)
        self.last_joint_states = {}
        self.last_joint_commands = {}
        self.error_pub_timer = self.create_timer(
            0.1, self.publish_joint_state_errors, callback_group=self.callback_group
        )

    def joint_state_callback(self, msg):
        """
        Receive the joint states from Isaac Sim and republish to ROS
        """
        self.joint_state_pub.publish(msg)
        # Save the joint states for comparison
        self.last_joint_states.update(
            {name: pos for name, pos in zip(msg.name, msg.position)}
        )

    def joint_command_callback(self, msg):
        """
        Receive the joint commands from ROS and republish to Isaac Sim
        """
        # Save the joint commands for comparison
        self.last_joint_commands.update(
            {name: pos for name, pos in zip(msg.name, msg.position)}
        )

        # Publish all the joints in each message
        msg.name = list(self.last_joint_commands.keys())
        msg.position = list(self.last_joint_commands.values())

        self.joint_command_pub.publish(msg)

    def publish_joint_state_errors(self):
        """
        Publish the joint state errors between Isaac and ROS
        """
        error_msg = JointState()
        error_msg.name = []
        error_msg.position = []
        for name, pos in self.last_joint_states.items():
            if name not in self.last_joint_commands:
                continue
            if pos != self.last_joint_commands[name]:
                error_msg.name.append(name)
                error_msg.position.append(pos - self.last_joint_commands[name])
        if error_msg.name:
            self.joint_state_error_pub.publish(error_msg)


def main(args=None):
    rclpy.init(args=args)

    joint_state_converter = IsaacMotionStackInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(joint_state_converter)

    try:
        executor.spin()
    except KeyboardInterrupt:
        joint_state_converter.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
