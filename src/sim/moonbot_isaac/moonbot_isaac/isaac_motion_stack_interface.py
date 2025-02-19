import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from bisect import bisect_left

class IsaacMotionStackInterface(Node):
    """
    Bridge between Isaac Sim and Motion Stack
     - Relays joint state messages to Motion Stack
     - Merges and relays joint command messages to Isaac
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

        # The latest values for all the joints accumulated from the command messages
        self.all_joint_state = JointState()

    def joint_state_callback(self, msg):
        """
        Receive the joint states from Isaac Sim and republish to ROS
        """
        self.joint_state_pub.publish(msg)

    def joint_command_callback(self, msg):
        """
        Receive the joint commands from ROS and republish to Isaac Sim
        """
        # Add the new joint commands to the accumulated joint states
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in self.all_joint_state.name:
                idx = self.all_joint_state.name.index(name)
                self.all_joint_state.position[idx] = pos
                self.all_joint_state.velocity[idx] = vel
            else:
                # find the index for the new joint based on alphabetical order and insert the values
                idx = bisect_left(self.all_joint_state.name, name)
                self.all_joint_state.name.insert(idx, name)
                self.all_joint_state.position.insert(idx, pos)
                self.all_joint_state.velocity.insert(idx, vel)

        # Publish all the joints in each message
        msg.name = self.all_joint_state.name
        msg.position = self.all_joint_state.position
        msg.velocity = self.all_joint_state.velocity

        self.joint_command_pub.publish(msg)


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
