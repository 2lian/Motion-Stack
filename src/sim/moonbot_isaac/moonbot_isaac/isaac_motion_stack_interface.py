from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class PositionVelocityEffort:
    position: Optional[float] = None
    velocity: Optional[float] = None
    effort: Optional[float] = None


class IsaacMotionStackInterface(Node):
    """
    Bridge between Isaac Sim and Motion Stack
     - Relays joint state messages to Motion Stack
     - Merges and relays joint command messages to Isaac
    """

    def __init__(self):
        super().__init__("isaac_motion_stack_interface")

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
        self.all_joint_state: dict[str, PositionVelocityEffort] = {}

    def joint_state_callback(self, msg):
        """
        Receive the joint states from Isaac Sim and republish to ROS
        """
        self.joint_state_pub.publish(msg)

    def joint_command_callback(self, msg):
        """
        Receive the joint commands from ROS and republish to Isaac Sim
        """
        # Merge the new joint commands with the accumulated joint states
        for i, name in enumerate(msg.name):
            if not msg.position or i >= len(msg.position):
                continue

            if name not in self.all_joint_state:
                self.all_joint_state[name] = PositionVelocityEffort()
            
            self.all_joint_state[name].position = msg.position[i]
            if msg.velocity and i < len(msg.velocity):
                self.all_joint_state[name].velocity = msg.velocity[i]

                # Skip effort if velocity is not present
                if msg.effort and i < len(msg.effort):
                    self.all_joint_state[name].effort = msg.effort[i]

        # Create states list and sort by completeness and alphabetically
        def sort_key(state_tuple):
            name, pos, vel, eff = state_tuple
            # Create a priority based on which values are present
            if all(x is not None for x in (pos, vel, eff)):
                priority = 0  # Complete states
            elif pos is not None and vel is not None:
                priority = 1  # Missing effort only
            else:
                priority = 2  # Only position or incomplete
            return (priority, name)  # Secondary sort by name

        states = [
            (name, pve.position, pve.velocity, pve.effort)
            for name, pve in self.all_joint_state.items()
        ]
        # Sort so states with more fields come first so they can be reshaped as parallel arrays
        states.sort(key=sort_key)

        # Publish all the joints in each message
        msg.name = [state[0] for state in states]
        msg.position = [state[1] for state in states if state[1] is not None]
        msg.velocity = [state[2] for state in states if state[2] is not None]
        msg.effort = [state[3] for state in states if state[3] is not None]

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
