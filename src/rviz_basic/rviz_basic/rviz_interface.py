import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty

class CallbackHolder:
    def __init__(self, leg, joint, parent_node, joint_state):
        self.leg = leg
        self.joint_state = joint_state
        self.joint = joint
        self.parent_node = parent_node
        self.parent_node.create_subscription(
            Float64,
            f'set_joint_{leg}_{joint}_real',
            self.set_joint_cbk,
            10)
        self.pub = self.parent_node.create_publisher(
            Float64,
            f'angle_{self.leg}_{self.joint}',
            10)

    def pub_angle_cbk(self):
        return
    def set_joint_cbk(self):
        return

class RVizInterfaceNode(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__(f'joint_state_rviz')

        #    node_list = self.get_node_names()
        rviz_is_running = False

        if not rviz_is_running:
            node_info = self.get_node_names_and_namespaces()
            for node_name, node_namespace in node_info:
                if node_name == "rviz2":
                    rviz_is_running = True
            self.get_logger().warning(f'''Waiting for rviz, check that the [/rviz] node is running''')
        
        while not rviz_is_running:
            node_info = self.get_node_names_and_namespaces()
            for node_name, node_namespace in node_info:
                if node_name == "rviz2":
                    rviz_is_running = True
            self.get_logger().info(f'''Waiting for rviz, check that the [/rviz] node is running''')
            time.sleep(1)

        self.get_logger().warning(f'''Rviz connected :)''')

        leg_num_remapping = [1, 3, 4, 2]

        self.joint_state = JointState()
        self.joint_state.name = [
            f'Leg{leg_num_remapping[0]}_Joint1', f'Leg{leg_num_remapping[0]}_Joint2', f'Leg{leg_num_remapping[0]}_Joint3',
            f'Leg{leg_num_remapping[1]}_Joint1', f'Leg{leg_num_remapping[1]}_Joint2', f'Leg{leg_num_remapping[1]}_Joint3',
            f'Leg{leg_num_remapping[2]}_Joint1', f'Leg{leg_num_remapping[2]}_Joint2', f'Leg{leg_num_remapping[2]}_Joint3',
            f'Leg{leg_num_remapping[3]}_Joint1', f'Leg{leg_num_remapping[3]}_Joint2', f'Leg{leg_num_remapping[3]}_Joint3',
        ]
        self.joint_state.position = [0.0] * (3 * 4)
        self.set_joint_subs = []
        self.loop_rate = 30  # Hz

        self.set_joint_subs.append(self.create_subscription(
            Float64,
            'set_joint_0_0_real',
            self.set_joint_0_0_callback,
            10)
        )

        self.set_joint_subs.append(self.create_subscription(
            Float64,
            'set_joint_0_1_real',
            self.set_joint_0_1_callback,
            10)
        )

        self.set_joint_subs.append(self.create_subscription(
            Float64,
            'set_joint_0_2_real',
            self.set_joint_0_2_callback,
            10)
        )

        self.set_joint_subs.append(self.create_subscription(
            Float64,
            'set_joint_1_0_real',
            self.set_joint_1_0_callback,
            10)
        )

        self.set_joint_subs.append(self.create_subscription(
            Float64,
            'set_joint_1_1_real',
            self.set_joint_1_1_callback,
            10)
        )

        self.set_joint_subs.append(self.create_subscription(
            Float64,
            'set_joint_1_2_real',
            self.set_joint_1_2_callback,
            10)
        )

        self.set_joint_subs.append(self.create_subscription(
            Float64,
            'set_joint_2_0_real',
            self.set_joint_2_0_callback,
            10)
        )

        self.set_joint_subs.append(self.create_subscription(
            Float64,
            'set_joint_2_1_real',
            self.set_joint_2_1_callback,
            10)
        )

        self.set_joint_subs.append(self.create_subscription(
            Float64,
            'set_joint_2_2_real',
            self.set_joint_2_2_callback,
            10)
        )

        self.set_joint_subs.append(self.create_subscription(
            Float64,
            'set_joint_3_0_real',
            self.set_joint_3_0_callback,
            10)
        )

        self.set_joint_subs.append(self.create_subscription(
            Float64,
            'set_joint_3_1_real',
            self.set_joint_3_1_callback,
            10)
        )

        self.set_joint_subs.append(self.create_subscription(
            Float64,
            'set_joint_3_2_real',
            self.set_joint_3_2_callback,
            10)
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10)

        self.tmr = self.create_timer(
            1 / self.loop_rate,
            self.publish_joint_state)

        ############   V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(Empty, 'rviz_interface_alive', lambda: None)
        #    /\    #
        #   /  \   #
        ############   ^ Service ^

    def set_joint_0_0_callback(self, msg):
        self.joint_state.position[0] = msg.data

    def set_joint_0_1_callback(self, msg):
        self.joint_state.position[1] = msg.data + np.pi/2

    def set_joint_0_2_callback(self, msg):
        self.joint_state.position[2] = msg.data

    def set_joint_1_0_callback(self, msg):
        self.joint_state.position[3] = msg.data

    def set_joint_1_1_callback(self, msg):
        self.joint_state.position[4] = msg.data + np.pi/2

    def set_joint_1_2_callback(self, msg):
        self.joint_state.position[5] = msg.data

    def set_joint_2_0_callback(self, msg):
        self.joint_state.position[6] = msg.data

    def set_joint_2_1_callback(self, msg):
        self.joint_state.position[7] = msg.data + np.pi/2

    def set_joint_2_2_callback(self, msg):
        self.joint_state.position[8] = msg.data

    def set_joint_3_0_callback(self, msg):
        self.joint_state.position[9] = msg.data

    def set_joint_3_1_callback(self, msg):
        self.joint_state.position[10] = msg.data + np.pi/2

    def set_joint_3_2_callback(self, msg):
        self.joint_state.position[11] = msg.data

    def publish_joint_state(self):
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state_pub.publish(self.joint_state)


def main(args=None):
    rclpy.init()
    joint_state_publisher = RVizInterfaceNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(joint_state_publisher)
    try:
        executor.spin()
    except KeyboardInterrupt as e:
        joint_state_publisher.get_logger().debug('KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3')
    joint_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
