import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
import inverse_kinematics as ik


class RVizInterfaceNode(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__(f'ik_node')

        #    node_list = self.get_node_names()
        rviz_is_running = False

        self.necessary_client = self.create_client(Empty, f'rviz_interface_alive')
        while not self.necessary_client.wait_for_service(timeout_sec=2):
            self.get_logger().warning(
                f'''Waiting for rviz interface, check that the [rviz_interface_alive] service is running''')

        self.get_logger().warning(f'''Rviz interface connected :)''')

        self.declare_parameter('leg_number', 0)
        self.leg_num = self.get_parameter('leg_number').get_parameter_value().integer_value

        ############   V Publishers V
        #   \  /   #
        #    \/    #
        self.joint_pub_list = [None] * 3
        for joint in range(3):
            pub = self.create_publisher(Float64, f'set_joint_{self.leg}_{joint}_real', 10)
            self.joint_pub_list[joint] = pub
        #    /\    #
        #   /  \   #
        ############   ^ Publishers ^

        ############   V Subscribers V
        #   \  /   #
        #    \/    #
        self.sub_rel_target = self.create_subscription(Vector3, f'set_ik_target_{self.leg_num}',
                                                       self.set_ik_cbk,
                                                       10
                                                       )
        #    /\    #
        #   /  \   #
        ############   ^ Subscribers ^

    def set_ik_cbk(self, msg):
        target = np.array([msg.x, msg.y, msg.z], dtype=float)
        angles = ik.leg_ik(self.leg_num, target)
        for joint in range(3):
            msg = Float64()
            msg.data = angles[joint]
            self.joint_pub_list[joint].publish(msg)



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
