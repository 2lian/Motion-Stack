import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3


class RVizInterfaceNode(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__(f'ik_node')

        #    node_list = self.get_node_names()
        rviz_is_running = False

        self.declare_parameter('leg_number', 0)
        self.leg_num = self.get_parameter('leg_number').get_parameter_value().integer_value

        self.necessary_client = self.create_client(Empty, f'ik_{self.leg_num}_alive')
        while not self.necessary_client.wait_for_service(timeout_sec=2):
            self.get_logger().warning(
                f'''Waiting for rviz interface, check that the [ik_{self.leg_num}_alive] service is running''')

        self.get_logger().warning(f'''ik_{self.leg_num} connected :)''')

        self.last_target = np.zeros(3, dtype=float)

        ############   V Publishers V
        #   \  /   #
        #    \/    #
        self.ik_pub = self.create_publisher(Vector3, f'set_ik_target_{self.leg_num}',
                                                   10
                                                   )
        #    /\    #
        #   /  \   #
        ############   ^ Publishers ^

        ############   V Subscribers V
        #   \  /   #
        #    \/    #
        self.sub_rel_target = self.create_subscription(Vector3, f'rel_transl_{self.leg_num}',
                                                       self.rel_transl_cbk,
                                                       10
                                                       )
        #    /\    #
        #   /  \   #
        ############   ^ Subscribers ^

    def rel_transl_cbk(self, msg):
        self.get_logger().warning(f'''moving''')
        samples = 60
        mov_time = 2
        target = np.array([msg.x, msg.y, msg.z], dtype=float)
        for x in np.linspace(0, 1, num=60):
            self.get_logger().warning(f'''{x}''')
            intermediate_target = target*x + self.last_target*(1-x)
            self.get_logger().warning(f'''{intermediate_target}''')
            msg = Vector3()
            msg.x, msg.y, msg.z = tuple(intermediate_target.tolist())
            self.ik_pub.publish(msg)
            time.sleep(0.02)
        self.last_target = target



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
