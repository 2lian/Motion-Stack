import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3


class LegNode(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__(f'ik_node')

        self.declare_parameter('leg_number', 0)
        self.leg_num = self.get_parameter('leg_number').get_parameter_value().integer_value

        self.necessary_client = self.create_client(Empty, f'ik_{self.leg_num}_alive')
        while not self.necessary_client.wait_for_service(timeout_sec=2):
            self.get_logger().warning(
                f'''Waiting for rviz interface, check that the [ik_{self.leg_num}_alive] service is running''')

        self.get_logger().warning(f'''ik_{self.leg_num} connected :)''')

        self.last_target = np.zeros(3, dtype=float)
        self.freq = 30  # Hz
        self.mov_time = 1  # s

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
        self.sub_rel_target = self.create_subscription(Vector3, f'rel_hop_{self.leg_num}',
                                                       self.rel_hop_cbk,
                                                       10
                                                       )
        #    /\    #
        #   /  \   #
        ############   ^ Subscribers ^

        ############   V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(Empty, f'leg_{self.leg_num}_alive', lambda: None)
        #    /\    #
        #   /  \   #
        ############   ^ Service ^

    def rel_transl_cbk(self, msg):
        samples = self.mov_time * self.freq

        target = np.array([msg.x, msg.y, msg.z], dtype=float)
        for x in np.linspace(0, 1, num=samples):
            x = (1-np.cos(x*np.pi))/2
            intermediate_target = target*x + self.last_target*(1-x)

            msg = Vector3()
            msg.x, msg.y, msg.z = tuple(intermediate_target.tolist())
            self.ik_pub.publish(msg)
            time.sleep(1/self.freq)

        self.last_target = target

    def rel_hop_cbk(self, msg):
        samples = self.mov_time * self.freq

        target = np.array([msg.x, msg.y, msg.z], dtype=float)
        for x in np.linspace(0, 1, num=samples):
            z_hop = (np.sin(x*np.pi)) * 50
            x = (1-np.cos(x*np.pi))/2
            intermediate_target = target*x + self.last_target*(1-x)
            intermediate_target[2] += z_hop

            msg = Vector3()
            msg.x, msg.y, msg.z = tuple(intermediate_target.tolist())
            self.ik_pub.publish(msg)
            time.sleep(1/self.freq)

        self.last_target = target



def main(args=None):
    rclpy.init()
    node = LegNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt as e:
        node.get_logger().debug('KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
