import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3


class MoverNode(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__(f'mover_node')

        alive_client_list = [f"leg_{leg}_alive" for leg in range(4)]
        while alive_client_list:
            for client_name in alive_client_list:
                self.necessary_client = self.create_client(Empty, client_name)
                if not self.necessary_client.wait_for_service(timeout_sec=2):
                    self.get_logger().warning(
                        f'''Waiting for necessary node, check that the [{client_name}] service is running''')
                else:
                    alive_client_list.remove(client_name)
                    self.get_logger().warning(f'''{client_name[:-6]} connected :)''')



def main(args=None):
    rclpy.init()
    joint_state_publisher = MoverNode()
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
