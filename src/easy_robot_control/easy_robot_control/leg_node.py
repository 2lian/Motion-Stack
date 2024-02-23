import time
import traceback

import numpy as np
from numpy.linalg import qr
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from custom_messages.srv import Vect3, ReturnVect3


def error_catcher(func):
    # This is a wrapper to catch and display exceptions
    # Python exceptions don't work because of ros2's multithreading
    # This func cannot be imported for some reasons
    # No need to use it on the __init__ because this part is not threaded
    def wrap(*args, **kwargs):
        try:
            out = func(*args, **kwargs)
        except Exception as exception:
            if exception is KeyboardInterrupt:
                raise KeyboardInterrupt
            else:
                traceback_logger_node = Node('error_node')
                traceback_logger_node.get_logger().error(traceback.format_exc())
                raise exception
        return out

    return wrap


class LegNode(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__(f'ik_node')

        self.declare_parameter('leg_number', 0)
        self.leg_num = self.get_parameter(
            'leg_number').get_parameter_value().integer_value

        self.declare_parameter('std_movement_time', 3.0)
        self.movement_time = self.get_parameter(
            'std_movement_time').get_parameter_value().double_value

        self.declare_parameter('movement_update_rate', 30.0)
        self.movement_update_rate = self.get_parameter(
            'movement_update_rate').get_parameter_value().double_value

        self.necessary_client = self.create_client(
            Empty, f'ik_{self.leg_num}_alive')
        while not self.necessary_client.wait_for_service(timeout_sec=2):
            self.get_logger().warning(
                f'''Waiting for rviz interface, check that the [ik_{self.leg_num}_alive] service is running''')

        self.get_logger().warning(f'''ik_{self.leg_num} connected :)''')

        self.last_target = np.zeros(3, dtype=float)
        self.current_tip = np.zeros(3, dtype=float)

        self.last_target_expired_timer = self.create_timer(
            1, self.overwrite_target)
        self.last_target_expired_timer.cancel()

        # V Callback Groups V
        #   \  /   #
        #    \/    #
        movement_cbk_group = MutuallyExclusiveCallbackGroup()
        #    /\    #
        #   /  \   #
        # ^ Callback Groups ^

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.ik_pub = self.create_publisher(
            Vector3, f'set_ik_target_{self.leg_num}', 10)
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Subscribers V
        #   \  /   #
        #    \/    #
        self.sub_rel_target = self.create_subscription(
            Vector3,
            f'rel_transl_{self.leg_num}',
            self.rel_transl_cbk,
            10,
            callback_group=movement_cbk_group)
        self.sub_rel_target = self.create_subscription(
            Vector3,
            f'rel_hop_{self.leg_num}',
            self.rel_hop_cbk,
            10,
            callback_group=movement_cbk_group)
        self.tip_pos_sub = self.create_subscription(
            Vector3,
            f'tip_pos_{self.leg_num}',
            self.tip_pos_received_cbk,
            10,
            callback_group=movement_cbk_group)
        #    /\    #
        #   /  \   #
        # ^ Subscribers ^

        # V Service server V
        #   \  /   #
        #    \/    #
        self.iAmAlive = self.create_service(
            Empty, f'leg_{self.leg_num}_alive', (lambda req, res: res))
        self.rel_transl_server = self.create_service(
            Vect3,
            f'leg_{self.leg_num}_rel_transl',
            self.rel_transl_srv_cbk,
            callback_group=movement_cbk_group)
        self.rel_hop_server = self.create_service(
            Vect3,
            f'leg_{self.leg_num}_rel_hop',
            self.rel_hop_srv_cbk,
            callback_group=movement_cbk_group)
        self.shift_server = self.create_service(
            Vect3,
            f'leg_{self.leg_num}_shift',
            self.shift_cbk,
            callback_group=movement_cbk_group)
        self.tipos_server = self.create_service(
            ReturnVect3,
            f'leg_{self.leg_num}_tip_pos',
            self.send_most_recent_tip,
        )
        #    /\    #
        #   /  \   #
        # ^ Service sever ^

    @error_catcher
    def send_most_recent_tip(self, request: ReturnVect3.Request, response: ReturnVect3.Response) -> ReturnVect3.Response:
        response.vector.x = self.last_target[0]
        response.vector.y = self.last_target[1]
        response.vector.z = self.last_target[2]
        return response

    @error_catcher
    def rel_transl(self, target: np.ndarray):
        samples = int(self.movement_time * self.movement_update_rate)
        rate = self.create_rate(self.movement_update_rate)
        start = self.last_target.copy()
        for x in np.linspace(0 + 1 / samples, 1, num=samples):
            x = (1 - np.cos(x * np.pi)) / 2
            x = (1 - np.cos(x * np.pi)) / 2
            intermediate_target = target * x + start * (1 - x)

            msg = Vector3()
            msg.x, msg.y, msg.z = tuple(intermediate_target.tolist())
            rate.sleep()
            self.ik_pub.publish(msg)

        self.last_target = target
        return target

    @error_catcher
    def shift(self, shift: np.ndarray):
        self.rel_transl(self.last_target + shift)
        return

    @error_catcher
    def rel_hop(self, target: np.ndarray):
        samples = int(self.movement_time * self.movement_update_rate)
        rate = self.create_rate(self.movement_update_rate)
        start = self.last_target.copy()
        for x in np.linspace(0 + 1 / samples, 1, num=samples):
            # x = (1 - np.cos(x * np.pi)) / 2
            x = (1 - np.cos(x * np.pi)) / 2
            z_hop = (np.sin(x * np.pi)) * 100
            x = (1 - np.cos(x * np.pi)) / 2
            intermediate_target = target * x + start * (1 - x)
            intermediate_target[2] += z_hop

            msg = Vector3()
            msg.x, msg.y, msg.z = tuple(intermediate_target.tolist())
            rate.sleep()
            self.ik_pub.publish(msg)

        self.last_target = target
        return target

    @error_catcher
    def tip_pos_received_cbk(self, msg):
        self.current_tip[0] = msg.x
        self.current_tip[1] = msg.y
        self.current_tip[2] = msg.z

        if np.linalg.norm(self.current_tip - self.last_target) > 50:
            if self.last_target_expired_timer.is_canceled():
                self.last_target_expired_timer.reset()
        else:
            self.last_target_expired_timer.cancel()

    @error_catcher
    def overwrite_target(self):
        self.get_logger().info(f"leg[{self.leg_num}] target overwriten")
        self.last_target = self.current_tip
        self.last_target_expired_timer.cancel()

    @error_catcher
    def rel_transl_cbk(self, msg):
        target = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.rel_transl(target)

    @error_catcher
    def rel_hop_cbk(self, msg):
        target = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.rel_hop(target)

    @error_catcher
    def shift_cbk(self, request, response):
        target = np.array([request.vector.x, request.vector.y,
                          request.vector.z], dtype=float)
        self.shift(target)
        time.sleep(0.1)
        response.success = True
        return response

    @error_catcher
    def rel_transl_srv_cbk(self, request, response):
        target = np.array([request.vector.x, request.vector.y,
                          request.vector.z], dtype=float)
        self.rel_transl(target)
        time.sleep(0.1)
        response.success = True
        return response

    @error_catcher
    def rel_hop_srv_cbk(self, request, response):
        target = np.array([request.vector.x, request.vector.y,
                          request.vector.z], dtype=float)

        self.rel_hop(target)
        time.sleep(0.1)

        response.success = True
        return response


def main(args=None):
    rclpy.init()
    node = LegNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt as e:
        node.get_logger().debug(
            'KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
