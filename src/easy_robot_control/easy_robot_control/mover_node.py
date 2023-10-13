import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


class MoverNode(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__(f'mover_node')
        self.number_of_leg = 4

        self.default_target = np.array([
            [300, 0, -150],
            [0, 300, -150],
            [-300, 0, -150],
            [0, -300, -150],
        ], dtype=float)

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

        ############   V Publishers V
        #   \  /   #
        #    \/    #
        self.ik_pub_arr = np.empty(self.number_of_leg, object)
        self.transl_pub_arr = np.empty(self.number_of_leg, object)
        self.hop_pub_arr = np.empty(self.number_of_leg, object)

        for leg in range(self.number_of_leg):
            self.ik_pub_arr[leg] = self.create_publisher(Vector3,
                                                         f'set_ik_target_{leg}',
                                                         10
                                                         )
            self.transl_pub_arr[leg] = self.create_publisher(Vector3,
                                                             f'rel_transl_{leg}',
                                                             10
                                                             )
            self.hop_pub_arr[leg] = self.create_publisher(Vector3,
                                                          f'rel_hop_{leg}',
                                                          10
                                                          )
        #    /\    #
        #   /  \   #
        ############   ^ Publishers ^

        self.startup_timer = self.create_timer(timer_period_sec=0.2,
                                               callback=self.startup_cbk,
                                               callback_group=None,
                                               clock=None)

    def startup_cbk(self):
        self.startup_timer.destroy()
        self.go_to_default_slow()
        time.sleep(2)
        self.gait_loop()
        time.sleep(2)
        self.gait_loop()
        time.sleep(2)
        self.gait_loop()

    def go_to_default_fast(self):
        for leg in range(self.default_target.shape[0]):
            target = self.default_target[leg, :]
            msg = Vector3()
            msg.x, msg.y, msg.z = tuple(target.tolist())
            self.ik_pub_arr[leg].publish(msg)

    def go_to_default_slow(self):
        for leg in range(self.default_target.shape[0]):
            target = self.default_target[leg, :]
            msg = Vector3()
            msg.x, msg.y, msg.z = tuple(target.tolist())
            self.transl_pub_arr[leg].publish(msg)

    def gait_loop(self):
        step_direction = np.array([100, 0, 0], dtype=float)
        step_back_mm = 20

        now_targets = self.default_target.copy()

        for leg in range(now_targets.shape[0]):
            target = now_targets[leg, :] + step_direction

            step_back = normalize(target * np.array([1, 1, 0])) * step_back_mm

            for ground_leg in range(now_targets.shape[0]):
                if ground_leg != leg:
                    target_for_stepback = now_targets[ground_leg, :] + step_back
                    now_targets[ground_leg, :] = target_for_stepback

                    msg = Vector3()
                    msg.x, msg.y, msg.z = tuple(target_for_stepback.tolist())
                    self.transl_pub_arr[ground_leg].publish(msg)

            now_targets[leg, :] = target + step_back
            msg = Vector3()
            msg.x, msg.y, msg.z = tuple(target.tolist())
            self.hop_pub_arr[leg].publish(msg)

            time.sleep(1)
            for ground_leg in range(now_targets.shape[0]):
                target = now_targets[ground_leg, :] - step_direction / 4 - step_back

                now_targets[ground_leg, :] = target

                msg = Vector3()
                msg.x, msg.y, msg.z = tuple(target.tolist())
                self.transl_pub_arr[ground_leg].publish(msg)

            time.sleep(1)


def main(args=None):
    rclpy.init()
    node = MoverNode()
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
