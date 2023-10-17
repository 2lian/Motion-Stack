import time
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3

from custom_messages.srv import Vect3


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

        self.declare_parameter('std_movement_time', 0)
        self.movement_time = self.get_parameter('std_movement_time').get_parameter_value().double_value

        self.declare_parameter('movement_update_rate', 0)
        self.movement_update_rate = self.get_parameter('movement_update_rate').get_parameter_value().double_value

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

        ############   V Service client V
        #   \  /   #
        #    \/    #
        self.transl_client_arr = np.empty(4, dtype=object)
        for leg in range(4):
            cli_name = f"leg_{leg}_rel_transl"
            self.transl_client_arr[leg] = self.create_client(Vect3, cli_name)
            while not self.transl_client_arr[leg].wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'service [{cli_name}] not available, waiting ...')

        self.hop_client_arr = np.empty(4, dtype=object)
        for leg in range(4):
            cli_name = f"leg_{leg}_rel_hop"
            self.hop_client_arr[leg] = self.create_client(Vect3, cli_name)
            while not self.hop_client_arr[leg].wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'service [{cli_name}] not available, waiting ...')
        #    /\    #
        #   /  \   #
        ############   ^ Service client ^

        self.startup_timer = self.create_timer(timer_period_sec=0.2,
                                               callback=self.startup_cbk,
                                               callback_group=None,
                                               clock=None)

    def startup_cbk(self):
        self.startup_timer.destroy()
        self.go_to_default_slow()
        time.sleep(1)
        self.gait_loopv2()
        time.sleep(2)
        self.gait_loopv2()
        time.sleep(2)
        self.gait_loopv2()
        time.sleep(2)

    def np2vect3(self, np3dvect):
        req = Vect3.Request()
        req.vector.x, req.vector.y, req.vector.z = tuple(np3dvect.tolist())
        return req

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
        plot_for_stability = False
        counter = 0
        step_direction = np.array([100, 0, 0], dtype=float)
        step_back_mm = 40

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

            if plot_for_stability:
                plt.plot(np.delete(now_targets, leg, axis=0)[:, 0],
                         np.delete(now_targets, leg, axis=0)[:, 1])
                plt.scatter(0, 0, c="red")
                plt.grid()
                plt.savefig(f"{counter}.png")
                plt.clf()
                counter += 1

            now_targets[leg, :] = target + step_back
            msg = Vector3()
            msg.x, msg.y, msg.z = tuple(target.tolist())
            self.hop_pub_arr[leg].publish(msg)

            time.sleep(self.movement_time)
            for ground_leg in range(now_targets.shape[0]):
                target = now_targets[ground_leg, :] - step_direction / 4 - step_back

                now_targets[ground_leg, :] = target

                msg = Vector3()
                msg.x, msg.y, msg.z = tuple(target.tolist())
                self.transl_pub_arr[ground_leg].publish(msg)

            if plot_for_stability:
                plt.plot(np.delete(now_targets, [], axis=0)[:, 0],
                         np.delete(now_targets, [], axis=0)[:, 1])
                plt.scatter(0, 0, c="red")
                plt.grid()
                plt.savefig(f"{counter}.png")
                plt.clf()
                counter += 1
            time.sleep(self.movement_time)

    def gait_loopv2(self):
        plot_for_stability = False
        counter = 0
        step_direction = np.array([100, 0, 0], dtype=float)
        step_back_mm = 40

        now_targets = self.default_target.copy()

        for leg in range(now_targets.shape[0]):
            target = now_targets[leg, :] + step_direction
            step_back = normalize(target * np.array([1, 1, 0])) * step_back_mm

            future_arr = []

            for ground_leg in range(now_targets.shape[0]):
                if ground_leg != leg:
                    target_for_stepback = now_targets[ground_leg, :] + step_back
                    now_targets[ground_leg, :] = target_for_stepback

                    fut = self.transl_client_arr[ground_leg].call_async(self.np2vect3(target_for_stepback))
                    future_arr.append(fut)

            if plot_for_stability:
                targets_to_plot = np.empty((4, 3), dtype=float)
                targets_to_plot[:-1, :] = np.delete(now_targets, leg, axis=0)
                targets_to_plot[-1, :] = np.delete(now_targets, leg, axis=0)[0, :]
                plt.plot(targets_to_plot[:, 0],
                         targets_to_plot[:, 1])
                plt.scatter(0, 0, c="red")
                plt.grid()
                plt.savefig(f"{counter}.png")
                plt.clf()
                counter += 1

            now_targets[leg, :] = target
            fut = self.transl_client_arr[leg].call_async(self.np2vect3(target))
            future_arr.append(fut)

            wait_rate = self.create_rate(20)  # wait for response
            while not all([f.done for f in future_arr]):
                wait_rate.sleep()

            future_arr = []

            for ground_leg in range(now_targets.shape[0]):
                target = now_targets[ground_leg, :] - step_direction / 4 - step_back

                now_targets[ground_leg, :] = target

                fut = self.transl_client_arr[ground_leg].call_async(self.np2vect3(target))
                future_arr.append(fut)

            if plot_for_stability:
                targets_to_plot = np.empty((5, 3), dtype=float)
                targets_to_plot[:-1, :] = now_targets
                targets_to_plot[-1, :] = now_targets[0, :]
                plt.plot(targets_to_plot[:, 0],
                         targets_to_plot[:, 1])
                plt.scatter(0, 0, c="red")
                plt.grid()
                plt.savefig(f"{counter}.png")
                plt.clf()
                counter += 1

            while not all([f.done for f in future_arr]):
                wait_rate.sleep()  # wait for response


def main(args=None):
    rclpy.init()
    node = MoverNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt as e:
        node.get_logger().debug('KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
