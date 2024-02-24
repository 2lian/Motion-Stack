import numpy as np
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import shape
import rclpy
from rclpy.task import Future
from rclpy.node import Node, Union, List
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Transform

from custom_messages.srv import ReturnVect3, Vect3


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def future_list_complete(future_list: List[Future]):
    return np.all([f.done() for f in future_list])


class MoverNode(Node):

    def __init__(self):
        # rclpy.init()
        super().__init__('mover_node')  # type: ignore
        self.number_of_leg = 4
        self.free_leg = np.zeros(self.number_of_leg, dtype=bool)
        self.last_sent_target_set = np.empty(
            (self.number_of_leg, 3), dtype=float)
        self.live_target_set = np.empty(
            (self.number_of_leg, 3), dtype=float)

        self.declare_parameter('std_movement_time', 0.0)
        self.movement_time = self.get_parameter(
            'std_movement_time').get_parameter_value().double_value

        self.declare_parameter('movement_update_rate', 0.0)
        self.movement_update_rate = self.get_parameter(
            'movement_update_rate').get_parameter_value().double_value
        self.default_step_back_ratio = 0.1
        height = 200
        width = 300
        self.default_target = np.array([
            [width, 0, -height],
            [0, width, -height],
            [-width, 0, -height],
            [0, -width, -height],
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
                    self.get_logger().warning(
                        f'''{client_name[:-6]} connected :)''')

        self.cbk_grp1 = MutuallyExclusiveCallbackGroup()

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.ik_pub_arr = np.empty(self.number_of_leg, object)
        self.transl_pub_arr = np.empty(self.number_of_leg, object)
        self.hop_pub_arr = np.empty(self.number_of_leg, object)

        for leg in range(self.number_of_leg):
            self.ik_pub_arr[leg] = self.create_publisher(
                Vector3, f'set_ik_target_{leg}', 10, callback_group=self.cbk_grp1)
            self.transl_pub_arr[leg] = self.create_publisher(
                Vector3, f'rel_transl_{leg}', 10, callback_group=self.cbk_grp1)
            self.hop_pub_arr[leg] = self.create_publisher(
                Vector3, f'rel_hop_{leg}', 10, callback_group=self.cbk_grp1)
        self.rviz_transl_smooth = self.create_publisher(
            Transform, "smooth_body_rviz", 10)
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Service client V
        #   \  /   #
        #    \/    #

        self.transl_client_arr = np.empty(self.number_of_leg, dtype=object)
        for leg in range(self.number_of_leg):
            cli_name = f"leg_{leg}_rel_transl"
            self.transl_client_arr[leg] = self.create_client(
                Vect3, cli_name, callback_group=self.cbk_grp1)
            while not self.transl_client_arr[leg].wait_for_service(
                    timeout_sec=1.0):
                self.get_logger().warn(
                    f'service [{cli_name}] not available, waiting ...')

        self.hop_client_arr = np.empty(self.number_of_leg, dtype=object)
        for leg in range(self.number_of_leg):
            cli_name = f"leg_{leg}_rel_hop"
            self.hop_client_arr[leg] = self.create_client(
                Vect3, cli_name, callback_group=self.cbk_grp1)
            while not self.hop_client_arr[leg].wait_for_service(
                    timeout_sec=1.0):
                self.get_logger().warn(
                    f'service [{cli_name}] not available, waiting ...')

        self.shift_client_arr = np.empty(self.number_of_leg, dtype=object)
        for leg in range(self.number_of_leg):
            cli_name = f"leg_{leg}_shift"
            self.shift_client_arr[leg] = self.create_client(
                Vect3, cli_name, callback_group=self.cbk_grp1)
            while not self.shift_client_arr[leg].wait_for_service(
                    timeout_sec=1.0):
                self.get_logger().warn(
                    f'service [{cli_name}] not available, waiting ...')

        self.tip_pos_client_arr = np.empty(self.number_of_leg, dtype=object)
        for leg in range(self.number_of_leg):
            cli_name = f"leg_{leg}_tip_pos"
            self.tip_pos_client_arr[leg] = self.create_client(
                ReturnVect3, cli_name, callback_group=self.cbk_grp1)
            while not self.tip_pos_client_arr[leg].wait_for_service(
                    timeout_sec=1.0):
                self.get_logger().warn(
                    f'service [{cli_name}] not available, waiting ...')

        #    /\    #
        #   /  \   #
        # ^ Service client ^

        # V Service server V
        #   \  /   #
        #    \/    #

        self.create_service(Vect3, "crawl_step", self.crawl_step_cbk)
        self.create_service(Vect3, "body_shift", self.body_shift_cbk)

        #    /\    #
        #   /  \   #
        # ^ Service server ^

        self.startup_timer = self.create_timer(
            timer_period_sec=0.2,
            callback=self.startup_cbk,
            callback_group=MutuallyExclusiveCallbackGroup(),
            clock=None)  # type: ignore

    def startup_cbk(self) -> None:
        self.startup_timer.destroy()
        self.go_to_default_slow()
        self.update_tip_pos()
        self.last_sent_target_set = self.live_target_set
        while 1:
            self.gait_loopv2()
            # break

    def wait_on_futures(self, future_list: List[Future], wait_Hz: float = 10):
        wait_rate = self.create_rate(wait_Hz)
        while not future_list_complete(future_list):
            wait_rate.sleep()

    def update_tip_pos(self):
        future_arr = []

        for leg in range(self.number_of_leg):
            service = self.tip_pos_client_arr[leg]
            fut = service.call_async(ReturnVect3.Request())
            future_arr.append(fut)
        self.wait_on_futures(future_arr, 50)

        for leg in range(self.number_of_leg):
            response = future_arr[leg].result().vector
            self.live_target_set[leg, :] = (response.x, response.y, response.z)
        return self.live_target_set

    def np2vect3(self, np3dvect: np.ndarray) -> Vect3.Request:
        req = Vect3.Request()
        req.vector.x, req.vector.y, req.vector.z = tuple(np3dvect.tolist())
        return req

    def go_to_default_fast(self) -> None:
        for leg in range(self.default_target.shape[0]):
            target = self.default_target[leg, :]
            msg = Vector3()
            msg.x, msg.y, msg.z = tuple(target.tolist())
            self.ik_pub_arr[leg].publish(msg)

    def go_to_default_slow(self) -> None:
        future_arr = []
        for leg in range(self.default_target.shape[0]):
            target = self.default_target[leg, :]
            fut = self.transl_client_arr[leg].call_async(
                self.np2vect3(target))
            future_arr.append(fut)
        self.wait_on_futures(future_arr, 2)

    def body_shift(self, shift: np.ndarray) -> None:
        future_arr = []
        wait_rate = self.create_rate(3)
        for leg in range(self.default_target.shape[0]):
            fut = self.shift_client_arr[leg].call_async(
                self.np2vect3(-shift))
            future_arr.append(fut)
        self.manual_body_translation_rviz(shift)
        while not np.all([f.done() for f in future_arr]):
            wait_rate.sleep()

    def manual_body_translation_rviz(self, shift: np.ndarray) -> None:
        msg = Transform()
        msg.translation.x = float(shift[0])
        msg.translation.y = float(shift[1])
        msg.translation.z = float(shift[2])
        msg.rotation.x = float(0)
        msg.rotation.y = float(0)
        msg.rotation.z = float(0)
        msg.rotation.w = float(1)
        self.rviz_transl_smooth.publish(msg)

    def body_shift_cbk(self, request, response):
        shift_vect = np.array(
            [request.vector.x, request.vector.y, request.vector.z], dtype=float)
        self.body_shift(shift_vect)
        response.success = True
        return response

    def crawl_step_cbk(self, request, response):
        step_direction = np.array(
            [request.x, request.y, request.z], dtype=float)
        self.gait_loop(
            step_direction, step_back_ratio=self.default_step_back_ratio)

        response.success = True
        return response

    def multi_transl(self, target_set: np.ndarray):
        future_list = []
        for leg in range(target_set.shape[0]):
            target = target_set[leg, :]
            if np.isnan(target[0]):
                continue
            fut = self.transl_client_arr[leg].call_async(
                self.np2vect3(target))
            future_list.append(fut)
        return future_list

    def multi_hop(self, target_set: np.ndarray):
        future_list = []
        for leg in range(target_set.shape[0]):
            target = target_set[leg, :]
            if np.isnan(target[0]):
                continue
            fut = self.hop_client_arr[leg].call_async(
                self.np2vect3(target))
            future_list.append(fut)
        return future_list

    def multi_shift(self, target_set: np.ndarray):
        self.get_logger().warn(f'{target_set}')
        future_list = []
        for leg in range(target_set.shape[0]):
            target = target_set[leg, :]
            if np.isnan(target[0]):
                continue
            fut = self.shift_client_arr[leg].call_async(
                self.np2vect3(target))
            future_list.append(fut)
        return future_list

    def move_body_and_hop(self, body_transl: np.ndarray, target_set: np.ndarray):
        is_move = ~np.isnan(target_set[:, 0])
        is_free = ~is_move & self.free_leg
        is_fix = ~is_move & (~self.free_leg)

        shift_target_set = np.empty_like(target_set)
        shift_target_set[is_fix, :] = -body_transl
        shift_target_set[~is_fix, :] = np.nan
        hop_target_set = target_set

        future_list = self.multi_shift(
            shift_target_set) + self.multi_hop(hop_target_set)
        self.free_leg = is_free
        self.manual_body_translation_rviz(body_transl)

        self.wait_on_futures(future_list)

        return

    def step_for_loop(self):
        step_direction = np.array([70, 70, 0], dtype=float)
        self.gait_loop(
            step_direction, step_back_ratio=self.default_step_back_ratio)
        return

    def gait_loopv2(self, step_direction: np.ndarray = np.array(
            [100, 100, 0], dtype=float), step_back_ratio: Union[float, None] = None):

        total_body_movement = np.zeros_like(step_direction)
        for leg in range(self.number_of_leg):
            body_movement = step_direction/4
            total_body_movement += body_movement
            target_set = np.empty_like(self.last_sent_target_set)
            target_set[:, :] = np.nan
            target_set[leg, :] = self.default_target[leg, :] + \
                step_direction.reshape((-1, 3)) - total_body_movement
            self.move_body_and_hop(body_movement, target_set)

    def gait_loop(self, step_direction: np.ndarray = np.array(
            [120, 120, 0], dtype=float), step_back_ratio: Union[float, None] = None):
        """step of 70 70 0 and step_back_ratio of 0.7 does not fall irl"""
        if step_back_ratio is None:
            step_back_ratio = self.default_step_back_ratio
        plot_for_stability = False
        counter = 0

        step_back_mm = step_back_ratio * np.linalg.norm(step_direction)

        now_targets = self.default_target.copy()
        wait_rate = self.create_rate(20)  # wait for response
        step_back = np.zeros(3)

        for leg in range(now_targets.shape[0]):
            target = now_targets[leg, :] + step_direction
            previous_stepback = step_back
            step_back = normalize(target * np.array([1, 1, 0])) * step_back_mm

            future_arr = []

            for ground_leg in range(now_targets.shape[0]):
                target_for_stepback = now_targets[ground_leg, :] + \
                    step_back - previous_stepback - step_direction / 4
                now_targets[ground_leg, :] = target_for_stepback

                fut = self.transl_client_arr[ground_leg].call_async(
                    self.np2vect3(target_for_stepback))
                future_arr.append(fut)
            self.manual_body_translation_rviz(
                -(step_back - previous_stepback - step_direction / 4))

            if plot_for_stability:
                targets_to_plot = np.empty((4, 3), dtype=float)
                targets_to_plot[:-1, :] = np.delete(now_targets, leg, axis=0)
                targets_to_plot[-1,
                                :] = np.delete(now_targets, leg, axis=0)[0, :]
                plt.plot(targets_to_plot[:, 0],
                         targets_to_plot[:, 1])
                plt.scatter(0, 0, c="red")
                plt.grid()
                plt.savefig(f"{counter}.png")
                plt.clf()
                counter += 1

            while not np.all([f.done() for f in future_arr]):
                wait_rate.sleep()
            future_arr = []
            # now_targets[leg, :] = target
            now_targets[leg, :] = now_targets[leg, :] + step_direction

            fut = self.hop_client_arr[leg].call_async(
                self.np2vect3(now_targets[leg, :]))
            future_arr.append(fut)

            while not np.all([f.done() for f in future_arr]):
                wait_rate.sleep()

        future_arr = []
        for ground_leg in range(now_targets.shape[0]):
            target_for_stepback = now_targets[ground_leg, :] - step_back
            now_targets[ground_leg, :] = target_for_stepback

            fut = self.transl_client_arr[ground_leg].call_async(
                self.np2vect3(target_for_stepback))
            future_arr.append(fut)
        self.manual_body_translation_rviz(step_back)
        while not np.all([f.done() for f in future_arr]):
            wait_rate.sleep()


def main(args=None):
    rclpy.init()
    node = MoverNode()
    executor = rclpy.executors.MultiThreadedExecutor()  # ignore
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
