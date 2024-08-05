"""
This node is responsible for synchronising several leg movement in order to move the
cente body and perform steps.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

from os import walk
import numpy as np
from numpy.typing import NDArray
import quaternion as qt
import time
import matplotlib.pyplot as plt
import rclpy
from rclpy.task import Future
from rclpy.node import Node, Union, List
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.publisher import Publisher
from EliaNode import EliaNode

import pkg_resources
from rclpy.time import Duration
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TransformStamped, Transform

from custom_messages.srv import ReturnVect3, Vect3, TFService
import python_package_include.distance_and_reachable_function as reach_pkg
import python_package_include.multi_leg_gradient as multi_pkg
import python_package_include.stability as stab_pkg
import python_package_include.inverse_kinematics as ik_pkg

SUCCESS = 0
MAP_PATH = pkg_resources.resource_filename(__name__, "python_package_include/map.npy")

def error_catcher(func):
    # This is a wrapper to catch and display exceptions
    def wrap(*args, **kwargs):
        try:
            out = func(*args, **kwargs)
        except Exception as exception:
            if exception is KeyboardInterrupt:
                raise KeyboardInterrupt
            else:
                traceback_logger_node = Node("error_node")  # type: ignore
                traceback_logger_node.get_logger().error(traceback.format_exc())
                raise KeyboardInterrupt
        return out

    return wrap

def compute_targetset_pressure_precise(
    last_targetset, body_shift, potential_next, leg_angles, leg_dimemsions
):
    body_shift = body_shift.astype(np.float32)
    legCount = leg_angles.shape[0]
    approximate_next_target = potential_next - body_shift

    dist_after = multi_pkg.multi_leg_vect(last_targetset - 1 * body_shift, leg_angles)

    dist_approx = multi_pkg.multi_leg_vect(approximate_next_target, leg_angles)

    angle_after = np.empty_like(last_targetset)
    angle_appromimate = np.empty_like(last_targetset)

    for leg in range(legCount):
        leg_param = ik_pkg.rotate_legparam_by(
            ik_pkg.moonbot0_leg_default, ik_pkg.PI_OVER_2_Z_QUAT**legCount, False
        )
        angle_after[leg, :] = ik_pkg.simple_leg_ik(
            last_targetset[leg, :] - body_shift,
            leg_param,
        )
        angle_appromimate[leg, :] = ik_pkg.simple_leg_ik(
            approximate_next_target[leg, :],
            leg_param,
        )

    pressure = np.empty((legCount,), np.float32)

    for leg in range(legCount):
        next_pressure = dist_after.copy()
        next_pressure[leg, :] = dist_approx[leg, :]
        next_pressure = np.linalg.norm(next_pressure, axis=1) / (
            np.linalg.norm(body_shift) * legCount
        )
        coxa_press = angle_after[:, 0].copy()
        coxa_press[leg] = angle_appromimate[leg, 0]
        coxa_press = abs(coxa_press) / abs(leg_dimemsions.coxaMax) * 3

        stabbool_before = stab_pkg.is_point_stable(
            np.delete(last_targetset, leg, axis=0), np.zeros_like(body_shift)
        )
        stabbool_after = stab_pkg.is_point_stable(
            np.delete(last_targetset, leg, axis=0), body_shift
        )
        if not (stabbool_before and stabbool_after):
            stab_before = stab_pkg.stability_vector(
                np.delete(last_targetset, leg, axis=0),
                np.zeros_like(body_shift),
                stabbool_before,
            )
            stab_after = stab_pkg.stability_vector(
                np.delete(last_targetset, leg, axis=0), body_shift, stabbool_after
            )
            stab_antipress = (
                (
                    np.linalg.norm(stab_after * ~stabbool_after)
                    - np.linalg.norm(stab_before * ~stabbool_before)
                )
                / np.linalg.norm(body_shift)
                / 3
            )
        else:
            stab_antipress = 0
        # pressure[leg] = sum(coxa_press + next_pressure)
        pressure[leg] = sum(coxa_press + next_pressure + stab_antipress)

    return pressure


def compute_targetset_pressure(last_targetset, body_shift, leg_angles, leg_dimemsions):
    legCount = leg_angles.shape[0]
    approximate_next_target = last_targetset + body_shift * (legCount - 1)

    dist_after = multi_pkg.multi_leg_vect(
        last_targetset - body_shift.astype(np.float32), leg_angles
    )

    dist_approx = multi_pkg.multi_leg_vect(approximate_next_target, leg_angles)

    angle_after = np.empty_like(last_targetset)
    angle_appromimate = np.empty_like(last_targetset)

    for leg in range(legCount):
        this_leg_dim = ik_pkg.rotate_leg_by(leg_dimemsions, ik_pkg.PI_OVER_2_Z_QUAT**leg)
        angle_after[leg, :] = ik_pkg.simple_leg_ik(
            last_targetset[leg, :] - body_shift, this_leg_dim
        )
        angle_appromimate[leg, :] = ik_pkg.simple_leg_ik(
            approximate_next_target[leg, :], this_leg_dim
        )

    pressure = np.empty((legCount,), np.float32)

    for leg in range(legCount):
        next_pressure = dist_after.copy()
        next_pressure[leg, :] = dist_approx[leg, :]
        next_pressure = np.linalg.norm(next_pressure, axis=1) / (
            np.linalg.norm(body_shift) * legCount
        )
        coxa_press = angle_after[:, 0].copy()
        coxa_press[leg] = angle_appromimate[leg, 0]
        coxa_press = abs(coxa_press) / abs(leg_dimemsions.coxaMax) * 1

        # stabbool_before = stab_pkg.is_point_stable(
        #     np.delete(last_targetset, leg, axis=0), np.zeros_like(body_shift)
        # )
        # stabbool_after = stab_pkg.is_point_stable(
        #     np.delete(last_targetset, leg, axis=0), body_shift
        # )
        # if not (stabbool_before and stabbool_after):
        #     stab_before = stab_pkg.stability_vector(
        #         np.delete(last_targetset, leg, axis=0),
        #         np.zeros_like(body_shift),
        #         stabbool_before,
        #     )
        #     stab_after = stab_pkg.stability_vector(
        #         np.delete(last_targetset, leg, axis=0), body_shift, stabbool_after
        #     )
        #     stab_antipress = (
        #         np.linalg.norm(stab_after * ~stabbool_after) - np.linalg.norm(stab_before * ~stabbool_before)
        #     ) / np.linalg.norm(body_shift)
        # else:
        #     stab_antipress = 0
        pressure[leg] = sum(coxa_press + next_pressure)
        # pressure[leg] = sum(coxa_press + next_pressure + stab_antipress)

    return pressure


def normalize(v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def future_list_complete(future_list: List[Future]) -> np.bool_:
    """Returns True is all futures in the input list are done.

    Args:
        future_list: a list of futures

    Returns:
        True if all futures are done
    """
    return np.all([f.done() for f in future_list])


class MoverNode(EliaNode):

    def __init__(self):
        # rclpy.init()
        super().__init__("mover_node")  # type: ignore

        self.declare_parameter("number_of_legs", 4)
        self.NUMBER_OF_LEG = (
            self.get_parameter("number_of_legs").get_parameter_value().integer_value
        )

        self.Alias = "M"
        self.IGNORE_LIMITS = True
        self.GRAV_STABILITY_MARGIN = 50  # mm
        self.FOOTHOLDS = np.load(MAP_PATH)
        self.leg_dimemsions = ik_pkg.moonbot0_leg_default
        self.HIGH_PRECISION_MANOUVERS = False
        self.legs_angle = np.linspace(
            0, 2 * np.pi, self.NUMBER_OF_LEG, endpoint=False, dtype=np.float32
        )
        self.free_leg = np.zeros(self.NUMBER_OF_LEG, dtype=bool)
        self.last_sent_target_set = np.empty((self.NUMBER_OF_LEG, 3), dtype=float)
        self.live_target_set = np.empty((self.NUMBER_OF_LEG, 3), dtype=float)
        self.body_coord = np.zeros((3,), np.float32)
        self.body_coord[2] = 0
        # self.body_coord[2] = 200
        self.body_quat = qt.from_euler_angles(0, 0, 0)

        self.declare_parameter("std_movement_time", 1.0)
        self.MOVEMENT_TIME = (
            self.get_parameter("std_movement_time").get_parameter_value().double_value
        )

        self.declare_parameter("movement_update_rate", 30.0)
        self.MOVEMENT_UPDATE_RATE = (
            self.get_parameter("movement_update_rate").get_parameter_value().double_value
        )
        self.default_step_back_ratio = 0.1
        height = 220
        width = 250
        self.default_target = np.array(
            [
                [width, 0, -height],
                # [width, 0, -height],
                # [width, 0, -height],
                # [width, 0, -height],
                [0, width, -height],
                [-width, 0, -height],
                [0, -width, -height],
            ],
            dtype=float,
        )

        alive_client_list = [f"leg_{leg}_alive" for leg in range(self.NUMBER_OF_LEG)]
        self.setAndBlockForNecessaryClients(alive_client_list)

        self.cbk_grp1 = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            Transform,
            "auto_place",
            self.auto_place_cbk,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        # V Publishers V
        #   \  /   #
        #    \/    #
        self.ik_pub_arr = np.empty(self.NUMBER_OF_LEG, object)
        self.roll_speed_pub: List[Publisher] = []

        for leg in range(self.NUMBER_OF_LEG):
            self.ik_pub_arr[leg] = self.create_publisher(
                Vector3,
                f"set_ik_target_{leg}",
                10,
            )
            self.roll_speed_pub.append(
                self.create_publisher(
                    Float64,
                    f"smart_roll_{leg}",
                    10,
                )
            )
        self.rviz_transl_smooth = self.create_publisher(Transform, "smooth_body_rviz", 10)
        self.rviz_teleport = self.create_publisher(Transform, "robot_body", 10)
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Service client V
        #   \  /   #
        #    \/    #

        LEG_MOVEMENT_MSG_TYPE = TFService

        self.transl_client_arr = np.empty(self.NUMBER_OF_LEG, dtype=object)
        for leg in range(self.NUMBER_OF_LEG):
            cli_name = f"leg_{leg}_rel_transl"
            self.transl_client_arr[leg] = self.get_and_wait_Client(
                cli_name, LEG_MOVEMENT_MSG_TYPE
            )

        self.hop_client_arr = np.empty(self.NUMBER_OF_LEG, dtype=object)
        for leg in range(self.NUMBER_OF_LEG):
            cli_name = f"leg_{leg}_rel_hop"
            self.hop_client_arr[leg] = self.get_and_wait_Client(
                cli_name, LEG_MOVEMENT_MSG_TYPE
            )

        self.shift_client_arr = np.empty(self.NUMBER_OF_LEG, dtype=object)
        for leg in range(self.NUMBER_OF_LEG):
            cli_name = f"leg_{leg}_shift"
            self.shift_client_arr[leg] = self.get_and_wait_Client(
                cli_name, LEG_MOVEMENT_MSG_TYPE
            )

        self.rot_client_arr = np.empty(self.NUMBER_OF_LEG, dtype=object)
        for leg in range(self.NUMBER_OF_LEG):
            cli_name = f"leg_{leg}_rot"
            self.rot_client_arr[leg] = self.get_and_wait_Client(
                cli_name, LEG_MOVEMENT_MSG_TYPE
            )

        self.tip_pos_client_arr = np.empty(self.NUMBER_OF_LEG, dtype=object)
        for leg in range(self.NUMBER_OF_LEG):
            cli_name = f"leg_{leg}_tip_pos"
            self.tip_pos_client_arr[leg] = self.get_and_wait_Client(cli_name, ReturnVect3)

        self.point_cli_arr = np.empty(self.NUMBER_OF_LEG, dtype=object)
        for leg in range(self.NUMBER_OF_LEG):
            cli_name = f"leg_{leg}_roll"
            self.point_cli_arr[leg] = self.get_and_wait_Client(cli_name, TFService)

        #    /\    #
        #   /  \   #
        # ^ Service client ^

        # V Service server V
        #   \  /   #
        #    \/    #

        self.create_service(Vect3, "crawl_step", self.crawl_step_cbk)
        self.create_service(Vect3, "body_shift", self.body_shift_cbk)
        self.create_service(TFService, "body_tfshift", self.body_tfshift_cbk)

        #    /\    #
        #   /  \   #
        # ^ Service server ^

        self.startup_timer = self.create_timer(
            timer_period_sec=1,
            callback=self.startup_cbk,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def startup_cbk(self) -> None:
        self.startup_timer.cancel()
        self.sleep(seconds=2)
        self.stand()
        # self.go_to_default_slow()
        # self.body_tfshift(np.array([0, 0, -50], dtype=float), qt.one)
        self.sleep(seconds=0.1)
        self.update_tip_pos()
        self.last_sent_target_set = self.live_target_set
        # r = False
        r = True
        while r:
            # # quat = qt.from_rotation_vector([0.3, 0, 0])
            # # self.body_tfshift(np.array([0, 25, -25], dtype=float), quat)
            # # self.body_tfshift(-np.array([0, 25, -25], dtype=float), 1/quat)
            # z_shift = 100 * 0.0
            # quat = qt.from_rotation_vector([0, -0.1, 0.1]) ** 2
            #
            # # [pub.publish(Float64(data=float(0))) for pub in self.roll_speed_pub]
            # fl: List[Future] = []
            # for leg in range(self.NUMBER_OF_LEG - 0):
            #     shift_msg = self.np2tfReq(np.array([1, 0, 0]), qt.one)
            #     # f: Future = self.point_cli_arr[leg].call_async(shift_msg)
            #     # fl.append(f)
            # self.sleep(0.01)
            #
            # self.body_tfshift(np.array([0, 0, -z_shift], dtype=float), quat)
            # self.wait_on_futures(fl)
            #
            # # [pub.publish(Float64(data=float(1000))) for pub in self.roll_speed_pub]
            # self.body_tfshift(-np.array([0, 0, -z_shift], dtype=float), 1 / quat)
            #
            # # [pub.publish(Float64(data=float(0))) for pub in self.roll_speed_pub]
            # fl = []
            # for leg in range(self.NUMBER_OF_LEG - 0):
            #     shift_msg = self.np2tfReq(np.array([0, 1, 0]), qt.one)
            #     # f = self.point_cli_arr[leg].call_async(shift_msg)
            #     # fl.append(f)
            # self.sleep(0.01)
            #
            # self.body_tfshift(np.array([0, 0, z_shift], dtype=float), 1 / quat)
            # self.wait_on_futures(fl)
            #
            # # [pub.publish(Float64(data=float(-1000))) for pub in self.roll_speed_pub]
            # self.body_tfshift(-np.array([0, 0, z_shift], dtype=float), quat)
            # # self.startup_timer.reset()
            # self.gait_loopv2()
            self.gait_loopv3()
            # self.fence_stepover()
            # break
            # r = self.dumb_auto_walk(np.array([40, 0, 0], dtype=float)) is SUCCESS
            # break
            continue
            r = self.dumb_auto_walk(np.array([40, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([40, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([40, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([40, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([40, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([40, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([40, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([40, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([40, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([40, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([40, 0, 10], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([40, 0, 10], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([40, 0, 10], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([20, 0, 10], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([20, 0, 10], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([0, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([0, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([0, 10, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([0, 10, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([0, 10, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([0, 10, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([10, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([10, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([10, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([10, 0, 0], dtype=float)) is SUCCESS
            r = self.dumb_auto_walk(np.array([10, 0, 0], dtype=float)) is SUCCESS
            # self.fence_stepover()
            break

    def update_tip_pos(self):
        future_arr = []

        for leg in range(self.NUMBER_OF_LEG):
            service = self.tip_pos_client_arr[leg]
            fut = service.call_async(ReturnVect3.Request())
            future_arr.append(fut)
        self.wait_on_futures(future_arr, 50)

        for leg in range(self.NUMBER_OF_LEG):
            response = future_arr[leg].result().vector
            self.live_target_set[leg, :] = (response.x, response.y, response.z)
        return self.live_target_set

    def np2vect3(self, np3dvect: np.ndarray) -> Vect3.Request:
        req = Vect3.Request()
        req.vector.x, req.vector.y, req.vector.z = tuple(np3dvect.tolist())
        return req

    def np2tfReq(
        self, coord: np.ndarray, quat: qt.quaternion = qt.one
    ) -> TFService.Request:
        request = TFService.Request()
        request.tf = self.np2tf(coord, quat)
        return request

    def go_to_default_fast(self) -> None:
        for leg in range(self.default_target.shape[0]):
            target = self.default_target[leg, :]
            msg = Vector3()
            msg.x, msg.y, msg.z = tuple(target.tolist())
            self.ik_pub_arr[leg].publish(msg)

    @error_catcher
    def stand(self) -> None:
        future_arr = []
        height = 0
        width = 350
        flatTarget = np.array(
            [
                [width, 0, -height],
                # [width, 0, -height],
                # [width, 0, -height],
                # [width, 0, -height],
                [0, width, -height],
                [-width, 0, -height],
                [0, -width, -height],
            ],
            dtype=float,
        )
        for leg in range(flatTarget.shape[0]):
            target = flatTarget[leg, :]
            fut = self.hop_client_arr[leg].call_async(self.np2tfReq(target))
            future_arr.append(fut)
        self.wait_on_futures(future_arr)

        future_arr = []
        height = -self.default_target[0, 2]
        flatTarget = np.array(
            [
                [width, 0, -height],
                # [width, 0, -height],
                # [width, 0, -height],
                # [width, 0, -height],
                [0, width, -height],
                [-width, 0, -height],
                [0, -width, -height],
            ],
            dtype=float,
        )
        for leg in range(flatTarget.shape[0]):
            target = flatTarget[leg, :]
            fut = self.transl_client_arr[leg].call_async(self.np2tfReq(target))
            future_arr.append(fut)
        self.wait_on_futures(future_arr)
        self.update_tip_pos()
        self.last_sent_target_set = self.live_target_set
        self.gait_loopv3(np.array([0,0,0], dtype = np.float32))

    def go_to_default_slow(self) -> None:
        future_arr = []
        for leg in range(self.default_target.shape[0]):
            target = self.default_target[leg, :]
            fut = self.transl_client_arr[leg].call_async(self.np2tfReq(target))
            future_arr.append(fut)
        self.wait_on_futures(future_arr, 2)

    def body_tfshift(self, shift: np.ndarray, rot: qt.quaternion = qt.one) -> None:
        future_list = []
        for leg in range(self.NUMBER_OF_LEG):
            shift_msg = self.np2tfReq(-shift, qt.one)
            shift_future = self.shift_client_arr[leg].call_async(shift_msg)
            future_list.append(shift_future)

            rot_msg = self.np2tfReq(shift * 0, 1 / rot)
            rot_future = self.rot_client_arr[leg].call_async(rot_msg)
            future_list.append(rot_future)
        self.manual_body_translation_rviz(shift, rot)

        self.wait_on_futures(future_list)

    def body_shift(self, shift: np.ndarray) -> None:
        return self.body_tfshift(shift)

    def manual_body_translation_rviz(
        self, coord: np.ndarray, quat: qt.quaternion = qt.one
    ) -> None:
        self.body_coord += coord
        msg = Transform()
        msg.translation.x = float(coord[0])
        msg.translation.y = float(coord[1])
        msg.translation.z = float(coord[2])
        msg.rotation.x = float(quat.x)
        msg.rotation.y = float(quat.y)
        msg.rotation.z = float(quat.z)
        msg.rotation.w = float(quat.w)
        self.rviz_transl_smooth.publish(msg)

    def set_body_transform_rviz(
        self, coord: np.ndarray, quat: qt.quaternion = qt.one
    ) -> None:
        self.body_coord = coord
        self.body_quat = quat
        msg = self.np2tf(coord, quat)
        self.rviz_teleport.publish(msg)

    def body_tfshift_cbk(
        self, request: TFService.Request, response: TFService.Response
    ) -> TFService.Response:
        shift, quat = self.tf2np(request.tf)
        self.body_tfshift(shift, quat)
        response.success_str.data = ""  # TODO
        return response

    def body_shift_cbk(self, request, response):  # old
        shift_vect = np.array(
            [request.vector.x, request.vector.y, request.vector.z], dtype=float
        )
        self.body_shift(shift_vect)
        response.success = True
        return response

    def crawl_step_cbk(self, request, response):
        step_direction = np.array([request.x, request.y, request.z], dtype=float)
        self.gait_loop(step_direction, step_back_ratio=self.default_step_back_ratio)

        response.success = True
        return response

    def multi_transl(self, target_set: np.ndarray):
        future_list = []
        for leg in range(target_set.shape[0]):
            target = target_set[leg, :]
            if np.isnan(target[0]):
                continue
            fut = self.transl_client_arr[leg].call_async(self.np2tfReq(target))
            future_list.append(fut)
        return future_list

    def multi_hop(self, target_set: np.ndarray):
        future_list = []
        for leg in range(target_set.shape[0]):
            target = target_set[leg, :]
            if np.isnan(target[0]):
                continue
            fut = self.hop_client_arr[leg].call_async(self.np2tfReq(target))
            future_list.append(fut)
            self.last_sent_target_set[leg, :] = target
        return future_list

    def multi_shift(self, target_set: np.ndarray):
        future_list = []
        for leg in range(target_set.shape[0]):
            target = target_set[leg, :]
            if np.isnan(target[0]) or sum(abs(target)) < 0.01:
                continue
            fut = self.shift_client_arr[leg].call_async(self.np2tfReq(target))
            future_list.append(fut)
            self.last_sent_target_set[leg, :] += target
        return future_list

    def stability_pressure(self, last_targetset, target_set, body_transl):
        pressure = np.zeros((last_targetset.shape[0],), np.float32)  # + np.inf
        for leg in range(last_targetset.shape[0]):
            # potential_TS = target_set.copy()
            # potential_TS[:leg, :] = np.nan
            # if leg < (last_targetset.shape[0] - 1):
            #     potential_TS[leg + 1 :, :] = np.nan

            # kine_ok, dist = self.can_move_to_stability(last_targetset, target_set)

            for legN in range(last_targetset.shape[0]):
                # if legN == leg:
                # continue  # skip
                potentialN = last_targetset.copy()
                potentialN[leg, :] = target_set[leg, :]
                potentialN[legN, :] = np.nan
                # potentialN-= body_transl
                nothing = np.empty_like(potentialN)
                nothing[:, :] = np.nan

                stab = stab_pkg.is_point_stable(potentialN, np.zeros_like(body_transl))
                if stab:
                    distN = 0
                else:
                    distN = stab_pkg.stability_vector(
                        potentialN, np.zeros_like(body_transl), stab
                    )

                pressure[leg] = max(pressure[leg], np.linalg.norm(distN))
                self.get_logger().warn(f"leg#{leg}: {np.linalg.norm(distN)}")
        self.get_logger().warn(f"stab pressure: {pressure}")
        return pressure

    @error_catcher
    def can_move_to_stability(self, last_targetset: np.ndarray, target_set: np.ndarray):
        is_move = ~np.isnan(target_set[:, 0])
        is_free_before = np.isnan(last_targetset[:, 0])
        is_free_after = ~is_move & is_free_before
        is_fix = ~is_move & (~is_free_before)
        last_targetset = last_targetset.astype(np.float32)
        target_set = target_set.astype(np.float32)

        not_stable_at_all = not stab_pkg.is_point_stable(
            last_targetset[is_fix, :][:, [0, 1]], np.zeros(2, np.float32)
        )
        mvt_to_stability = np.zeros((3,), np.float32)

        if not_stable_at_all:
            xy_vect_to_stability = stab_pkg.stability_vector(
                last_targetset[is_fix, :][:, [0, 1]], np.zeros(2, np.float32), False
            )
            stab_direction = xy_vect_to_stability / np.linalg.norm(xy_vect_to_stability)
            mvt_to_stability[[0, 1]] = (
                xy_vect_to_stability + stab_direction * self.GRAV_STABILITY_MARGIN
            )
        else:
            xy_vect_to_stability = stab_pkg.stability_vector(
                last_targetset[is_fix, :][:, [0, 1]], np.zeros(2, np.float32), True
            )
            dist_to_stability = np.linalg.norm(xy_vect_to_stability)
            almost_stable = dist_to_stability < self.GRAV_STABILITY_MARGIN
            if almost_stable:
                if dist_to_stability < 0.00001:  # dist is zero
                    v = np.sum(last_targetset[is_fix, :][:, [0, 1]], axis=0)
                    stab_direction = v / np.linalg.norm(v)
                else:
                    stab_direction = -xy_vect_to_stability / dist_to_stability
                mvt_to_stability[[0, 1]] = (
                    xy_vect_to_stability + stab_direction * self.GRAV_STABILITY_MARGIN
                )
        points = (last_targetset - mvt_to_stability)[~is_free_after, :].astype(np.float32)
        angles = self.legs_angle[~is_free_after].astype(np.float32)
        iK_feasable = np.all(multi_pkg.multi_leg_reachable(points, angles))

        return iK_feasable, mvt_to_stability

    @error_catcher
    def move_and_hop_stable(self, body_transl: np.ndarray, target_set: np.ndarray):
        is_move = ~np.isnan(target_set[:, 0])
        is_free = self.free_leg
        is_free_after = ~is_move & self.free_leg
        is_fix = ~is_move & (~self.free_leg)

        last_targetset = self.last_sent_target_set.astype(np.float32)
        next_targetset = last_targetset - body_transl
        next_targetset[is_move, :] = target_set[is_move, :]

        # step 1 goes inside leg raised stability zone
        step1_feasable, mvt_to_stability = self.can_move_to_stability(
            last_targetset, target_set
        )

        # step 2 goes to edge of stability close to body target and drops legs
        step2_feasable, mvt_to_final = self.can_move_to_stability(
            next_targetset, target_set
        )
        mvt_to_final *= np.float32(-1)

        # step 3 goes to body target fron inside stability
        points = (next_targetset + mvt_to_final)[~is_free_after, :].astype(np.float32)
        angles = self.legs_angle[~is_free_after].astype(np.float32)
        step3_feasable = np.all(multi_pkg.multi_leg_reachable(points, angles))

        mvt_with_leg_raised = body_transl - (mvt_to_final + mvt_to_stability)

        self.get_logger().info(
            f"step1: {step1_feasable} | step2: {step2_feasable} | step3: {step3_feasable}"
        )

        if (step1_feasable and step2_feasable and step3_feasable) or self.IGNORE_LIMITS:

            # no_leg_is_moving = is_move.sum() == 0
            # if no_leg_is_moving:
            #     self.body_shift(body_transl)
            #     return SUCCESS

            nothing = np.empty_like(last_targetset)
            nothing[:, :] = np.nan
            transl = mvt_to_stability
            self.move_body_and_hop(transl, nothing)

            transl = mvt_with_leg_raised
            hop_target_set = nothing.copy()
            hop_target_set[is_move, :] = next_targetset[is_move, :] + mvt_to_final
            self.move_body_and_hop(transl, hop_target_set)

            if self.HIGH_PRECISION_MANOUVERS:
                transl = mvt_to_final
                self.move_body_and_hop(transl, nothing)

            return SUCCESS
        self.get_logger().warn(
            f"[move_and_hop_stable] failed to generate a valid movement"
        )

        return not SUCCESS

    def move_body_and_hop(self, body_transl: np.ndarray, target_set: np.ndarray):
        is_move = ~np.isnan(target_set[:, 0])
        is_free = ~is_move & self.free_leg
        is_fix = ~is_move & (~self.free_leg)

        # those jump at the same place as before, so they can stay fixed
        # on the ground
        # can_be_fix = np.isclose(target_set, (-body_transl.reshape(1, 3)), atol=0.01)
        can_be_fix = np.isclose(
            (target_set - self.last_sent_target_set), -body_transl, atol=0.1
        )
        can_be_fix = np.all(can_be_fix, axis=1)

        shift_target_set = np.empty_like(target_set)
        shift_target_set[:, :] = np.nan
        shift_target_set[is_fix | can_be_fix, :] = -body_transl

        hop_target_set = target_set.copy()
        hop_target_set[can_be_fix, :] = np.nan

        future_list = []
        future_list = self.multi_shift(shift_target_set) + self.multi_hop(hop_target_set)
        translation_is_not_zero = np.linalg.norm(body_transl) > 0.0001
        if translation_is_not_zero:
            self.manual_body_translation_rviz(body_transl)

        self.wait_on_futures(future_list)
        self.free_leg = is_free

        return

    def compute_targetset_pressure(self, last_targetset, body_shift):
        return compute_targetset_pressure(
            last_targetset, body_shift, self.legs_angle, self.leg_dimemsions
        )

    def compute_targetset_pressure_precise(
        self, last_targetset, body_shift, potential_next
    ):
        return compute_targetset_pressure_precise(
            last_targetset,
            body_shift,
            potential_next,
            self.legs_angle,
            self.leg_dimemsions,
        ) + self.stability_pressure(
            last_targetset, potential_next, body_shift
        ) / np.linalg.norm(
            body_shift
        )

    def gait_loopv3(
        self,
        step_direction: NDArray = np.array([100, 0, 0], dtype=float),
        step_back_ratio: Union[float, None] = None,
    ):

        total_body_movement = np.zeros_like(step_direction)
        # for leg in range(self.NUMBER_OF_LEG):
        for leg in [0,2,1,3]:
            leg = (leg + 0) % (self.NUMBER_OF_LEG)
            body_movement = step_direction / self.NUMBER_OF_LEG
            total_body_movement += body_movement
            target_set = np.empty_like(self.last_sent_target_set)
            target_set[:, :] = np.nan
            target_set[leg, :] = (
                self.default_target[leg, :]
                + step_direction.reshape((-1, 3))
                - total_body_movement
            )
            self.move_and_hop_stable(body_movement, target_set)

    def gait_loopv2(
        self,
        step_direction: np.ndarray = np.array([100, 100, 0], dtype=float),
        step_back_ratio: Union[float, None] = None,
    ):

        total_body_movement = np.zeros_like(step_direction)
        for leg in range(self.NUMBER_OF_LEG):
            leg = (leg + 0) % (self.NUMBER_OF_LEG)
            body_movement = step_direction / self.NUMBER_OF_LEG
            total_body_movement += body_movement
            target_set = np.empty_like(self.last_sent_target_set)
            target_set[:, :] = np.nan
            target_set[leg, :] = (
                self.default_target[leg, :]
                + step_direction.reshape((-1, 3))
                - total_body_movement
            )
            self.move_body_and_hop(body_movement, target_set)

    def get_best_foothold(
        self, legnum: int, body_pos: np.ndarray, body_quat: np.ndarray
    ) -> np.ndarray:

        potential_target = qt.rotate_vectors(
            (
                # qt.from_euler_angles(0, 0, legnum / self.NUMBER_OF_LEG * 2 * np.pi)
                body_quat
            )
            ** (-1),
            self.FOOTHOLDS - body_pos,
        )
        potential_target = potential_target.astype(np.float32)
        reachable = reach_pkg.is_reachable_array(
            potential_target, self.legs_angle[legnum]
        )
        if np.any(reachable):
            potential_target = potential_target[reachable, :]

        score = np.linalg.norm(
            reach_pkg.dist_to_avg_surf_PAarray(potential_target, self.legs_angle[legnum]),
            axis=1,
        )
        # perfect_target = self.default_target[legnum, :]
        # dist_to_perfect = np.linalg.norm(potential_target - perfect_target, axis=1)
        # dist_to_perfect = -np.minimum(dist_to_perfect, 0)
        # score = dist_to_perfect

        return potential_target[np.argmin(score), :]

    def auto_place(self, body_pos: np.ndarray, body_quat: np.ndarray) -> None:
        self.set_body_transform_rviz(body_pos / 1000, body_quat)

        target_set = np.empty_like(self.default_target)

        for legnum in range(self.NUMBER_OF_LEG):
            target_set[legnum, :] = self.get_best_foothold(legnum, body_pos, body_quat)

        self.move_body_and_hop(
            body_transl=np.zeros_like(self.body_coord), target_set=target_set
        )
        return

    def auto_place_cbk(self, tf: Transform) -> None:
        self.pwarn("autoplace")
        body_pos = np.array(
            [tf.translation.x, tf.translation.y, tf.translation.z], dtype=float
        )
        body_quat = qt.from_float_array(
            [tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z]
        )
        self.auto_place(body_pos, body_quat)
        return

    def dumb_auto_walk(
        self,
        body_shift: np.ndarray = np.array([40, 0, 0], dtype=float),
    ):

        body_shift = body_shift.astype(np.float32)
        last_targetset = self.last_sent_target_set.astype(np.float32)
        legCount = self.NUMBER_OF_LEG
        # self.get_logger().info(f"current TargetSet:\n{last_target}")

        potential_next = np.empty_like(self.last_sent_target_set).astype(np.float32)
        for leg in range(legCount):
            leg_movement = body_shift * (legCount - 1)

            potential_target = (
                self.FOOTHOLDS.copy() - self.body_coord - body_shift
            ).astype(np.float32)
            reachable = reach_pkg.is_reachable_array(
                potential_target, self.legs_angle[leg]
            )
            potential_target = potential_target[reachable, :]
            score = np.linalg.norm(
                reach_pkg.dist_to_avg_surf_PAarray(
                    potential_target, self.legs_angle[leg]
                ),
                axis=1,
            )
            # self.get_logger().warn(f"{score}")
            if np.linalg.norm(body_shift) > 0.01:
                perfect_target = last_targetset[leg, :] + leg_movement
                v = normalize(body_shift)
                dist_to_perfect = potential_target - perfect_target
                dist_to_perfect = np.sum(v * dist_to_perfect, axis=1)
                dist_to_perfect = dist_to_perfect  # - np.linalg.norm(leg_movement)
                dist_to_perfect = -np.minimum(dist_to_perfect, 0)
                # dist_to_perfect = np.abs(dist_to_perfect)
                score += dist_to_perfect

            potential_next[leg, :] = potential_target[np.argmin(score), :]

        pressure = self.compute_targetset_pressure_precise(
            last_targetset, body_shift, potential_next
        )

        # the targetset with the less pressure will be tried first
        tentative_queue = np.argsort(pressure)

        self.get_logger().info(f"tentative_queue: {tentative_queue}")
        self.get_logger().info(f"pressure: {pressure}")

        for leg in tentative_queue:

            leg_movement = body_shift * (legCount - 1)

            tentative_targetset = np.empty_like(last_targetset)
            tentative_targetset[:, :] = np.nan

            leg_target = potential_next[leg, :]
            # leg_target = last_targetset[leg, :] + body_shift * (legCount - 1)
            if np.allclose(leg_target, last_targetset[leg, :], rtol=0.1, equal_nan=True):
                pass  # kept empty with nans
            else:
                tentative_targetset[leg, :] = leg_target

            mvt_succesful = (
                self.move_and_hop_stable(body_shift, tentative_targetset) is SUCCESS
            )

            if mvt_succesful:
                return 0
        self.get_logger().warn("[simple_auto_walk] no movement found")
        return 1

    def gait_loop(
        self,
        step_direction: np.ndarray = np.array([120, 120, 0], dtype=float),
        step_back_ratio: Union[float, None] = None,
    ):
        """step of 70 70 0 and step_back_ratio of 0.7 does not fall irl"""
        if step_back_ratio is None:
            step_back_ratio = self.default_step_back_ratio
        plot_for_stability = False
        counter = 0

        step_back_mm = step_back_ratio * np.linalg.norm(step_direction)

        now_targets = self.default_target.copy()
        step_back = np.zeros(3)

        for leg in range(now_targets.shape[0]):
            target = now_targets[leg, :] + step_direction
            previous_stepback = step_back
            step_back = normalize(target * np.array([1, 1, 0])) * step_back_mm

            future_arr = []

            for ground_leg in range(now_targets.shape[0]):
                target_for_stepback = (
                    now_targets[ground_leg, :]
                    + step_back
                    - previous_stepback
                    - step_direction / 4
                )
                now_targets[ground_leg, :] = target_for_stepback

                fut = self.transl_client_arr[ground_leg].call_async(
                    self.np2tfReq(target_for_stepback)
                )
                future_arr.append(fut)
            self.manual_body_translation_rviz(
                -(step_back - previous_stepback - step_direction / 4)
            )

            if plot_for_stability:
                targets_to_plot = np.empty((4, 3), dtype=float)
                targets_to_plot[:-1, :] = np.delete(now_targets, leg, axis=0)
                targets_to_plot[-1, :] = np.delete(now_targets, leg, axis=0)[0, :]
                plt.plot(targets_to_plot[:, 0], targets_to_plot[:, 1])
                plt.scatter(0, 0, c="red")
                plt.grid()
                plt.savefig(f"{counter}.png")
                plt.clf()
                counter += 1

            self.wait_on_futures(future_arr)
            future_arr = []
            # now_targets[leg, :] = target
            now_targets[leg, :] = now_targets[leg, :] + step_direction

            fut = self.hop_client_arr[leg].call_async(self.np2tfReq(now_targets[leg, :]))
            future_arr.append(fut)

            self.wait_on_futures(future_arr)

        future_arr = []
        for ground_leg in range(now_targets.shape[0]):
            target_for_stepback = now_targets[ground_leg, :] - step_back
            now_targets[ground_leg, :] = target_for_stepback

            fut = self.transl_client_arr[ground_leg].call_async(
                self.np2tfReq(target_for_stepback)
            )
            future_arr.append(fut)
        self.manual_body_translation_rviz(step_back)
        self.wait_on_futures(future_arr)

        return

    def fence_stepover(self):
        self.gait_loopv2(np.array([500 / 3, 0, 0], dtype=float))
        self.gait_loopv2(np.array([500 / 3, 0, 0], dtype=float))
        self.gait_loopv2(np.array([500 / 3, 0, 0], dtype=float))

        body_movement = np.array([50, -50, 60], dtype=float)
        leg = 0
        leg_movement = np.array([150, -150, 140], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.default_target[leg, :] + leg_movement.reshape((-1, 3))
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([60, 0, 0], dtype=float)
        leg = 2
        leg_movement = np.array([70, 0, -60], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.default_target[leg, :] + leg_movement.reshape((-1, 3))
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([20, 50, 0], dtype=float)
        leg = 1
        leg_movement = np.array([100, 0, -60], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.default_target[leg, :] + leg_movement.reshape((-1, 3))
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([0, 0, 0], dtype=float)
        leg = 2
        leg_movement = np.array([80, 0, -60], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.default_target[leg, :] + leg_movement.reshape((-1, 3))
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([20, 0, 0], dtype=float)
        leg = 3
        leg_movement = np.array([120, 0, -50], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.default_target[leg, :] + leg_movement.reshape((-1, 3))
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([70, 0, 0], dtype=float)
        leg = 1
        leg_movement = np.array([200, 50, 200], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        # break

        body_movement = np.array([70, 0, 0], dtype=float)
        leg = 0
        leg_movement = np.array([0, 0, -50], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.default_target[leg, :] + leg_movement.reshape((-1, 3))
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([0, 0, 0], dtype=float)
        leg = 2
        leg_movement = np.array([200, 0, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        # break

        body_movement = np.array([30, 0, 0], dtype=float)
        leg = 1
        leg_movement = np.array([-30, 50, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        # break

        body_movement = np.array([0, 0, 0], dtype=float)
        leg = 0
        leg_movement = np.array([100, 0, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        # break

        body_movement = np.array([100, 0, 0], dtype=float)
        leg = 3
        leg_movement = np.array([0, 0, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([30, 0, 0], dtype=float)
        leg = 2
        leg_movement = np.array([120, 0, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([40, 0, 0], dtype=float)
        leg = 3
        leg_movement = np.array([90, -110, 200], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([100, 0, 0], dtype=float)
        leg = 2
        leg_movement = np.array([0, 0, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([0, 0, 0], dtype=float)
        leg = 3
        leg_movement = np.array([200, 150, -200], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([50, -50, 0], dtype=float)
        leg = 0
        leg_movement = np.array([200, 0, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([50, 0, 0], dtype=float)
        leg = 1
        leg_movement = np.array([-50, -100, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([100, 0, 0], dtype=float)
        leg = 2
        leg_movement = np.array([30, 150, 200], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([0, 0, 0], dtype=float)
        leg = 1
        leg_movement = np.array([200, 0, -200], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([0, 0, 0], dtype=float)
        leg = 0
        leg_movement = np.array([200, 0, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([30, 30, 0], dtype=float)
        leg = 1
        leg_movement = np.array([200, -30, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([0, 0, 0], dtype=float)
        leg = 3
        leg_movement = np.array([200, 0, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([150, 0, 0], dtype=float)
        leg = 0
        leg_movement = np.array([0, 0, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([100, 50, -60], dtype=float)
        leg = 2
        leg_movement = np.array([200, -50, -140], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.last_sent_target_set[leg, :] + leg_movement.reshape(
            (-1, 3)
        )
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([0, 0, 0], dtype=float)
        leg = 1
        leg_movement = np.array([0, 0, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.default_target[leg, :] + leg_movement.reshape((-1, 3))
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([0, 0, 0], dtype=float)
        leg = 2
        leg_movement = np.array([0, 0, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.default_target[leg, :] + leg_movement.reshape((-1, 3))
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([0, 0, 0], dtype=float)
        leg = 3
        leg_movement = np.array([0, 0, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.default_target[leg, :] + leg_movement.reshape((-1, 3))
        self.move_body_and_hop(body_movement, target_set)

        body_movement = np.array([0, 0, 0], dtype=float)
        leg = 0
        leg_movement = np.array([0, 0, 0], dtype=float)
        target_set = np.empty_like(self.last_sent_target_set)
        target_set[:, :] = np.nan
        target_set[leg, :] = self.default_target[leg, :] + leg_movement.reshape((-1, 3))
        self.move_body_and_hop(body_movement, target_set)
        return


def main(args=None):
    rclpy.init()
    node = MoverNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt as e:
        node.get_logger().debug(
            "KeyboardInterrupt caught, node shutting down cleanly\nbye bye <3"
        )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
