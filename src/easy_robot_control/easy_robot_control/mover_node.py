"""
This node is responsible for synchronising several leg movement in order to move the
cente body and perform steps.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

import traceback
from typing import Optional
from custom_messages.msg import TargetSet
import numpy as np
from numpy.typing import NDArray
import quaternion as qt
import time
import matplotlib.pyplot as plt
import rclpy
from rclpy.task import Future
from rclpy.node import Node, Service, Union, List
from rclpy.callback_groups import (
    CallbackGroup,
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.publisher import Publisher
from EliaNode import Client, EliaNode, error_catcher, tf2np

import pkg_resources
from rclpy.time import Duration
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TransformStamped, Transform

from custom_messages.srv import (
    ReturnTargetSet,
    ReturnVect3,
    SendTargetBody,
    SendTargetSet,
    Vect3,
    TFService,
)
import python_package_include.distance_and_reachable_function as reach_pkg
import python_package_include.multi_leg_gradient as multi_pkg
import python_package_include.stability as stab_pkg
import python_package_include.inverse_kinematics as ik_pkg

from easy_robot_control.EliaNode import myMain, targetSet2np, np2TargetSet

SUCCESS = 0
MAP_PATH = pkg_resources.resource_filename(__name__, "python_package_include/map.npy")


class MoverNode(EliaNode):

    def __init__(self):
        # rclpy.init()
        super().__init__("mover_node")  # type: ignore

        self.declare_parameter("number_of_legs", 4)
        self.NUMBER_OF_LEG = (
            self.get_parameter("number_of_legs").get_parameter_value().integer_value
        )

        self.Alias = "M"
        self.free_leg = np.zeros(self.NUMBER_OF_LEG, dtype=bool)
        self.last_sent_target_set = np.empty((self.NUMBER_OF_LEG, 3), dtype=float)
        self.live_target_set = np.empty((self.NUMBER_OF_LEG, 3), dtype=float)

        self.declare_parameter("std_movement_time", 1.0)
        self.MOVEMENT_TIME = (
            self.get_parameter("std_movement_time").get_parameter_value().double_value
        )

        self.declare_parameter("movement_update_rate", 30.0)
        self.MOVEMENT_UPDATE_RATE = (
            self.get_parameter("movement_update_rate").get_parameter_value().double_value
        )
        self.body_coord = np.zeros(shape=(3,), dtype=float)

        alive_client_list = [f"leg_{leg}_alive" for leg in range(self.NUMBER_OF_LEG)]
        self.setAndBlockForNecessaryClients(alive_client_list)

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.ik_pub_arr: List[Publisher] = []
        self.roll_speed_pub: List[Publisher] = []

        for leg in range(self.NUMBER_OF_LEG):
            self.ik_pub_arr.append(
                self.create_publisher(
                    Vector3,
                    f"set_ik_target_{leg}",
                    10,
                )
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

        self.transl_client_arr: List[Client] = []
        self.hop_client_arr: List[Client] = []
        self.shift_client_arr: List[Client] = []
        self.rot_client_arr: List[Client] = []
        self.tip_pos_client_arr: List[Client] = []
        self.point_cli_arr: List[Client] = []

        for leg in range(self.NUMBER_OF_LEG):
            cli_name = f"leg_{leg}_rel_transl"
            self.transl_client_arr.append(
                self.get_and_wait_Client(cli_name, LEG_MOVEMENT_MSG_TYPE)
            )
            cli_name = f"leg_{leg}_rel_hop"
            self.hop_client_arr.append(
                self.get_and_wait_Client(cli_name, LEG_MOVEMENT_MSG_TYPE)
            )
            cli_name = f"leg_{leg}_shift"
            self.shift_client_arr.append(
                self.get_and_wait_Client(cli_name, LEG_MOVEMENT_MSG_TYPE)
            )
            cli_name = f"leg_{leg}_rot"
            self.rot_client_arr.append(
                self.get_and_wait_Client(cli_name, LEG_MOVEMENT_MSG_TYPE)
            )
            cli_name = f"leg_{leg}_tip_pos"
            self.tip_pos_client_arr.append(
                self.get_and_wait_Client(cli_name, ReturnVect3)
            )
            cli_name = f"leg_{leg}_point"
            self.point_cli_arr.append(self.get_and_wait_Client(cli_name, TFService))

        #    /\    #
        #   /  \   #
        # ^ Service client ^

        # V Service server V
        #   \  /   #
        #    \/    #
        self.PARALEL_GRP: CallbackGroup = ReentrantCallbackGroup()
        self.iAmAlive: Optional[Service] = None

        self.create_service(
            TFService,
            "body_tfshift",
            self.body_tfshift_cbk,
            callback_group=self.PARALEL_GRP,
        )
        self.create_service(
            SendTargetBody,
            "go2_targetbody",
            self.go2_targetbodyCBK,
            callback_group=self.PARALEL_GRP,
        )
        self.create_service(
            ReturnTargetSet,
            "get_targetset",
            self.get_targetsetCBK,
            callback_group=self.PARALEL_GRP,
        )

        #    /\    #
        #   /  \   #
        # ^ Service server ^

        self.firstSpin = self.create_timer(
            timer_period_sec=1,
            callback=self.firstSpinCBK,
            callback_group=self.PARALEL_GRP,
        )

    @error_catcher
    def firstSpinCBK(self) -> None:
        self.destroy_timer(self.firstSpin)
        self.update_tip_pos()
        self.last_sent_target_set = self.live_target_set
        self.create_service(Empty, "mover_alive", lambda req, res: res)

    @error_catcher
    def go2_targetbodyCBK(
        self, req: SendTargetBody.Request, res: SendTargetBody.Response
    ) -> SendTargetBody.Response:
        ts: NDArray = targetSet2np(req.target_body.target_set)
        if ts.shape[0] == 0:
            ts = np.empty_like(self.live_target_set)
            ts[:, :] = np.nan
        bodytf: Transform = req.target_body.body
        bodyxyz, bodyQuat = tf2np(bodytf)
        self.pwarn(ts)
        self.pwarn(bodyxyz)
        self.pwarn(bodyQuat)
        self.move_body_and_hop(bodyxyz, ts)
        return res

    @error_catcher
    def get_targetsetCBK(
        self, req: ReturnTargetSet.Request, res: ReturnTargetSet.Response
    ) -> ReturnTargetSet.Response:
        ts = self.update_tip_pos()
        res.target_set = np2TargetSet(ts)
        return res

    @error_catcher
    def update_tip_pos(self) -> NDArray:
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

    @error_catcher
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
        msg = self.np2tf(coord=coord, quat=quat)
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


def main(args=None):
    myMain(MoverNode, multiThreaded=True)


if __name__ == "__main__":
    main()
