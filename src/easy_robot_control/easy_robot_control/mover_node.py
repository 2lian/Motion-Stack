"""
This node is responsible for synchronising several leg movement in order to move the
cente body and perform steps.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

from typing import Optional

import numpy as np
import quaternion as qt
from geometry_msgs.msg import Transform
from motion_stack_msgs.srv import ReturnTargetSet, ReturnVect3, SendTargetBody, TFService
from numpy.typing import NDArray
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup
from rclpy.node import List, Service
from rclpy.publisher import Publisher
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from easy_robot_control.EliaNode import (
    Client,
    EliaNode,
    error_catcher,
    myMain,
    np2TargetSet,
    np2tf,
    targetSet2np,
    tf2np,
)

SUCCESS = 0


class MoverNode(EliaNode):

    def __init__(self):
        # rclpy.init()
        super().__init__("mover")  # type: ignore

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

        self.declare_parameter("leg_list", [0])
        self.LEG_LIST: List[int] = (  # type: ignore
            self.get_parameter("leg_list").get_parameter_value().integer_array_value
        )
        self.declare_parameter("mvmt_update_rate", 30.0)
        self.MOVEMENT_UPDATE_RATE = (
            self.get_parameter("mvmt_update_rate").get_parameter_value().double_value
        )
        self.body_coord = np.zeros(shape=(3,), dtype=float)
        self.body_quat = qt.one.copy()

        alive_client_list = [f"leg{leg}/leg_alive" for leg in self.LEG_LIST]
        self.wait_for_lower_level(alive_client_list, all_requiered=True)

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.ik_pub_arr: List[Publisher] = []
        self.roll_speed_pub: List[Publisher] = []
        self.rviz_smooths: List[Publisher] = []
        self.rviz_teleports: List[Publisher] = []

        for leg in self.LEG_LIST:
            self.ik_pub_arr.append(
                self.create_publisher(
                    Transform,
                    f"leg{leg}/set_ik_target",
                    10,
                )
            )
            self.roll_speed_pub.append(
                self.create_publisher(
                    Float64,
                    f"leg{leg}/smart_roll",
                    10,
                )
            )
            self.rviz_smooths.append(
                self.create_publisher(Transform, f"leg{leg}/smooth_body_rviz", 10)
            )
            self.rviz_teleports.append(
                self.create_publisher(Transform, f"leg{leg}/robot_body", 10)
            )
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

        for leg in self.LEG_LIST:
            cli_name = f"leg{leg}/rel_transl"
            self.transl_client_arr.append(
                self.get_and_wait_Client(cli_name, LEG_MOVEMENT_MSG_TYPE)
            )
            cli_name = f"leg{leg}/rel_hop"
            self.hop_client_arr.append(
                self.get_and_wait_Client(cli_name, LEG_MOVEMENT_MSG_TYPE)
            )
            cli_name = f"leg{leg}/shift"
            self.shift_client_arr.append(
                self.get_and_wait_Client(cli_name, LEG_MOVEMENT_MSG_TYPE)
            )
            cli_name = f"leg{leg}/rot"
            self.rot_client_arr.append(
                self.get_and_wait_Client(cli_name, LEG_MOVEMENT_MSG_TYPE)
            )
            cli_name = f"leg{leg}/tip_pos"
            self.tip_pos_client_arr.append(
                self.get_and_wait_Client(cli_name, ReturnVect3)
            )
            cli_name = f"leg{leg}/point"
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
        self.pinfo(f"{self.get_name()} spinning :)")

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
        # self.pwarn(ts)
        # self.pwarn(bodyxyz)
        # self.pwarn(bodyQuat)
        self.move_body_and_hop(bodyxyz, ts, bodyQuat)
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

        for service in self.tip_pos_client_arr:
            fut = service.call_async(ReturnVect3.Request())
            future_arr.append(fut)
        self.wait_on_futures(future_arr, 50)

        for leg in range(len(future_arr)):
            response = future_arr[leg].result().vector
            self.live_target_set[leg, :] = (response.x, response.y, response.z)
        return self.live_target_set

    @error_catcher
    def body_tfshift(self, shift: np.ndarray, rot: qt.quaternion = qt.one) -> None:
        future_list = []
        for leg_ind in range(len(self.shift_client_arr)):
            shift_msg = self.np2tfReq(-shift, qt.one)
            shift_future = self.shift_client_arr[leg_ind].call_async(shift_msg)
            future_list.append(shift_future)

            rot_msg = self.np2tfReq(shift * 0, 1 / rot)
            rot_future = self.rot_client_arr[leg_ind].call_async(rot_msg)
            future_list.append(rot_future)
        # self.manual_body_translation_rviz(shift, rot)

        # self.wait_on_futures(future_list)

    def body_shift(self, shift: np.ndarray) -> None:
        return self.body_tfshift(shift)

    def manual_body_translation_rviz(
        self, coord: np.ndarray, quat: qt.quaternion = qt.one
    ) -> None:
        self.body_coord += coord
        self.body_quat *= quat
        msg = np2tf(coord=coord, quat=quat)
        [p.publish(msg) for p in self.rviz_smooths]

    def set_body_transform_rviz(
        self, coord: np.ndarray, quat: qt.quaternion = qt.one
    ) -> None:
        self.body_coord = coord
        self.body_quat = quat
        msg = np2tf(coord, quat)
        [p.publish(msg) for p in self.rviz_teleports]

    def body_tfshift_cbk(
        self, request: TFService.Request, response: TFService.Response
    ) -> TFService.Response:
        shift, quat = tf2np(request.tf)
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

    def multi_rotate(self, target_set: np.ndarray, quat: qt.quaternion):
        future_list = []
        for leg in range(target_set.shape[0]):
            target = target_set[leg, :]
            if np.isnan(target[0]):
                continue
            fut = self.rot_client_arr[leg].call_async(self.np2tfReq(quat=quat))
            future_list.append(fut)
            self.last_sent_target_set[leg, :] += target
        return future_list

    def move_body_and_hop(
        self,
        body_xyz: np.ndarray,
        target_set: np.ndarray,
        body_quat: Optional[qt.quaternion] = None,
    ):
        is_move = ~np.isnan(target_set[:, 0])
        is_free = ~is_move & self.free_leg
        is_fix = ~is_move & (~self.free_leg)

        # those jump at the same place as before, so they can stay fixed
        # on the ground
        # can_be_fix = np.isclose(target_set, (-body_transl.reshape(1, 3)), atol=0.01)
        can_be_fix = np.isclose(
            (target_set - self.last_sent_target_set), -body_xyz, atol=0.1
        )
        can_be_fix = np.all(can_be_fix, axis=1)

        shift_target_set = np.empty_like(target_set)
        shift_target_set[:, :] = np.nan
        shift_target_set[is_fix | can_be_fix, :] = -body_xyz

        hop_target_set = target_set.copy()
        hop_target_set[can_be_fix, :] = np.nan

        future_list = []
        future_list = (
            self.multi_shift(shift_target_set)
            + self.multi_hop(hop_target_set)
            + self.multi_rotate(shift_target_set, 1 / body_quat)
        )
        mvt_is_zero = np.linalg.norm(body_xyz) < 0.0001 and qt.isclose(
            body_quat, qt.one, atol=0.01
        )
        # if not mvt_is_zero:
        # self.manual_body_translation_rviz(body_xyz, body_quat)

        # self.wait_on_futures(future_list)
        self.free_leg = is_free

        return


def main(args=None):
    myMain(MoverNode, multiThreaded=True)


if __name__ == "__main__":
    main()
