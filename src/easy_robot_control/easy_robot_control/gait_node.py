"""
This node is responsible for chosing targets and positions.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

from typing import Optional
import numpy as np
from numpy.typing import NDArray
import quaternion as qt
import rclpy
from rclpy.task import Future
from rclpy.node import Node, Service, Union, List
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.publisher import Publisher
from EliaNode import (
    Client,
    EliaNode,
    error_catcher,
    np2TargetSet,
    np2tf,
    myMain,
    targetSet2np,
)

import pkg_resources
from rclpy.time import Duration
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TransformStamped, Transform

from custom_messages.srv import (
    ReturnTargetBody,
    ReturnVect3,
    Vect3,
    TFService,
    ReturnTargetSet,
    SendTargetSet,
    SendTargetBody,
)
from custom_messages.msg import TargetBody, TargetSet

float_formatter = "{:.1f}".format
np.set_printoptions(formatter={"float_kind": float_formatter})


class GaitNode(EliaNode):
    def __init__(self):
        # rclpy.init()
        super().__init__("gait_node")  # type: ignore
        self.Alias = "G"
        self.setAndBlockForNecessaryClients("mover_alive")

        # V Params V
        #   \  /   #
        #    \/    #
        self.declare_parameter("std_movement_time", 2.0)
        self.MVT_TIME = (
            self.get_parameter("std_movement_time").get_parameter_value().double_value
        )
        self.declare_parameter("robot_name", "None")
        self.ROBOT_NAME = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )
        self.declare_parameter("number_of_legs", 4)
        self.NUMBER_OF_LEG = (
            self.get_parameter("number_of_legs").get_parameter_value().integer_value
        )
        #    /\    #
        #   /  \   #
        # ^ Params ^

        # V Publishers V
        #   \  /   #
        #    \/    #
        self.ik0 = self.create_publisher(Transform, "leg0/set_ik_target", 10)
        #    /\    #
        #   /  \   #
        # ^ Publishers ^

        # V Service client V
        #   \  /   #
        #    \/    #

        self.returnTargetSet: Client = self.get_and_wait_Client(
            "get_targetset", ReturnTargetSet
        )
        self.sendTargetBody: Client = self.get_and_wait_Client(
            "go2_targetbody", SendTargetBody
        )

        #    /\    #
        #   /  \   #
        # ^ Service client ^

        # V Service server V
        #   \  /   #
        #    \/    #

        #    /\    #
        #   /  \   #
        # ^ Service server ^

        self.firstSpin = self.create_timer(
            1, self.firstSpinCBK, callback_group=MutuallyExclusiveCallbackGroup()
        )

    @error_catcher
    def firstSpinCBK(self):
        self.pinfo("go")
        self.destroy_timer(self.firstSpin)
        tsnow = self.getTargetSetBlocking()
        if self.ROBOT_NAME == "ur16_3f":
            self.ashutosh()
        return
        # self.randomPosLoop()
        # while 1:
        # self.mZeroBasicSetAndWalk()
        # self.goToTargetBodyBlocking(ts=np.array([[-1100, 0, 460]]))
        shiftcmd = self.get_and_wait_Client("leg_0_rel_transl", TFService)

        shiftcmd = self.get_and_wait_Client("leg_0_rel_transl", TFService)
        shiftcmd.call_async(
            TFService.Request(
                tf=np2tf(
                    coord=np.array([-1200, 0, 0]),
                    # quat=qt.from_rotation_vector(np.array([0, 1, 0])),
                    sendNone=True,
                )
            )
        )
        shiftcmd = self.get_and_wait_Client("leg_0_shift", TFService)
        shiftcmd.call(
            TFService.Request(
                tf=np2tf(
                    # coord=np.array([-1200,0,0]),
                    quat=qt.from_rotation_vector(np.array([0, -1, 0])),
                )
            )
        )

        while 1:
            self.crawl1Wheel()

    def ashutosh(self, res=None) -> None:

        # get the current end effector position
        get_tipCMD: Client = self.get_and_wait_Client("leg0/tip_pos", ReturnVect3)
        tip_pos_result: ReturnVect3.Response = get_tipCMD.call(
            ReturnVect3.Request(),
        )  # this blocks until recieved

        tip_pos = np.array(
            [
                tip_pos_result.vector.x,
                tip_pos_result.vector.y,
                tip_pos_result.vector.z,
            ],
            dtype=float,
        )  # get your data

        # let's go to the same postion but y=0, and reset to ee quaternion
        target = tip_pos.copy()
        target[0] += 100
        target[1] = -200
        request = TFService.Request()
        request.tf = np2tf(
            coord=target,
            quat=qt.one,
            sendNone=True,
        )
        shiftCMD = self.get_and_wait_Client("leg0/rel_transl", TFService)
        shiftCMD.call(request)

        # let's go down and back 200mm, and rotate the ee
        movement = np.array([400, 0, -200], dtype=float)

        rot_axis = np.array([1, 0, 0], dtype=float)
        rot_axis = rot_axis / np.linalg.norm(rot_axis)
        rot_magnitude = -np.pi / 2
        rot_vec = rot_magnitude * rot_axis
        rotation: qt.quaternion = qt.from_rotation_vector(rot_vec)

        request = TFService.Request()
        request.tf = np2tf(
            coord=movement,
            quat=rotation,
            sendNone=True,
        )
        shiftCMD = self.get_and_wait_Client("leg0/shift", TFService)
        shiftCMD.call(request)

        # let's go forward 400mm, wait 1 s then go left 300mm
        movement = np.array([-400, 0, 0], dtype=float)
        request = TFService.Request()
        request.tf = np2tf(
            coord=movement,
            quat=None,
            sendNone=True,
        )
        call1: Future = shiftCMD.call_async(request)
        self.sleep(1)

        movement = np.array([0, -300, 0], dtype=float)
        request = TFService.Request()
        request.tf = np2tf(
            coord=movement,
            quat=None,
            sendNone=True,
        )
        call2: Future = shiftCMD.call_async(request)

        self.wait_on_futures([call1, call2])  # will block until all your futures are done

        # lets go back to the start position, but not by doing a trajectory. Simply
        # executing the IK immediately
        self.sleep(0.3)
        msg = np2tf(
            coord=np.array([-600, -300, 800]),
            quat=None,
            sendNone=False,
        )
        self.ik0.publish(msg)
        self.ik0.publish(msg)
        self.ik0.publish(msg)
        self.ik0.publish(msg)
        self.sleep(1.5)

        self.ashutosh()

    def shiftCrawlDebbug(self, res=None):
        tsnow = self.getTargetSetBlocking()
        mvt = np.array([100, 0, 0], dtype=float)
        ts_next = tsnow + mvt.reshape(1, 3)
        self.goToTargetBodyBlocking(ts=ts_next)
        fut: Future = self.goToTargetBody(bodyXYZ=mvt)
        fut.add_done_callback(self.shiftCrawlDebbug)

    @error_catcher
    def crawl1Wheel(self):
        """for moonbot hero one arm+wheel only"""
        tsnow = self.getTargetSetBlocking()

        shiftcmd = self.get_and_wait_Client("leg_0_shift", TFService)
        mvt = np.array([400, 0, 0], dtype=float)
        speed = np.linalg.norm(mvt) / self.MVT_TIME

        rollcmd: Publisher = self.create_publisher(
            Float64,
            f"smart_roll_0",
            10,
        )
        rollcmd.publish(Float64(data=speed))

        shiftcmd.call(
            TFService.Request(
                tf=np2tf(
                    coord=mvt,
                )
            )
        )

        rollcmd.publish(Float64(data=0.0))

        self.goToTargetBodyBlocking(bodyXYZ=mvt)

    @error_catcher
    def mZeroBasicSetAndWalk(self):
        """for moonbot 0"""
        height = 220
        width = 250
        targetSet = np.array(
            [
                [width, 0, -height],
                [0, width, -height],
                [-width, 0, -height],
                [0, -width, -height],
            ],
            dtype=float,
        )

        self.sendTargetBody.call(
            SendTargetBody.Request(
                target_body=TargetBody(
                    target_set=np2TargetSet(targetSet),
                    body=np2tf(),
                )
            )
        )
        self.sendTargetBody.call(
            SendTargetBody.Request(
                target_body=TargetBody(
                    # target_set=np2TargetSet(ts),
                    body=np2tf(np.array([-50.0, 0.0, 0.0], None)),
                )
            )
        )

        for l in range(4):
            newTS = np.empty(shape=(4, 3), dtype=float)
            newTS[:, :] = np.nan
            newTS[l, :] = targetSet[l, :]
            self.sendTargetBody.call(
                SendTargetBody.Request(
                    target_body=TargetBody(
                        target_set=np2TargetSet(newTS),
                        body=np2tf(np.array([0.0, 0.0, 0.0], None)),
                    )
                )
            )

    def randomPosLoop(self):
        """for multi legged robots"""
        self.goToTargetBodyBlocking(bodyXYZ=np.array([-200, 0, 0], dtype=float))
        d = 400
        v = np.random.random(size=(3,)) * 0
        while 1:
            dir = np.random.random(size=(3,))
            vnew = (dir - 0.5) * d
            self.goToTargetBodyBlocking(bodyXYZ=vnew - v)
            v = vnew

    def getTargetSet(self) -> Future:
        call = self.returnTargetSet.call_async(ReturnTargetSet.Request())
        processFuture = Future()

        @error_catcher
        def processMessage(f: Future) -> None:
            response: Optional[ReturnTargetSet.Response] = f.result()
            assert response is not None
            ts = targetSet2np(response.target_set)
            processFuture.set_result(ts)

        call.add_done_callback(processMessage)
        return processFuture

    def getTargetSetBlocking(self) -> NDArray:
        response: ReturnTargetSet.Response = self.returnTargetSet.call(
            ReturnTargetSet.Request()
        )
        return targetSet2np(response.target_set)

    def goToTargetBody(
        self,
        ts: Optional[NDArray] = None,
        bodyXYZ: Optional[NDArray] = None,
        bodyQuat: Optional[qt.quaternion] = None,
    ) -> Future:
        call = self.sendTargetBody.call_async(
            SendTargetBody.Request(
                target_body=TargetBody(
                    target_set=np2TargetSet(ts),
                    body=np2tf(bodyXYZ, bodyQuat),
                )
            )
        )
        return call

    def goToTargetBodyBlocking(
        self,
        ts: Optional[NDArray] = None,
        bodyXYZ: Optional[NDArray] = None,
        bodyQuat: Optional[qt.quaternion] = None,
    ) -> SendTargetBody.Response:
        call = self.sendTargetBody.call(
            SendTargetBody.Request(
                target_body=TargetBody(
                    target_set=np2TargetSet(ts),
                    body=np2tf(bodyXYZ, bodyQuat),
                )
            )
        )
        return call

    def crawlToTargetSet(self, NDArray) -> None: ...

    def goToDefault(self): ...

    def stand(self): ...


def main(args=None):
    myMain(GaitNode, multiThreaded=True)


if __name__ == "__main__":
    main()
