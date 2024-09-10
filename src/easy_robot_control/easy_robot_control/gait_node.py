"""
This node is responsible for chosing targets and positions.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

from typing import Optional
import numpy as np
from numpy.typing import NDArray
import quaternion as qt
import matplotlib.pyplot as plt
import rclpy
from rclpy.task import Future
from rclpy.node import Node, Union, List
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
import python_package_include.distance_and_reachable_function as reach_pkg
import python_package_include.multi_leg_gradient as multi_pkg
import python_package_include.stability as stab_pkg
import python_package_include.inverse_kinematics as ik_pkg

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
        # self.goToTargetBodyBlocking(ts=np.array([[-1100, 0, 460]]))

        shiftcmd = self.get_and_wait_Client("leg_0_shift", TFService)
        shiftcmd.call(
            TFService.Request(
                tf=np2tf(
                    coord=np.array([-480,0,460]),
                    quat=qt.from_rotation_vector(np.array([0, -1, 0])),
                )
            )
        )

        while 1:
            self.crawl1Wheel()


    @error_catcher
    def crawl1Wheel(self):
        tsnow = self.getTargetSetBlocking()

        shiftcmd = self.get_and_wait_Client("leg_0_shift", TFService)
        mvt = np.array([600, 0, 0], dtype=float)
        speed = np.linalg.norm(mvt) / self.MVT_TIME

        rollcmd: Publisher = self.create_publisher(
            Float64,
            f"smart_roll_0",
            10,
        )
        rollcmd.publish(Float64(data=-speed))

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
    def mZeroSetAndWalk(self):
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

    @error_catcher
    def randomPosLoop(self):
        d = 400
        v = np.random.random(size=(3,)) * 0
        while 1:
            dir = np.random.random(size=(3,))
            vnew = (dir - 0.5) * d
            self.goToTargetBodyBlocking(bodyXYZ=vnew - v)
            v = vnew

    @error_catcher
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

    @error_catcher
    def getTargetSetBlocking(self) -> NDArray:
        response: ReturnTargetSet.Response = self.returnTargetSet.call(
            ReturnTargetSet.Request()
        )
        return targetSet2np(response.target_set)

    @error_catcher
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

    @error_catcher
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
