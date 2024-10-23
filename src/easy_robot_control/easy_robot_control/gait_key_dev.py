from typing import Dict, Final, Literal, Optional, Sequence, overload
import re
import numpy as np
from numpy.typing import NDArray
import quaternion as qt
import rclpy
from rclpy.task import Future
from rclpy.node import Union, List
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

from std_msgs.msg import Float64
from geometry_msgs.msg import Transform

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
from keyboard_msgs.msg import Key
from std_srvs.srv import Empty

from easy_robot_control.gait_node import Leg, MVT2SRV, AvailableMvt

LEGNUMS_TO_SCAN = 10


class KeyGaitNode(EliaNode):
    def __init__(self, name: str = "keygait_node"):
        super().__init__(name)
        self.Alias = "G"
        self.setAndBlockForNecessaryClients("mover_alive")

        self.leg_scaners: Dict[int, Client] = dict(
            [
                (l, self.create_client(Empty, f"leg{l}/leg_alive"))
                for l in range(LEGNUMS_TO_SCAN)
            ]
        )
        self.legs: Dict[int, Leg] = {}

        self.keySUB = self.create_subscription(Key, "keydown", self.keySUBCBK, 10)
        self.leg_scanTMR = self.create_timer(1, self.leg_scanTMRCBK)
        self.next_leg_to_scan = 0

    @error_catcher
    def leg_scanTMRCBK(self):
        time_to_next_scan = self.leg_scanTMR.timer_period_ns / 1e9
        l = self.next_leg_to_scan
        self.next_leg_to_scan = (l + 1) % LEGNUMS_TO_SCAN
        cli = self.leg_scaners[l]
        if l in self.legs.keys():
            self.leg_scanTMRCBK()  # continue scanning if already scanned
            return
        if cli.wait_for_service(0.01):
            self.pinfo(f"Hey there leg{l}, nice to meet you")
            self.legs[l] = Leg(l, self)
            self.leg_scanTMRCBK()  # continue scanning if service ping succesful
            return
        else:
            return  # stops scanning for this tick if service ping fails

    @error_catcher
    def keySUBCBK(self, msg: Key):
        key_char = chr(msg.code)
        key_code = msg.code
        self.pinfo(f"chr: {chr(msg.code)}, int: {msg.code}")

        # if statement hell, yes bad, if you unhappy fix it
        if key_char == "w":
            for leg in self.legs.values():
                leg.move(xyz=[100, 0, 0], blocking=False)
        elif key_char == "s":
            for leg in self.legs.values():
                leg.move(xyz=[-100, 0, 0], blocking=False)


def main(args=None):
    myMain(KeyGaitNode, multiThreaded=True)


if __name__ == "__main__":
    main()
