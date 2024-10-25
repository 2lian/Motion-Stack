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

        self.leg_aliveCLI: Dict[int, Client] = dict(
            [
                (l, self.create_client(Empty, f"leg{l}/leg_alive"))
                for l in range(LEGNUMS_TO_SCAN)
            ]
        )
        self.legs: Dict[int, Leg] = {}

        self.keySUB = self.create_subscription(Key, "keydown", self.keySUBCBK, 10)
        self.nokeySUB = self.create_subscription(Key, "keyup", self.nokeySUBCBK, 10)
        self.leg_scanTMR = self.create_timer(
            1, self.leg_scanTMRCBK, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.next_leg_to_scan = 0
        self.selected_joint: Optional[int] = None

        wpub = [
            "/leg11/canopen_motor/base_link1_joint_velocity_controller/command",
            "/leg11/canopen_motor/base_link2_joint_velocity_controller/command",
            "/leg13/canopen_motor/base_link1_joint_velocity_controller/command",
            "/leg13/canopen_motor/base_link2_joint_velocity_controller/command",
        ]
        self.wpub = [self.create_publisher(Float64, n, 10) for n in wpub]

    @error_catcher
    def leg_scanTMRCBK(self):

        l = self.next_leg_to_scan
        self.next_leg_to_scan = (l + 1) % LEGNUMS_TO_SCAN
        cli = self.leg_aliveCLI[l]
        if l in self.legs.keys():
            self.leg_scanTMRCBK()  # continue scanning if already scanned
            return
        if cli.wait_for_service(0.01):
            self.pinfo(f"Hey there leg{l}, nice to meet you")
            self.legs[l] = Leg(l, self)
            self.leg_scanTMRCBK()  # continue scanning if leg found
            return
        if not len(self.legs.keys()):
            self.leg_scanTMRCBK()  # continue scanning if no legs
            return

        return  # stops scanning if all fails

    @error_catcher
    def nokeySUBCBK(self, msg: Key):
        key_char = chr(msg.code)
        key_code = msg.code
        # self.pinfo(f"chr: {chr(msg.code)}, int: {msg.code}")
        self.stop_all_joints()

    def stop_all_joints(self):
        for leg in self.legs.values():
            for joint in leg.joints.keys():
                jobj = leg.get_joint_obj(joint)
                if jobj is None:
                    continue
                if jobj.angle is None:
                    continue
                if jobj.speed_target is None:
                    jobj.set_angle(angle=jobj.angle)
                else:
                    jobj.set_speed(0)

    @error_catcher
    def keySUBCBK(self, msg: Key):
        key_char = chr(msg.code)
        key_code = msg.code
        # self.pinfo(f"chr: {chr(msg.code)}, int: {msg.code}")
        s: Optional[float] = None
        if key_char == "o":
            s = 10000.0
        if key_char == "p":
            s = 0.0
        if key_char == "l":
            s = -10000.0
        if s is not None:
            self.wpub[0].publish(Float64(data=-s))
            self.wpub[1].publish(Float64(data=s))
            self.wpub[2].publish(Float64(data=-s))
            self.wpub[3].publish(Float64(data=s))
        if key_char == "0":
            for leg in self.legs.values():
                leg.go2zero()

        if key_char in [f"{num + 1}" for num in range(9)]:  # +1 to avoid 0
            self.selected_joint = int(key_char) - 1
            self.pinfo(
                f"selected joint {self.selected_joint}: "
                f"{[l.joint_name_list[self.selected_joint] for l in self.legs.values()if self.selected_joint < len(l.joint_name_list)]}"
            )
        if key_char == "r":
            self.vehicle_default()

        if self.selected_joint is not None:
            self.joint_control_key(key_char)
            return

        # if statement hell, yes bad, if you unhappy fix it
        if key_char == "w":
            for leg in self.legs.values():
                leg.move(xyz=[20, 0, 0], blocking=False)
        elif key_char == "s":
            for leg in self.legs.values():
                leg.move(xyz=[-20, 0, 0], blocking=False)

    def vehicle_default(self):
        jnums = [0, 6, 8]
        jang = [np.pi, np.pi, np.pi]
        for leg in self.legs.values():
            for num in jnums:
                jobj = leg.get_joint_obj(num)
                if jobj is None:
                    continue
                jobj.set_angle(np.pi/2)
        

    def joint_control_key(self, key_char):
        if self.selected_joint is None:
            return
        if key_char == "w":
            inc = 1.0
        elif key_char == "s":
            inc = -1.0
        else:
            return

        for leg in self.legs.values():
            jobj = leg.get_joint_obj(self.selected_joint)
            if jobj is None:
                continue
            jobj.set_speed(inc)


def main(args=None):
    myMain(KeyGaitNode, multiThreaded=True)


if __name__ == "__main__":
    main()
