from os import environ
from os.path import join

import matplotlib
from easy_robot_control.utils.joint_state_util import js_from_ros
from sensor_msgs.msg import JointState

matplotlib.use("Agg")  # fix for when there is no display

import csv

# from pytest import ExitCode
from typing import Dict, List, Optional, Tuple

import numpy as np
from easy_robot_control.EliaNode import (
    EliaNode,
    bcolors,
    error_catcher,
    get_src_folder,
    myMain,
    replace_incompatible_char_ros2,
)
from motion_stack.core.utils.csv import csv_to_dict, update_csv
from motion_stack_msgs.srv import SendJointState
from rclpy.node import Client
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, Float64

EMULATE_PHOTO = False  # True for debug when usin rviz

STEP_RAD = 0.2  # rad
MAX_SAMPLE = 10

MOONBOT_PC_NUMBER = str(environ.get("M_LEG"))  # leg number saved on lattepanda
if MOONBOT_PC_NUMBER is None:
    MOONBOT_PC_NUMBER = "1"

csv_name = f"offset_M{MOONBOT_PC_NUMBER}.csv"
CSV_PATH = join(get_src_folder("ros2_m_hero_pkg"), csv_name)
# CSV_PATH = join(get_package_share_directory("easy_robot_control"), "offsets.csv")

URDFJointName = str
JOINTS: List[URDFJointName] = [
    f"leg{MOONBOT_PC_NUMBER}grip1",
    f"leg{MOONBOT_PC_NUMBER}base_link_link2",
    f"leg{MOONBOT_PC_NUMBER}link2_link3",
    f"leg{MOONBOT_PC_NUMBER}link3_link4",
    f"leg{MOONBOT_PC_NUMBER}link4_link5",
    f"leg{MOONBOT_PC_NUMBER}link5_link6",
    f"leg{MOONBOT_PC_NUMBER}link6_link7",
    f"leg{MOONBOT_PC_NUMBER}link7_link8",
    f"leg{MOONBOT_PC_NUMBER}grip2",
]
# JOINTS = [replace_incompatible_char_ros2(n) for n in JOINTS]

p = [f"photo_{t+1}" for t in range(len(JOINTS))]
PHOTO_TOPIC = dict(zip(JOINTS, p))

DIRECTION: Dict[str, int] = {
    JOINTS[0]: 1,
    JOINTS[1]: 1,
    JOINTS[2]: 1,
    JOINTS[3]: 1,
    JOINTS[4]: 1,
    JOINTS[5]: 1,
    JOINTS[6]: 1,
    JOINTS[7]: 1,  
    JOINTS[8]: 1,
}

JS_SEND = "joint_set"
JS_READ = "joint_read"


def is_upper_front(before: bool, after: bool) -> Optional[bool]:
    """find if the transisiton is a front going up (True) or down (False).
    Returns None if not front.
    """
    if before == after:
        return None
    if before and not after:  # 1->0
        return False
    else:  # 0->1
        return True


class Joint:
    def __init__(self, name: str, parent: "LimitGoNode"):
        self.photopic = PHOTO_TOPIC[name]
        self.parent = parent

        self.name: str = name

        self.angle: Optional[float] = None
        self.previous_ang: float = 0.0
        self.command: Optional[float] = None
        self.increment = 0.01
        self.last_com_time: Time = self.parent.getNow()
        self.photo_value: Optional[bool] = None

        offset_not_defined = not self.name in self.parent.OFFSETS.keys()
        self.offset_from_upper: Optional[float]
        if offset_not_defined:
            self.offset_from_upper = None
            self.parent.pinfo(f"Offset does not exit for {self.name}")
            self.parent.pinfo(f"{self.parent.OFFSETS.keys()}")
        else:
            self.offset_from_upper = self.parent.OFFSETS[self.name]

        self.upper_limit: Optional[float] = None
        self.lower_limit: Optional[float] = None
        self.upper_samples: List[float] = []
        self.lower_samples: List[float] = []

        if DIRECTION[self.name] == 0:  # skips
            return
        self.other_side = DIRECTION[self.name] < 0

        self.photoSUB = self.parent.create_subscription(
            Bool, f"{self.photopic}", self.photoCBK, 10
        )
        self.save = self.parent.create_subscription(
            Empty,
            f"save_offset_{replace_incompatible_char_ros2(self.name)}",
            self.save_as_offsetCBK,
            10,
        )
        self.pub = self.parent.create_publisher(JointState, JS_SEND, 10)
        if EMULATE_PHOTO:
            self.fake_photoPUB = self.parent.create_publisher(Bool, self.photopic, 10)

        # self.sinTMR = self.parent.create_timer(0.1, self.sinmove)
        # return
        self.oneTMR = self.parent.create_timer(1, self.one_sec_tmrCBK)
        self.init_dispTMR = self.parent.create_timer(0.1, self.init_dispTMRCBK)
        self.display_updateTMR = self.parent.create_timer(1, self.display_updateTMRCBK)

    @error_catcher
    def display_updateTMRCBK(self):
        if max(len(self.upper_samples), len(self.upper_samples)) == 0:
            return
        if self.photo_value is None:
            return
        if self.angle is None:
            return
        if self.is_it_done():
            return
        is_up_trans = len(self.lower_samples) < len(self.upper_samples)
        i = "upper" if is_up_trans else "lower"
        l = self.upper_samples if is_up_trans else self.lower_samples
        avg = float(np.average(l))
        self.parent.pinfo(
            f"{self.name}: {i} transition around {avg:.2f} "
            f"(sample {len(self.upper_samples) + 1}/{MAX_SAMPLE})"
        )

    def move_based_on_photo(self):
        if self.photo_value is None:
            return
        if self.angle is None:
            return
        if max(len(self.upper_samples), len(self.upper_samples)) > MAX_SAMPLE:
            return
        is_high = self.photo_value
        if self.other_side:
            is_high = not is_high
        if is_high:
            target = self.angle + STEP_RAD
        else:
            target = self.angle - STEP_RAD

        self.send_angle(target)

    @error_catcher
    def photoCBK(self, msg: Bool) -> None:
        if self.is_it_done():
            return
        new_state = msg.data
        self.analyse_photo_transi(new_state)
        self.photo_value = new_state
        self.move_based_on_photo()

    def analyse_photo_transi(self, new_state: bool) -> None:
        if self.photo_value is None:  # first photo read
            self.parent.pinfo(f"{self.photoSUB.topic} first read {new_state}")
            return
        if self.angle is None:
            return
        if self.command is None:
            return

        is_up_trans = self.is_upper_transition(new_state)
        if is_up_trans is None:  # no change on photo
            return

        if is_up_trans:
            self.upper_samples.append(self.angle)
        else:
            self.lower_samples.append(self.angle)
        return

    def is_it_done(self):
        if self.lower_limit is not None:
            return True
        if self.upper_limit is not None:
            return True
        if len(self.upper_samples) >= MAX_SAMPLE:
            self.upper_limit = float(np.average(self.upper_samples))
            self.parent.pinfo(
                f"{bcolors.OKGREEN}[{self.photoSUB.topic_name}] Upper transition found at "
                f"~{self.upper_limit:.2f} :){bcolors.ENDC}"
            )
            self.go_to_default()
            return True
        if len(self.lower_samples) >= MAX_SAMPLE:
            self.lower_limit = float(np.average(self.lower_samples))
            self.parent.pinfo(
                f"{bcolors.OKGREEN}[{self.photoSUB.topic_name}] Lower transition found at "
                f"~{self.lower_limit:.2f} :){bcolors.ENDC}"
            )
            self.go_to_default()
            return True

    def is_upper_transition(self, new_photo_read: bool) -> Optional[bool]:
        if self.photo_value is None:  # first photo read
            return None
        if self.angle is None:
            return None
        if self.command is None:
            return None

        front_up = is_upper_front(self.photo_value, new_photo_read)
        if front_up is None:
            return None
        going_up = self.command > self.angle
        return going_up == front_up

    def go_to_default(self):
        if self.upper_limit is None and self.lower_limit is None:
            self.parent.pwarn(
                f"transition not yet found on {self.name}. Please start the calibration again."
            )
            return

        transition = (
            self.upper_limit if self.upper_limit is not None else self.lower_limit
        )
        ul = "upper" if self.upper_limit is not None else "lower"

        if self.offset_from_upper is None:
            self.parent.pwarn(
                f"The default position relative to the upper transition is not "
                f"defined for {self.name}"
            )
            self.parent.pinfo(
                f"the current upper transition is: {self.upper_limit}\n"
                f"now go to the default position you want to set as zero "
                f".\n Then save this position using"
                f"'ros2 topic pub {self.save.topic_name} std_msgs/msg/Empty'"
            )
            return

        zero_angle = transition + self.offset_from_upper
        # assert self.lower_limit - np.pi < zero_angle < self.lower_limit
        # self.send_angle(zero_angle)

        req = SendJointState.Request()
        req.js.name = [self.name]
        req.js.position = [zero_angle]
        self.parent.set_offSRV.call_async(req)

        self.parent.pinfo(
            f"{self.name} going to zero. "
            f"{ul} transition: {transition}, zero: {zero_angle}"
        )
        self.send_angle(0)

    @error_catcher
    def save_as_offsetCBK(self, msg):
        if self.upper_samples is None:
            self.parent.pwarn("Cannot save upper transition not found")
            return
        if self.angle is None:
            self.parent.pwarn("Cannot save no angle readings received")
            return
        off = self.angle - self.lower_limit
        self.parent.pinfo(f"Offset joint is '{self.name},{off}' :)")
        update_csv(CSV_PATH, self.name[4:], off)

    @error_catcher
    def init_dispTMRCBK(self):
        if self.photo_value is None:  # first photo read
            return
        if self.angle is None:
            return
        if self.command is None:
            return

        if self.offset_from_upper is None:
            dispoff = "UNKNOWN"
        else:
            dispoff = f"{self.offset_from_upper:.2f}"

        if self.command > self.angle:
            i = "up"
        else:
            i = "down"

        self.parent.pinfo(
            f"{bcolors.OKCYAN}{self.name} initialized{bcolors.ENDC}, "
            f"photo state: {self.photo_value}, joint moving {i}, "
            f"current angle: {self.angle:.2f}, offset from upper limit: {dispoff}"
        )
        self.parent.destroy_timer(self.init_dispTMR)

    @error_catcher
    def readCBK(self, msg: Float64) -> None:
        if self.angle is None:  # first read
            self.previous_ang = msg.data
        else:
            self.previous_ang = self.angle

        self.angle = msg.data
        self.emulate_photoreceptor()

    def emulate_photoreceptor(self):
        if self.angle is None:
            return
        if EMULATE_PHOTO:
            if self.offset_from_upper is None:
                target = -0.5
            else:
                target = self.offset_from_upper
            target = -target
            self.fake_photoPUB.publish(
                Bool(
                    data=(target - np.pi < self.angle < target),
                )
            )
        else:
            return

    @error_catcher
    def one_sec_tmrCBK(self):
        if self.photo_value is None:
            self.parent.pinfo(
                f"{self.photoSUB.topic} listening but received nothing after 1s,"
                f" {bcolors.FAIL}might not be published{bcolors.ENDC}"
            )
        if self.angle is None:
            self.parent.pinfo(
                f"{self.parent.sensorSUB.topic_name}:-{self.name} "
                f"listening but received nothing after 1s,"
                f" {bcolors.FAIL}might not be published{bcolors.ENDC}"
            )
        self.parent.destroy_timer(self.oneTMR)

    def send_angle(self, ang: float) -> None:
        # self.parent.pwarn(f"{self.pub.topic_name}: {ang}")
        self.command = float(ang)
        self.last_com_time = self.parent.getNow()
        msg = JointState()
        msg.header.stamp = self.parent.getNow().to_msg()
        msg.name = [self.name]
        msg.position = [float(ang)]
        self.pub.publish(msg)


class LimitGoNode(EliaNode):
    def __init__(self):
        super().__init__("limit_go_node")  # type: ignore
        self.NAMESPACE = self.get_namespace()
        self.Alias = "LG"

        self.OFFSETS = csv_to_dict(CSV_PATH)
        if self.OFFSETS is None:
            raise Exception(f"Could not load {CSV_PATH}")
        off = [
            (f"leg{MOONBOT_PC_NUMBER}{key}", val) for key, val in self.OFFSETS.items()
        ]
        self.OFFSETS = dict(off)
        self.pinfo(
            f"Offsets (zero position relative to upper limit), loaded from {CSV_PATH}, "
            f"defined as {self.OFFSETS}"
        )
        self.set_offSRV: Client = self.get_and_wait_Client(
            f"/leg{MOONBOT_PC_NUMBER}/set_offset", SendJointState
        )
        self.sensorSUB = self.create_subscription(
            JointState, JS_READ, self.js_sensorCBK, 10
        )

        self.wait_for_lower_level(["joint_alive"])

        self.jointDic: Dict[str, Joint] = {}  # reader topic name -> Joint obj

        for j in JOINTS:
            self.jointDic[j] = Joint(j, self)

        self.recovering = False

    def js_sensorCBK(self, msg: JointState):
        states = js_from_ros(msg)
        for state in states:
            if state.position is None:
                continue
            jobj = self.jointDic.get(state.name)
            if jobj is None:
                # self.pinfo(f"Untracked: {state.name}")
                continue
            jobj.readCBK(Float64(data=float(state.position)))


def main(args=None):
    myMain(LimitGoNode)


if __name__ == "__main__":
    main()
