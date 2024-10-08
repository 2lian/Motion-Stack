from os.path import join
from launch_ros.substitutions.find_package import get_package_share_directory
import matplotlib

matplotlib.use("Agg")  # fix for when there is no display

from rclpy.time import Duration, Time
from std_msgs.msg import Empty, Float64
from std_srvs.srv import Trigger
from std_srvs.srv import Empty as EmptySrv
import csv


# from pytest import ExitCode
from typing import Dict, List, Optional, Tuple
import re

import numpy as np
from numpy.typing import NDArray
from rclpy.node import (
    Publisher,
    Subscription,
    Timer,
)

from EliaNode import EliaNode, rosTime2Float
from EliaNode import (
    replace_incompatible_char_ros2,
    error_catcher,
    myMain,
    bcolors,
    get_src_folder,
)

BYPASS_RECOVERY = True # True for debug when usin rviz

POST_RECOVER_SLEEP = 1  # s
SAFETY_MARGIN = 0.2  # rad
MOVING_TOL = 0.001  # rad
STEP_PERIOD = 1  # s
WAIT_AFTER_COMMAND = STEP_PERIOD * 0.9  # s
assert STEP_PERIOD > WAIT_AFTER_COMMAND
STEP_RAD = 0.4  # rad
ON_TARGET_TOL = STEP_RAD * 0.98  # rad

# CSV_PATH = join(get_package_share_directory("easy_robot_control"), "offsets.csv")
CSV_PATH = join(get_src_folder("easy_robot_control"), "offsets.csv")

URDFJointName = str
JOINTS: List[URDFJointName] = [
    "base_link-link2",
    "link2-link3",
    "link3-link4",
    "link4-link5",
    "link5-link6",
    "link6-link7",
    "link7-link8",
]
JOINTS = [replace_incompatible_char_ros2(n) for n in JOINTS]

DIRECTION: Dict[str, int] = {
        JOINTS[0]: 1,
        JOINTS[1]: 1,
        JOINTS[2]: 1,
        JOINTS[3]: 1,
        JOINTS[4]: 1,
        JOINTS[5]: 1,
        JOINTS[6]: 1,
}

# DIRECTION = {
#     "leg3_joint1": 0,
#     "leg3_joint2": 0,
#     "leg3_joint3": 0,
#     "leg3_joint4": 0,
#     "leg3_joint5": 0,
#     "leg3_joint6": 0,
#     "leg3_steering_joint": 1,
# }

FLIP = 1


def update_csv(file_path, new_str: str, new_float: float) -> None:
    # Read the existing CSV data into a list
    rows = []
    str_found = False

    # Open the file in read mode
    with open(file_path, mode="r") as file:
        reader = csv.reader(file)
        for row in reader:
            if row and row[0] == new_str:
                # If the string is found, update the float value
                row[1] = str(new_float)
                str_found = True
            rows.append(row)

    # If the string is not found, append a new row
    if not str_found:
        rows.append([new_str, str(new_float)])

    # Write the updated data back to the CSV file
    with open(file_path, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerows(rows)


def csv_to_dict(file_path):
    data_dict = {}

    # Open the CSV file in read mode
    with open(file_path, mode="r") as file:
        reader = csv.reader(file)

        # Iterate through each row in the CSV
        for row in reader:
            # Assuming the first column is string and the second column is float
            data_dict[row[0]] = float(row[1])

    return data_dict


class Joint:
    def __init__(
        self, reader_topic_name: str, setter_topic_name: str, parent: "LimitGoNode"
    ):
        self.reader_topic_name = reader_topic_name
        self.setter_topic_name = setter_topic_name
        self.parent = parent

        n = self.parent.exctract_joint(self.setter_topic_name)
        assert n is not None
        self.jName: str = n

        self.pastAngle: Optional[float] = None
        self.angle: Optional[float] = None
        self.previous_ang: float = 0.0
        self.command: Optional[float] = None
        self.isMoving = False
        self.stuck = False
        self.increment = 0.01
        self.last_com_time: Time = self.parent.getNow()
        offset_not_defined = not self.jName in self.parent.OFFSETS.keys()
        self.offset_from_upper: Optional[float]
        if offset_not_defined:
            self.offset_from_upper = None
            self.parent.pinfo(f"Offset does not exit for {self.jName}")
            self.parent.pinfo(f"{self.parent.OFFSETS.keys()}")
        else:
            self.offset_from_upper = self.parent.OFFSETS[self.jName]

        self.upper_limit: Optional[float] = None
        self.lower_limit: Optional[float] = None

        if BYPASS_RECOVERY:
            self.recov_serv = self.parent.get_and_wait_Client("joint_alive", EmptySrv)
        else:
            self.recov_serv = self.parent.get_and_wait_Client(
            "/maxon/driver/recover", Trigger
            )
        self.sub = self.parent.create_subscription(
            Float64, self.reader_topic_name, self.readCBK, 10
        )
        self.save = self.parent.create_subscription(
            Empty, f"save_offset_{self.jName}", self.save_as_offsetCBK, 10
        )
        self.pub = self.parent.create_publisher(Float64, self.setter_topic_name, 10)
        self.moveSub = self.parent.create_subscription(
            Float64, f"test_{self.jName}", self.try_to_reachCBK, 10
        )

        # self.sinTMR = self.parent.create_timer(0.1, self.sinmove)
        # return
        self.oneTMR = self.parent.create_timer(1, self.one_sec_tmrCBK)
        self.movecheckTMR = self.parent.create_timer(0.1, self.move_check_tmrCBK)
        self.movecheckTMR.cancel()
        self.auto_recoverTMR = self.parent.create_timer(1, self.auto_recover_tmrCBK)

    # def sinmove(self):
    #     s = rosTime2Float(self.parent.getNow())
    #     self.pub.publish(Float64(data=np.sin(s * np.pi / 2)/5))
    #

    def go_to_default(self):
        if self.upper_limit is None:
            self.parent.pwarn(
                f"limit not yet found on {self.jName}. Please start the calibration again."
            )
            return

        if self.offset_from_upper is None:
            self.parent.pwarn(
                f"The default position relative to the upper limit is not "
                f"defined for {self.jName}"
            )
            self.parent.pinfo(
                f"the current upper limit is: {self.upper_limit}\n"
                f"now go to the default position you want to set as zero using "
                f"'ros2 topic pub {self.setter_topic_name} "
                """std_msgs/msg/Float64 "{data: YOUR_ANGLE_VALUE}"'"""
                f".\n Then save this position using"
                f"'ros2 topic pub {self.save.topic_name}'"
            )
            return

        zero_angle = self.upper_limit + self.offset_from_upper
        assert self.upper_limit - np.pi < zero_angle < self.upper_limit
        self.send_angle(zero_angle)
        self.parent.pinfo(
            f"{self.jName} going to zero. "
            f"Upper limit: {self.upper_limit}, zero: {zero_angle}"
        )

    @error_catcher
    def save_as_offsetCBK(self, msg):
        if self.upper_limit is None:
            self.parent.pwarn("Cannot save upper limit not found")
            return
        if self.angle is None:
            self.parent.pwarn("Cannot save no angle readings received")
            return
        update_csv(CSV_PATH, self.jName, self.angle - self.upper_limit)

    @error_catcher
    def auto_recover_tmrCBK(self):
        if self.parent.recovering:
            return
        if self.angle is None:
            return
        if self.command is None:
            return
        if not self.stuck:
            return

        limit = self.angle
        direction = self.command - limit
        direction = direction / abs(direction)

        self.save_limit(direction)
        safe_target = limit - direction * SAFETY_MARGIN
        self.send_angle(safe_target)
        self.recover()

    def save_limit(self, direction: float):
        assert direction != 0
        if direction > 0:
            self.upper_limit = self.angle
            self.parent.pinfo(f"Upper limit found at {self.upper_limit}")
        else:
            self.lower_limit = self.angle
            self.parent.pinfo(f"Lower limit found at {self.upper_limit}")

    def recover(self):
        self.parent.pinfo(f"recovering to {self.command}")
        self.parent.recovering = True
        if BYPASS_RECOVERY:
            fut = self.recov_serv.call_async(EmptySrv.Request())
        else:
            fut = self.recov_serv.call_async(Trigger.Request())
        fut.add_done_callback(self.recoveredCBK)

    @error_catcher
    def recoveredCBK(self, msg):
        self.parent.sleep(POST_RECOVER_SLEEP)
        self.parent.recovering = False
        self.stuck = False
        self.parent.pinfo(f"{self.jName} recovered")

    @error_catcher
    def move_check_tmrCBK(self):
        if self.parent.recovering:
            self.movecheckTMR.cancel()
            return
        if self.angle is None:
            return
        if self.pastAngle is None:
            self.pastAngle = self.angle
            return

        youngCommand = self.parent.getNow() - self.last_com_time < Duration(
            seconds=WAIT_AFTER_COMMAND  # type: ignore
        )
        if np.isclose(self.angle, self.pastAngle, atol=MOVING_TOL):
            self.isMoving = False
        else:
            self.isMoving = True

        if self.command is None:
            isOnTarget = True
        else:
            isOnTarget = np.isclose(self.angle, self.command, atol=ON_TARGET_TOL)

        # self.parent.pinfo((self.isMoving, isOnTarget, youngCommand))
        if not self.isMoving and not isOnTarget and not youngCommand:
            self.stuck = True
            self.parent.pwarn(f"{self.jName} is stuck at {self.angle}")
            self.movecheckTMR.cancel()

        self.pastAngle = self.angle
        return

    @error_catcher
    def try_to_reachCBK(self, msg: Float64):
        if self.parent.recovering:
            return
        if self.movecheckTMR.is_canceled():
            self.movecheckTMR.reset()
        self.send_angle(msg.data)

    @error_catcher
    def further_tmrCBK(self):
        if self.angle is None:
            self.parent.pwarn(f"no angle readings on {self.reader_topic_name}")
            return

        if self.command is None:
            last_com = self.angle
        else:
            last_com = self.command
        self.send_angle(last_com + self.increment)

    @error_catcher
    def readCBK(self, msg: Float64) -> None:
        if self.angle is None:
            if self.offset_from_upper is None:
                dispoff = "UNKNOWN"
            else:
                dispoff = f"{self.offset_from_upper:.2f}"
            self.parent.pinfo(
                    f"{bcolors.OKCYAN}{self.jName} initialized{bcolors.ENDC}, current angle: {msg.data:.2f}, offset from upper limit: {dispoff}"
            )
            self.previous_ang = msg.data
        else:
            self.previous_ang = self.angle

        self.angle = msg.data

    @error_catcher
    def one_sec_tmrCBK(self):
        if self.angle is None:
            self.parent.pinfo(
                f"{self.reader_topic_name} listening but received nothing after 1s,"
                f" {bcolors.FAIL}might not be published{bcolors.ENDC}"
            )
        self.parent.destroy_timer(self.oneTMR)

    def send_angle(self, ang: float) -> None:
        if self.parent.recovering:
            return
        self.command = ang
        self.last_com_time = self.parent.getNow()
        if not self.movecheckTMR.is_canceled:
            self.movecheckTMR.cancel()
            self.movecheckTMR.reset()
        self.pub.publish(Float64(data=ang))

    def __del__(self):
        self.parent.destroy_timer(self.movecheckTMR)
        self.parent.destroy_timer(self.auto_recoverTMR)
        self.parent.destroy_client(self.recov_serv)
        self.parent.destroy_publisher(self.pub)
        self.parent.destroy_subscription(self.sub)
        self.parent.destroy_subscription(self.moveSub)


class LimitGoNode(EliaNode):
    def __init__(self):
        super().__init__("limit_go_node")  # type: ignore
        self.NAMESPACE = self.get_namespace()
        self.Alias = "LG"

        self.OFFSETS = csv_to_dict(CSV_PATH)
        self.pinfo(
            f"Offsets (zero position relative to upper limit) are defined as "
            f"{self.OFFSETS}"
        )
        # for key, value in OFFSETS.items():
        # update_csv(CSV_PATH, key, value)

        self.setAndBlockForNecessaryClients(["joint_alive"])

        self.jointDic: Dict[str, Joint] = {}  # reader topic name -> Joint obj

        self.scanTMR = self.create_timer(1, self.scan_topics)
        self.furtherTMR = self.create_timer(STEP_PERIOD, self.furtherTMRCBK)

        self.recovering = False

    @error_catcher
    def furtherTMRCBK(self):
        at_least_one_found = False
        if self.recovering:
            return
        for name, j in self.jointDic.items():
            if j.stuck is True:
                return
            if j.jName is None:
                continue
            if j.angle is None:
                continue
            if DIRECTION[j.jName] == 0:
                continue
            if j.upper_limit is not None:
                at_least_one_found = True
                continue
            self.pinfo(f"testing {name}")
            a = j.angle
            j.try_to_reachCBK(Float64(data=a + STEP_RAD * DIRECTION[j.jName] * FLIP))
            return
        self.pinfo("no joints to initialize")
        if at_least_one_found:
            self.destroy_timer(self.furtherTMR)
            self.all_go_to_default()

    def all_go_to_default(self):
        for name, joint in self.jointDic.items():
            joint.go_to_default()

    def add_couple(self, reader: str, setter: str):
        if reader in self.jointDic.keys():
            return
        self.jointDic[reader] = Joint(reader, setter, self)

    @error_catcher
    def scan_topics(self):
        self.scanTMR.destroy()
        topics = self.get_topic_names_and_types()
        valid_setters = [t for t in topics if self.isSetAngle(t)]
        valid_readers = [t for t in topics if self.isReadAngle(t)]
        for setter in valid_setters:
            partner = self.find_partner(setter, valid_readers)
            if partner is None:
                continue
            self.add_couple(partner[0], setter[0])

        valid_setters = [t[0] for t in topics if valid_setters]
        valid_readers = [t[0] for t in topics if valid_readers]
        for tracked_reader in list(self.jointDic.keys()):
            lost = False
            tracked_setter = self.jointDic[tracked_reader].setter_topic_name
            if not tracked_reader in valid_readers:
                lost = True
            if not tracked_setter in valid_setters:
                lost = True
            if lost:
                self.pwarn(f"{self.exctract_joint(tracked_setter)} lost")
                del self.jointDic[tracked_reader]

    @staticmethod
    def exctract_joint(setter_topic_name: str) -> Optional[str]:
        regMatch = re.search(r"ang_(.*?)_set", setter_topic_name)
        if regMatch:
            return regMatch.group(1)  # Extract and return the matched substring
        else:
            return None

    def find_partner(
        self,
        setter_topic: Tuple[str, List[str]],
        reader_topic_list: List[Tuple[str, List[str]]],
    ) -> Optional[Tuple[str, List[str]]]:
        jointName = self.exctract_joint(setter_topic[0])
        if jointName is None:
            return
        for potential in reader_topic_list:
            partnerMatch = re.search(f"read_{jointName}", potential[0])
            if partnerMatch:
                # self.pinfo(f"{setter_topic[0]}, {potential[0]}")
                return potential
        return

    def isSetAngle(self, topic: Tuple[str, List[str]]):
        tName = topic[0]
        tType = topic[1]
        return (
            ("_set" in tName) and ("ang_" in tName) and ("std_msgs/msg/Float64" in tType)
        )

    def isReadAngle(self, topic: Tuple[str, List[str]]):
        tName = topic[0]
        tType = topic[1]
        return ("read_" in tName) and ("std_msgs/msg/Float64" in tType)


def main(args=None):
    myMain(LimitGoNode)


if __name__ == "__main__":
    main()
