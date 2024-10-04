import matplotlib
from rclpy.clock import Duration, Time
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

matplotlib.use("Agg")  # fix for when there is no display

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
    loadAndSet_URDF,
    replace_incompatible_char_ros2,
    error_catcher,
    myMain,
    bcolors,
)

RECOVER_SLEEP = 3  # s
SAFETY_MARGIN = 0.1  # rad
ON_TARGET_TOL = 0.05  # rad
MOVING_TOL = 0.001  # rad
WAIT_AFTER_COMMAND = 0.5  # s

DIRECTION = {
    "leg3_joint1": -1,
    "leg3_joint2": -1,
    "leg3_joint3": 1,
    "leg3_joint4": 1,
    "leg3_joint5": 1,
    "leg3_joint6": 1,
    "leg3_steering_joint": -1,
}


class Joint:
    def __init__(
        self, reader_topic_name: str, setter_topic_name: str, parent: "LimitGoNode"
    ):
        self.reader_topic_name = reader_topic_name
        self.setter_topic_name = setter_topic_name
        self.parent = parent

        self.jName = self.parent.exctract_joint(self.setter_topic_name)
        self.pastAngle: Optional[float] = None
        self.angle: Optional[float] = None
        self.previous_ang: float = 0.0
        self.command: Optional[float] = None
        self.isMoving = False
        self.stuck = False
        self.increment = 0.01
        self.last_com_time: Time = self.parent.getNow()

        self.limit: Optional[float] = None

        self.recov_serv = self.parent.get_and_wait_Client(
            "/maxon/driver/recover", Trigger
        )
        self.sub = self.parent.create_subscription(
            Float64, self.reader_topic_name, self.read_cbk, 10
        )
        self.pub = self.parent.create_publisher(Float64, self.setter_topic_name, 10)
        self.moveSub = self.parent.create_subscription(
            Float64, f"test_{self.jName}", self.test_cbk, 10
        )

        self.oneTMR = self.parent.create_timer(1, self.oneSecAfter)
        self.movecheckTMR = self.parent.create_timer(0.1, self.move_check_tmrCBK)
        self.movecheckTMR.cancel()
        self.furtherTMR: Optional[Timer] = None
        self.auto_recoverTMR = self.parent.create_timer(1, self.auto_recover_tmrCBK)

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
        self.save_limit()
        direction = self.command - limit
        direction = direction / abs(direction)

        safe_target = limit - direction * SAFETY_MARGIN
        self.send_angle(safe_target)
        self.recover()

    def save_limit(self):
        self.limit = self.angle
        self.parent.pinfo(f"limit saved at {self.limit}")

    def recover(self):
        self.parent.pinfo(f"recovering to {self.command}")
        self.parent.recovering = True
        fut = self.recov_serv.call_async(Trigger.Request())
        fut.add_done_callback(self.recovered_cbk)

    def recovered_cbk(self, msg):
        self.parent.recovering = False
        self.stuck = False
        self.parent.sleep(RECOVER_SLEEP)
        self.parent.pinfo(f"{self.jName} recovered")

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

    def test_cbk(self, msg: Float64):
        if self.parent.recovering:
            return
        if self.movecheckTMR.is_canceled():
            self.movecheckTMR.reset()
        # self.furtherTMR = self.parent.create_timer(0.2, self.further_tmrCBK)
        # if self.parent.getNow() - self.last_com_time > Duration(seconds=0.5):
        self.send_angle(msg.data)

    def further_tmrCBK(self):
        if self.angle is None:
            self.parent.pwarn(f"no angle readings on {self.reader_topic_name}")
            return

        if self.command is None:
            last_com = self.angle
        else:
            last_com = self.command
        self.send_angle(last_com + self.increment)

    def read_cbk(self, msg: Float64) -> None:
        if self.angle is None:
            self.parent.pinfo(
                f"{self.reader_topic_name} initialized with 1st angle read: {msg.data}"
            )
            self.previous_ang = msg.data
        else:
            self.previous_ang = self.angle

        self.angle = msg.data

    def oneSecAfter(self):
        if self.angle is None:
            self.parent.pinfo(
                f"{self.reader_topic_name} listening but received nothing after 1s, might not be published"
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


class LimitGoNode(EliaNode):
    def __init__(self):
        super().__init__("joint_node")  # type: ignore
        self.NAMESPACE = self.get_namespace()
        self.Alias = "LG"

        self.setAndBlockForNecessaryClients(["joint_alive"])

        self.jointDic: Dict[str, Joint] = {}

        self.scanTMR = self.create_timer(1, self.scan_topics)
        self.auto_limitTMR = self.create_timer(1, self.further)

        self.recovering = False

    def further(self):
        if self.recovering:
            return
        for name, j in self.jointDic.items():
            if j.jName is None:
                continue
            if j.stuck is True:
                return
            if j.angle is None:
                continue
            # if j.command is None:
            #     continue
            if j.limit is not None:
                continue
            self.pinfo(f"testing {name}")
            j.test_cbk(Float64(data=j.angle + 0.2 * DIRECTION[j.jName]))
            return
        self.pinfo("no joints left to initialize")

    def add_couple(self, reader: str, setter: str):
        if reader in self.jointDic.keys():
            return
        self.jointDic[reader] = Joint(reader, setter, self)

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
