import matplotlib

matplotlib.use("Agg")  # fix for when there is no display

from typing import Dict, List, Optional

# from dataclasses import dataclass
import dataclasses

import numpy as np
from numpy.typing import NDArray
from rclpy.node import (
    ReentrantCallbackGroup,
    MutuallyExclusiveCallbackGroup,
    Service,
    Timer,
)

from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from easy_robot_control.EliaNode import (
    loadAndSet_URDF,
    replace_incompatible_char_ros2,
    error_catcher,
    myMain,
    EliaNode,
    rosTime2Float,
)
from rclpy.time import Duration, Time

MAX_SPEED = 0.1  # rad/s


@dataclasses.dataclass
class JState:
    name: str
    position: Optional[float] = None
    velocity: Optional[float] = None
    effort: Optional[float] = None
    time: Optional[Time] = None


class RVizInterfaceNode(EliaNode):
    def __init__(self):
        # rclpy.init()
        super().__init__("rviz_interface")  # type: ignore

        self.NAMESPACE = self.get_namespace()
        self.Alias = "RV"

        self.setAndBlockForNecessaryNodes(["rviz", "rviz2"])

        self.jsDic: Dict[str, JState] = {}

        # V Params V
        #   \  /   #
        #    \/    #
        self.declare_parameter("refresh_rate", 30.0)
        self.REFRESH_RATE: float = (
            self.get_parameter("refresh_rate").get_parameter_value().double_value
        )
        self.declare_parameter("mirror_angles", False)
        self.MIRROR_ANGLES: bool = (
            self.get_parameter("mirror_angles").get_parameter_value().bool_value
        )
        if self.MIRROR_ANGLES:
            self.pwarn(
                "! WARNING ! : Rviz is used as angle feedback "
                f"disable mirror_angle setting if you are working "
                f"with the real robot or simu"
            )
        #    /\    #
        #   /  \   #
        # ^ Params ^

        # V Subscriber V
        #   \  /   #
        #    \/    #
        self.joint_state_sub = self.create_subscription(
            JointState, "joint_commands", self.jsRecieved, 10
        )
        #    /\    #
        #   /  \   #
        # ^ Subscriber ^

        # V Publisher V
        #   \  /   #
        #    \/    #
        self.joint_state_pub = self.create_publisher(JointState, "rviz_commands", 10)
        self.joint_feedback_pub = self.create_publisher(JointState, "joint_states", 10)
        # self.body_pose_pub = self.create_publisher(
        # TFMessage,
        # '/BODY', 10)
        #    /\    #
        #   /  \   #
        # ^ Publisher ^

        # V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive: Optional[Service] = None
        #    /\    #
        #   /  \   #
        # ^ Service ^

        # V Timer V
        #   \  /   #
        #    \/    #
        self.firstSpin: Timer = self.create_timer(1 / 100, self.firstSpinCBK)
        # self.upwardTMR: Timer = self.create_timer(self.REFRESH_RATE, self.send_upward)
        self.displayTRM: Timer = self.create_timer(
            1 / self.REFRESH_RATE, self.refreshRviz
        )
        #    /\    #
        #   /  \   #
        # ^ Timer ^

    @error_catcher
    def firstSpinCBK(self):
        self.iAmAlive = self.create_service(Empty, "rviz_interface_alive", lambda i, o: o)
        self.destroy_timer(self.firstSpin)

    @error_catcher
    def jsRecieved(self, jsMSG: JointState) -> None:
        stamp = Time.from_msg(jsMSG.header.stamp)
        areName = len(jsMSG.name) > 0
        areAngle = len(jsMSG.position) > 0
        areVelocity = len(jsMSG.velocity) > 0
        areEffort = len(jsMSG.effort) > 0

        nothingInside = not (areAngle or areVelocity or areEffort)
        if not areName:
            # if nothingInside or (not areName):
            return

        for index, name in enumerate(jsMSG.name):
            state = JState(name=name, time=stamp)

            if areAngle and not areVelocity:
                state.position = jsMSG.position[index]
            if areVelocity:
                # also applies max speed
                state.velocity = np.clip(jsMSG.velocity[index], -MAX_SPEED, MAX_SPEED)
            if areEffort:
                state.effort = jsMSG.effort[index]

            self.updateJS(state)

        # self.refreshRviz(jsMSG.name)

    def updateJS(self, state: JState) -> None:
        assert state.name is not None
        alreadyTracked: List[str] = list(self.jsDic.keys())
        isNew = state.name not in alreadyTracked
        if isNew:
            if self.MIRROR_ANGLES:
                self.jsDic[state.name] = JState(state.name, 0.0, None, None, state.time)
            else:
                self.jsDic[state.name] = JState(state.name, None, None, None, state.time)

        previous: JState = dataclasses.replace(self.jsDic[state.name])
        next: JState = dataclasses.replace(self.jsDic[state.name])

        next = self.integrateSpeed(previous, state.time)
        if state.position is not None:
            next.position = state.position
        next.velocity = state.velocity
        next.time = state.time

        self.jsDic[state.name] = next

    def integrateSpeed(self, state: JState, updateTime: Optional[Time]) -> JState:
        assert updateTime is not None
        new: JState = dataclasses.replace(state)
        if state.position is None:
            new.position = float(0)

        if state.velocity is None:
            return new
        if state.velocity == 0:
            return new

        new.time = updateTime
        deltaT = rosTime2Float(updateTime - state.time)
        deltaP = state.velocity * deltaT
        new.position += deltaP  # type: ignore
        # new.position %= 2 * np.pi  # type: ignore
        # self.pwarn(f"speed: {new.velocity}")
        # self.pwarn(f"dT: {deltaT}")
        # self.pwarn(f"pos: {new.position}")
        # self.pwarn(f"\n")

        return new

    @error_catcher
    def refreshRviz(
        self,
        names: Optional[List[str]] = None,
    ) -> None:
        # self.pwarn("hey")
        out = JointState()
        alreadyTracked: List[str] = list(self.jsDic.keys())
        nameList: List[str]
        if names is None:
            nameList = alreadyTracked
        else:
            if len(names) == 0:
                return
            # nameList = list(set(names + self.getSpeedControledNames()))
            nameList = alreadyTracked  # always update all

        nameout = []
        posout = []
        now = self.getNow()

        for name in nameList:
            isNew = name not in alreadyTracked
            if isNew:
                self.pwarn("update asked for unknown joint")
                continue
            state = self.jsDic[name]
            state = self.integrateSpeed(state, now)
            nameout.append(state.name)
            posout.append(state.position % (2 * np.pi))

        # self.pwarn(nameout)
        # self.pwarn(posout)
        out.name = nameout
        out.position = posout
        out.header.stamp = now.to_msg()
        self.joint_state_pub.publish(out)
        self.send_upward(nameList)

    @error_catcher
    def send_upward(self, names: Optional[List[str]] = None) -> None:
        if not self.MIRROR_ANGLES:
            return

        out = JointState()
        alreadyTracked: List[str] = list(self.jsDic.keys())
        nameList: List[str]
        if names is None:
            nameList = alreadyTracked
        else:
            nameList = list(set(names + self.getSpeedControledNames()))
            # nameList = alreadyTracked

        nameout = []
        posout = []

        for name in nameList:
            isNew = name not in alreadyTracked
            if isNew:
                self.pwarn("update asked for unknown joint")
                continue
            state = self.jsDic[name]
            nameout.append(state.name)
            posout.append(state.position)

        out.name = nameout
        out.position = posout
        out.header.stamp = self.getNow().to_msg()
        self.joint_feedback_pub.publish(out)

    @staticmethod
    def jsIsMoving(js: JState) -> bool:
        if js.velocity is None:
            return False
        if js.velocity == 0:  # can do better but who cares
            return False

        return True

    def getSpeedControledNames(self) -> List[str]:
        return [js.name for js in self.jsDic.values() if self.jsIsMoving(js)]


def main(args=None):
    myMain(RVizInterfaceNode)


if __name__ == "__main__":
    main()
