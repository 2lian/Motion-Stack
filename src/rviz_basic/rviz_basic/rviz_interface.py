from pprint import pprint
import matplotlib
from spatialmath.base import delta2tr

matplotlib.use("Agg")  # fix for when there is no display

import time
import traceback
from typing import Dict, List, Optional

# from dataclasses import dataclass
import dataclasses

import numpy as np
from numpy.core.multiarray import dtype
from numpy.typing import NDArray
import quaternion as qt
from rclpy.executors import ExternalShutdownException
from scipy.spatial import geometric_slerp
import rclpy
from rclpy.node import (
    Node,
    ReentrantCallbackGroup,
    MutuallyExclusiveCallbackGroup,
    Service,
    Timer,
    Union,
)
import tf2_ros

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Header
from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped, Transform

from easy_robot_control.EliaNode import (
    loadAndSet_URDF,
    replace_incompatible_char_ros2,
    error_catcher,
    myMain,
    EliaNode,
    rosTime2Float,
)
from rclpy.clock import Clock, ClockType
from rclpy.time import Duration, Time

TIME_TO_ECO_MODE: float = 1  # seconds
ECO_MODE_PERIOD: float = 1  # seconds


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
        super().__init__("joint_state_rviz")  # type: ignore

        self.NAMESPACE = self.get_namespace()
        self.Alias = "RV"

        self.setAndBlockForNecessaryNodes(["rviz", "rviz2"])

        self.get_logger().warning(f"""Rviz connected :)""")

        self.jsDic: Dict[str, JState] = {}

        # V Params V
        #   \  /   #
        #    \/    #
        self.declare_parameter("mirror_angles", True)
        self.MIRROR_ANGLES: bool = (
            self.get_parameter("mirror_angles").get_parameter_value().bool_value
        )
        if self.MIRROR_ANGLES:
            self.pwarn("! WARNING ! : Rviz is used as angle feedback\
                    disable mirror_angle setting if you are working with the real robot\
                    or simu")
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
        self.joint_feedback_pub = self.create_publisher(JointState, "joint_state", 10)
        # self.body_pose_pub = self.create_publisher(
        # TFMessage,
        # '/BODY', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
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
        self.upwardTMR: Timer = self.create_timer(1 / 5, self.send_upward)
        self.displayTRM: Timer = self.create_timer(1, self.refreshRviz)
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

            if areAngle:
                state.position = jsMSG.position[index]
            if areVelocity:
                state.velocity = jsMSG.velocity[index]
            if areEffort:
                state.effort = jsMSG.effort[index]

            self.updateJS(state)

        self.refreshRviz(jsMSG.name)

    @error_catcher
    def updateJS(self, state: JState) -> None:
        assert state.name is not None
        alreadyTracked: List[str] = list(self.jsDic.keys())
        isNew = state.name not in alreadyTracked
        if isNew:
            if self.MIRROR_ANGLES:
                self.jsDic[state.name] = JState(
                    state.name, 0.0, None, None, state.time
                )
            else:
                self.jsDic[state.name] = JState(state.name, None, None, None, state.time)

        previous: JState = dataclasses.replace(self.jsDic[state.name])
        next: JState = dataclasses.replace(self.jsDic[state.name])

        if state.position is None:
            next = self.integrateSpeed(previous, state.time)
            next.velocity = state.velocity
        else:
            next.position = state.position
        next.time = state.time

        self.jsDic[state.name] = next

    @error_catcher
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
        new.position %= 2 * np.pi  # type: ignore
        # self.pwarn(f"speed: {new.velocity}")
        # self.pwarn(f"dT: {deltaT}")
        # self.pwarn(f"pos: {new.position}")
        # self.pwarn(f"\n")

        return new

    @error_catcher
    def refreshRviz(
        self, names: Optional[List[str]] = None,
    ) -> None:
        out = JointState()
        alreadyTracked: List[str] = list(self.jsDic.keys())
        nameList: List[str]
        if names is None:
            nameList = alreadyTracked
        else:
            if len(names) == 0:
                return
            # nameList = list(set(names + self.getSpeedControledNames()))
            nameList = alreadyTracked # always update all

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
            posout.append(state.position)

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

    @error_catcher
    def getSpeedControledNames(self) -> List[str]:
        return [js.name for js in self.jsDic.values() if self.jsIsMoving(js)]

    @error_catcher
    def refreshSpeed(self):
        nameList = self.getSpeedControledNames()
        self.refreshRviz(nameList)


def main(args=None):
    myMain(RVizInterfaceNode)


if __name__ == "__main__":
    main()
