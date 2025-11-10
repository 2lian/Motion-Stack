import matplotlib
from motion_stack.core.utils.joint_state import JState, Time
from motion_stack.ros2.utils.conversion import ros_to_time
from motion_stack.ros2.utils.joint_state import stateOrderinator3000

matplotlib.use("Agg")  # fix for when there is no display

# from dataclasses import dataclass
import dataclasses
from typing import Dict, List, Optional, Set

import numpy as np
from easy_robot_control.EliaNode import EliaNode, error_catcher, myMain, rosTime2Float
from motion_stack.ros2.communication import lvl1 as comms
from rclpy.node import Service, Timer
from rclpy.time import Time as RosTime
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

MAX_SPEED = 0.30  # rad/s


class RVizInterfaceNode(EliaNode):
    def __init__(self):
        super().__init__("rviz_interface")  # type: ignore

        self.NAMESPACE = self.get_namespace()
        self.Alias = "RV"

        self.jsDic: Dict[str, JState] = {}
        self._displayed_to_usr = set()

        # V Params V
        #   \  /   #
        #    \/    #
        self.declare_parameter("refresh_rate", 30.0)
        self.REFRESH_RATE: float = (
            self.get_parameter("refresh_rate").get_parameter_value().double_value
        )
        self.declare_parameter("mirror_angles", False)
        self.pwarn(
            "! WARNING ! : Rviz is used as angle feedback \n"
            f"DO NOT USE THIS NODE WHILE THE REAL ROBOT IS RUNNING"
        )
        #    /\    #
        #   /  \   #
        # ^ Params ^

        # V Subscriber V
        #   \  /   #
        #    \/    #
        self.joint_state_sub = self.create_subscription(
            comms.output.motor_command.type,
            comms.output.motor_command.name,
            self.jsRecieved,
            qos_profile=comms.output.motor_command.qos,
        )
        #    /\    #
        #   /  \   #
        # ^ Subscriber ^

        # V Publisher V
        #   \  /   #
        #    \/    #
        self.joint_state_pub = self.create_publisher(JointState, "rviz_commands", 10)
        self.joint_feedback_pub = self.create_publisher(
            comms.input.motor_sensor.type,
            comms.input.motor_sensor.name,
            qos_profile=comms.input.motor_sensor.qos,
        )
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
        # self.upwardTMR: Timer = self.create_timer(self.REFRESH_RATE, self.send_upward)
        self.displayTMR: Timer = self.create_timer(
            1, self.display_new_joints
        )
        self.updateTRM: Timer = self.create_timer(
            1 / self.REFRESH_RATE, self.refreshRviz
        )
        #    /\    #
        #   /  \   #
        # ^ Timer ^

    @error_catcher
    def display_new_joints(self):
        new = set(self.jsDic.keys()) - self._displayed_to_usr
        if len(new) == 0:
            return
        self.get_logger().info(f"Simulating joints: {new}")
        self._displayed_to_usr |= new

    @error_catcher
    def jsRecieved(self, jsMSG: JointState) -> None:
        stamp = RosTime.from_msg(jsMSG.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.getNow()
        areName = len(jsMSG.name) > 0
        areAngle = len(jsMSG.position) > 0
        areVelocity = len(jsMSG.velocity) > 0
        areEffort = len(jsMSG.effort) > 0

        nothingInside = not (areAngle or areVelocity or areEffort)
        if not areName:
            # if nothingInside or (not areName):
            return

        for index, name in enumerate(jsMSG.name):
            state = JState(name=name, time=Time(sec=ros_to_time(stamp)))

            if areAngle and not areVelocity:
                state.position = jsMSG.position[index]
            if areVelocity:
                # also applies max speed
                state.velocity = np.clip(jsMSG.velocity[index], -MAX_SPEED, MAX_SPEED)
            if areEffort:
                state.effort = jsMSG.effort[index]

            self.updateJS(state)

        # self.refreshRviz(jsMSG.name)

    @error_catcher
    def updateJS(self, state: JState) -> None:
        assert state.name is not None
        alreadyTracked: Set[str] = set(self.jsDic.keys())
        isNew = state.name not in alreadyTracked
        if isNew:
            # self.get_logger().info(f"New simulated joint: {state.name}")
            self.jsDic[state.name] = JState(state.name, state.time, 0.0, None, None)

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
        deltaT = (updateTime - state.time).sec()
        deltaP = state.velocity * deltaT
        new.position += deltaP  # type: ignore
        # new.position %= 2 * np.pi  # type: ignore

        return new

    @error_catcher
    def refreshRviz(
        self,
        names: Optional[List[str]] = None,
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

        out.name = nameout
        out.position = posout
        out.header.stamp = now.to_msg()
        self.joint_state_pub.publish(out)
        self.send_upward(nameList)

    @error_catcher
    def send_upward(self, names: Optional[List[str]] = None) -> None:

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
        states: List[JState] = []

        now = self.getNow()
        for name in nameList:
            isNew = name not in alreadyTracked
            if isNew:
                self.pwarn("update asked for unknown joint")
                continue
            state = self.jsDic[name]
            state = self.integrateSpeed(state, now)
            states.append(state)

        msgs = stateOrderinator3000(states)
        stmp = self.getNow().to_msg()
        for msg in msgs:
            msg.header.stamp = stmp
            self.joint_feedback_pub.publish(msg)

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
