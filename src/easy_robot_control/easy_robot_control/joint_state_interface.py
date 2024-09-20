import matplotlib

matplotlib.use("Agg")  # fix for when there is no display

import time
import traceback
from typing import Dict, List, Optional
from dataclasses import dataclass

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
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped, Transform

from EliaNode import EliaNode
from EliaNode import (
    loadAndSet_URDF,
    replace_incompatible_char_ros2,
    error_catcher,
    myMain,
    bcolors,
)
from rclpy.clock import Clock, ClockType
from rclpy.time import Duration, Time

TIME_TO_ECO_MODE: float = 1  # seconds
ECO_MODE_PERIOD: float = 1  # seconds


@dataclass
class JState:
    jointName: Optional[str]
    position: Optional[float] = None
    velocity: Optional[float] = None
    effort: Optional[float] = None


class MiniJointHandler:
    def __init__(self, name: str, index: int, parent_node):
        self.name = name
        self.corrected_name = replace_incompatible_char_ros2(name)
        self.index = index
        self.parent_node = parent_node

        self.stateCommand = JState(jointName=self.name)
        self.stateRead = JState(jointName=self.name)
        self.angle_updated = False
        self.speed_updated = False
        self.effort_updated = False
        self.clock: Clock = self.parent_node.get_clock()
        self.last_speed2angle_stamp: Time = self.clock.now()

        self.parent_node.create_subscription(
            Float64,
            f"ang_{self.corrected_name}_set",
            self.set_angle_cbk,
            10,
            callback_group=self.parent_node.cbk_legs,
        )
        self.parent_node.create_subscription(
            Float64,
            f"spe_{self.corrected_name}_set",
            self.set_speed_cbk,
            10,
            callback_group=self.parent_node.cbk_legs,
        )
        self.parent_node.create_subscription(
            Float64,
            f"eff_{self.corrected_name}_set",
            self.set_effort_cbk,
            10,
            callback_group=self.parent_node.cbk_legs,
        )
        self.pub_back_to_ros2_structure = self.parent_node.create_publisher(
            Float64,
            f"read_{self.corrected_name}",
            10,
        )

    @error_catcher
    def resetAnglesAtZero(self):
        self.set_angle_cbk(0)

    @error_catcher
    def setJSRead(self, js: JState):
        assert js.jointName == self.name
        self.stateRead = js
        self.publish_back_up_to_ros2()

    @error_catcher
    def set_angle_cbk(self, msg: Union[Float64, float]):
        if isinstance(msg, Float64):
            angle = msg.data
        else:
            angle = msg

        assert isinstance(angle, float)

        self.stateCommand.position = angle
        self.angle_updated = True

        self.parent_node.request_refresh()

    @error_catcher
    def set_speed_cbk(self, msg: Float64):
        speed = msg.data

        self.stateCommand.velocity = speed
        self.speed_updated = True

        self.parent_node.request_refresh()
        return

    @error_catcher
    def set_effort_cbk(self, msg: Float64):
        effort = msg.data

        self.stateCommand.effort = effort
        self.effort_updated = True

        self.parent_node.request_refresh()
        return

    # @error_catcher
    # def update_angle_from_speed(self, time_stamp: Optional[Time] = None) -> None:
    #     now: Time
    #     if time_stamp is None:
    #         now: Time = self.clock.now()
    #     else:
    #         now: Time = time_stamp
    #
    #     noSpeed = self.stateCommand.velocity in [None, 0.0]
    #     if noSpeed:
    #         self.last_speed2angle_stamp = now
    #         return
    #
    #     delta: Duration = self.last_speed2angle_stamp - now
    #
    #     if self.stateCommand.position is None:
    #         self.stateCommand.position = 0.0
    #
    #     self.stateCommand.position = (
    #         self.stateCommand.position + self.stateCommand.velocity * delta.nanoseconds * 10e-9
    #     )
    #     self.last_speed2angle_stamp = now

    @error_catcher
    def get_stateCommand(
        self, force_position: bool = False, reset: bool = True
    ) -> JState:
        # if force_position:
        #     if self.stateCommand.position is None:
        #         self.stateCommand.position = 0
        #     self.update_angle_from_speed()
        #     self.angle_updated = True

        state_out = JState(jointName=self.stateCommand.jointName)
        if self.angle_updated:
            state_out.position = self.stateCommand.position
        if self.speed_updated:
            state_out.velocity = self.stateCommand.velocity
        if self.effort_updated:
            state_out.effort = self.stateCommand.effort

        if reset:
            self.angle_updated = False
            self.speed_updated = False
            self.effort_updated = False
        return state_out

    @error_catcher
    def publish_back_up_to_ros2(self, angle: Optional[float] = None) -> None:
        angle_out: float

        if angle is None:
            # self.update_angle_from_speed()
            if self.stateCommand.position is None:
                angle_out = 0
            else:
                angle_out = self.stateCommand.position
        else:
            angle_out = angle

        msg = Float64()
        msg.data = float(angle_out)
        self.pub_back_to_ros2_structure.publish(msg)


class RVizInterfaceNode(EliaNode):

    def __init__(self):
        # rclpy.init()
        super().__init__("joint_node")  # type: ignore

        self.NAMESPACE = self.get_namespace()
        self.Alias = "JS"

        self.current_body_xyz: NDArray = np.array([0, 0, 0.200], dtype=float)
        self.current_body_quat: qt.quaternion = qt.one
        self.body_xyz_queue = np.zeros((0, 3), dtype=float)
        self.body_quat_queue = qt.from_float_array(np.zeros((0, 4), dtype=float))

        self.setAndBlockForNecessaryClients(["rviz_interface_alive"])

        self.pwarn(f"""{bcolors.OKBLUE}Interface connected to motors :){bcolors.ENDC}""")

        # V Params V
        #   \  /   #
        #    \/    #
        self.declare_parameter("std_movement_time", float(0.5))
        self.MOVEMENT_TIME = (
            self.get_parameter("std_movement_time").get_parameter_value().double_value
        )

        self.declare_parameter("frame_prefix", "")
        self.FRAME_PREFIX = (
            self.get_parameter("frame_prefix").get_parameter_value().string_value
        )

        self.declare_parameter("mvmt_update_rate", float(30))
        self.MVMT_UPDATE_RATE = (
            self.get_parameter("mvmt_update_rate").get_parameter_value().double_value
        )

        self.declare_parameter("always_write_position", False)
        self.ALWAYS_WRITE_POSITION = (
            self.get_parameter("always_write_position").get_parameter_value().bool_value
        )

        self.declare_parameter("start_coord", [0.0, 0.0, 0.0])
        self.START_COORD = np.array(
            self.get_parameter("start_coord").get_parameter_value().double_array_value,
            dtype=float,
        )
        self.current_body_xyz: NDArray = self.START_COORD

        self.declare_parameter("mirror_angles", False)
        self.MIRROR_ANGLES: bool = (
            self.get_parameter("mirror_angles").get_parameter_value().bool_value
        )
        self.declare_parameter("urdf_path", str())
        self.urdf_path = (
            self.get_parameter("urdf_path").get_parameter_value().string_value
        )
        #    /\    #
        #   /  \   #
        # ^ Params ^

        (
            self.model,
            self.ETchain,
            self.joint_names,
            self.joints_objects,
            self.last_link,
        ) = loadAndSet_URDF(self.urdf_path)
        self.baselinkName = self.model.base_link.name

        self.pinfo(f"Joints controled: {self.joint_names}")
        self.pinfo(f"Detected base_link: {self.baselinkName}")

        # V Subscriber V
        #   \  /   #
        #    \/    #
        self.cbk_legs = MutuallyExclusiveCallbackGroup()
        self.jointHandlerL: List[MiniJointHandler] = []
        for index, name in enumerate(self.joint_names):
            holder = MiniJointHandler(name, index, self)
            self.jointHandlerL.append(holder)
        self.jointHandlerDic = dict(zip(self.joint_names, self.jointHandlerL))
        # for leg in range(4):
        #     for joint in range(3):
        #         holder = CallbackHolder(leg, joint, self, self.joint_state)
        #         self.cbk_holder_list.append(holder)

        self.body_pose_sub = self.create_subscription(
            Transform,
            "robot_body",
            self.robot_body_pose_cbk,
            10,
            callback_group=self.cbk_legs,
        )

        self.smooth_body_pose_sub = self.create_subscription(
            Transform,
            "smooth_body_rviz",
            self.smooth_body_trans,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.smooth_body_pose_sub = self.create_subscription(
            JointState,
            "joint_state",
            self.jsRecieved,
            10,
        )
        #    /\    #
        #   /  \   #
        # ^ Subscriber ^

        # V Publisher V
        #   \  /   #
        #    \/    #
        self.joint_state_pub = self.create_publisher(JointState, "joint_commands", 10)
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
        self.refresh_timer = self.create_timer(1 / self.MVMT_UPDATE_RATE, self.__refresh)
        self.go_in_eco = self.create_timer(TIME_TO_ECO_MODE, self.eco_mode)
        self.go_in_eco.cancel()
        self.eco_timer = self.create_timer(
            ECO_MODE_PERIOD, lambda: (self.__refresh(), self.publish_all_angle_upstream())
        )
        self.eco_timer.cancel()
        self.angle_upstream_tmr = self.create_timer(
            1 / self.MVMT_UPDATE_RATE, self.publish_all_angle_upstream
        )
        self.firstSpin: Timer = self.create_timer(1 / 100, self.firstSpinCBK)
        #    /\    #
        #   /  \   #
        # ^ Timer ^

    @error_catcher
    def firstSpinCBK(self):
        self.iAmAlive = self.create_service(Empty, "joint_alive", lambda i, o: o)
        self.destroy_timer(self.firstSpin)

        # send empty command to initialize (notabily Rviz interface)
        empty = JointState(name=self.joint_names)
        empty.header.stamp = self.getNow().to_msg()
        self.joint_state_pub.publish(empty)

        # we should not start at zero when using real robot
        # for jointMiniNode in self.cbk_holder_list:
        # jointMiniNode.resetAnglesAtZero()

    @error_catcher
    def jsRecieved(self, jsReading: JointState) -> None:
        jointsHandled: List[str] = list(self.jointHandlerDic.keys())
        areAngle = len(jsReading.position) > 0
        areVelocity = len(jsReading.velocity) > 0
        areEffort = len(jsReading.effort) > 0

        nothingInside = not (areAngle or areVelocity or areEffort)
        if nothingInside:
            return

        for index, name in enumerate(jsReading.name):
            isResponsable = name in jointsHandled
            if not isResponsable:
                continue  # skips
            handler: MiniJointHandler = self.jointHandlerDic[name]
            js = JState(jointName=name)

            if areAngle:
                js.position = jsReading.position[index]
            if areVelocity:
                js.velocity = jsReading.velocity[index]
            if areEffort:
                js.effort = jsReading.effort[index]

            handler.setJSRead(js)

    @error_catcher
    def __pull_states(self, force_position: bool = False) -> List[JState]:
        allStates: List[JState] = []

        for jointMiniNode in self.jointHandlerL:
            state: JState = jointMiniNode.get_stateCommand(force_position)
            allEmpty: bool = (
                (state.velocity is None)
                and (state.position is None)
                and (state.effort is None)
            )
            if allEmpty:
                continue
            allStates.append(state)

        return allStates

    @staticmethod
    def __stateOrderinator3000(allStates: List[JState]) -> List[JointState]:
        # out = [JointState() for i in range(2**3)]
        outDic: Dict[int, JointState] = {}
        for state in allStates:
            idx = 0
            if state.position is not None:
                idx += 2**0
            if state.velocity is not None:
                idx += 2**1
            if state.effort is not None:
                idx += 2**2
            # workingJS = out[idx]

            workingJS: JointState
            if not idx in outDic.keys():
                outDic[idx] = JointState()
                workingJS = outDic[idx]
                workingJS.name = []
                if state.position is not None:
                    workingJS.position = []
                if state.velocity is not None:
                    workingJS.velocity = []
                if state.effort is not None:
                    workingJS.effort = []
            else:
                workingJS = outDic[idx]

            workingJS.name.append(state.jointName)
            if state.position is not None:
                workingJS.position.append(state.position)
            if state.velocity is not None:
                workingJS.velocity.append(state.velocity)
            if state.effort is not None:
                workingJS.effort.append(state.effort)

        withoutNone: List[JointState] = list(outDic.values())
        return withoutNone

    @error_catcher
    def __refresh(self, time_stamp: Optional[Time] = None):
        if time_stamp is None:
            now: Time = self.get_clock().now()
        else:
            now: Time = time_stamp
        time_now_stamp = now.to_msg()

        self.__pop_and_load_body()
        xyz = self.current_body_xyz.copy()
        rot = self.current_body_quat.copy()
        msgTF = self.np2tf(xyz, rot)

        body_transform = TransformStamped()
        body_transform.header.stamp = time_now_stamp
        body_transform.header.frame_id = "world"
        body_transform.child_frame_id = f"{self.FRAME_PREFIX}{self.baselinkName}"
        body_transform.transform = msgTF

        forcePosUpdate = self.ALWAYS_WRITE_POSITION
        allStates: List[JState] = self.__pull_states(forcePosUpdate)
        ordinatedJointStates: List[JointState] = self.__stateOrderinator3000(allStates)
        for jsMSG in ordinatedJointStates:
            jsMSG.header.stamp = time_now_stamp
            self.joint_state_pub.publish(jsMSG)
            if self.MIRROR_ANGLES:
                self.jsRecieved(jsMSG)

        self.tf_broadcaster.sendTransform(body_transform)

        if self.go_in_eco.is_canceled() and self.eco_timer.is_canceled():
            self.go_in_eco.reset()
        return

    @error_catcher
    def __pop_and_load_body(self):
        empty = self.body_xyz_queue.shape[0] <= 0
        if not empty:
            self.current_body_xyz = self.body_xyz_queue[0, :]
            self.current_body_quat = self.body_quat_queue[0]
            self.body_xyz_queue = np.delete(self.body_xyz_queue, 0, axis=0)
            self.body_quat_queue = np.delete(self.body_quat_queue, 0, axis=0)

    @error_catcher
    def robot_body_pose_cbk(self, msg):
        tra, quat = self.tf2np(msg)
        self.current_body_xyz = tra
        self.current_body_quat = quat
        self.request_refresh()

    @error_catcher
    def smoother(self, x: NDArray) -> NDArray:
        """smoothes the interval [0, 1] to have a soft start and end
        (derivative is zero)
        """
        # x = (1 - np.cos(x * np.pi)) / 2
        # x = (1 - np.cos(x * np.pi)) / 2
        return x

    @error_catcher
    def smooth_body_trans(self, request: Transform):
        # return
        tra, quat = self.tf2np(request)
        final_coord = self.current_body_xyz + tra / 1000
        final_quat = self.current_body_quat * quat

        samples = int(self.MOVEMENT_TIME * self.MVMT_UPDATE_RATE)
        start_coord = self.current_body_xyz.copy()
        start_quat = self.current_body_quat.copy()

        x = np.linspace(0, 1, num=samples)  # x: [0->1]
        x = self.smoother(x)

        quaternion_interpolation = geometric_slerp(
            start=qt.as_float_array(start_quat), end=qt.as_float_array(final_quat), t=x
        )
        quaternion_interpolation = qt.as_quat_array(quaternion_interpolation)

        x = np.tile(x, (3, 1)).transpose()
        coord_interpolation = final_coord * x + start_coord * (1 - x)

        self.body_xyz_queue = coord_interpolation
        self.body_quat_queue = quaternion_interpolation
        self.request_refresh()
        return

    @error_catcher
    def request_refresh(self):
        self.go_in_eco.reset()
        if self.refresh_timer.is_canceled():
            self.refresh_timer.reset()
            self.angle_upstream_tmr.reset()
            self.eco_timer.cancel()

    @error_catcher
    def eco_mode(self):
        if self.eco_timer.is_canceled() and not self.ALWAYS_WRITE_POSITION:
            # self.pwarn("eco mode")
            self.refresh_timer.cancel()
            self.angle_upstream_tmr.cancel()
            self.eco_timer.reset()

    @error_catcher
    def publish_all_angle_upstream(self):
        # if not self.MIRROR_ANGLES:
        # return

        # for holder in self.jointHandlerL:
        # holder.publish_back_up_to_ros2()
        ...


def main(args=None):
    myMain(RVizInterfaceNode)


if __name__ == "__main__":
    main()
