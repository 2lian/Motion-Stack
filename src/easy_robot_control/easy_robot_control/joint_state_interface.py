import matplotlib

matplotlib.use("Agg")  # fix for when there is no display
import sys
import pytest

# from pytest import ExitCode
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass

import numpy as np
from numpy.core.multiarray import dtype
from numpy.typing import NDArray
import quaternion as qt
from rclpy.executors import ExternalShutdownException
from scipy.spatial import geometric_slerp
import rclpy
from rclpy.node import (
    Publisher,
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

from EliaNode import EliaNode, Joint, list_cyanize, rosTime2Float
from EliaNode import (
    loadAndSet_URDF,
    replace_incompatible_char_ros2,
    error_catcher,
    myMain,
    bcolors,
    get_src_folder,
)
from rclpy.clock import Clock
from rclpy.time import Duration, Time

import python_package_include.pure_remap

from ament_index_python.packages import get_package_share_directory
import os
import importlib.util

rem_default = python_package_include.pure_remap
DISABLE_AUTO_RELOAD = False  # s
RELOAD_MODULE_DUR = 1  # s
P_GAIN = 3.5
D_GAIN = 0.00005
INIT_AT_ZERO = False  # dangerous

EXIT_CODE_TEST = {
    0: "OK",
    1: "TESTS_FAILED",
    2: "INTERRUPTED",
    3: "INTERNAL_ERROR",
    4: "USAGE_ERROR",
    5: "NO_TESTS_COLLECTED",
}
TIME_TO_ECO_MODE: float = 1  # seconds
ECO_MODE_PERIOD: float = 1  # seconds


def import_module_from_path(module_name, file_path):
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module  # Add the module to sys.modules

    # Load the module
    spec.loader.exec_module(module)

    return module


@dataclass
class JState:
    name: Optional[str]
    position: Optional[float] = None
    velocity: Optional[float] = None
    effort: Optional[float] = None
    time: Optional[Time] = None


class MiniJointHandler:
    def __init__(
        self,
        name: str,
        index: int,
        parent_node: "JointNode",
        joint_object: Joint,
        IGNORE_LIM: bool = False,
        MARGIN: float = 0.0,
    ):
        self.name = name
        self.corrected_name = replace_incompatible_char_ros2(name)
        self.index = index
        self.parent = parent_node
        self.joint_object = joint_object
        self.smode = self.parent.SPEED_MODE
        self.IGNORE_LIM = IGNORE_LIM
        self.MARGIN = MARGIN
        if not self.IGNORE_LIM:
            try:
                self.lower: float = joint_object.limit.lower + self.MARGIN
                self.upper: float = joint_object.limit.upper - self.MARGIN
            except AttributeError:
                self.IGNORE_LIM = True
                self.lower: float = -np.inf
                self.upper: float = np.inf
        else:
            self.lower: float = -np.inf
            self.upper: float = np.inf
        assert self.lower <= self.upper

        self.stateCommand = JState(name=self.name)
        self.stateSensor = JState(name=self.name)
        self.angle_updated = False
        self.speed_updated = False
        self.effort_updated = False
        self.clock: Clock = self.parent.get_clock()
        self.last_speed2angle_stamp: Time = self.clock.now()

        self.parent.create_subscription(
            Float64,
            f"ang_{self.corrected_name}_set",
            self.set_angle_cbk,
            10,
            callback_group=self.parent.cbk_legs,
        )
        self.parent.create_subscription(
            Float64,
            f"spe_{self.corrected_name}_set",
            self.set_speed_cbk,
            10,
            callback_group=self.parent.cbk_legs,
        )
        self.parent.create_subscription(
            Float64,
            f"eff_{self.corrected_name}_set",
            self.set_effort_cbk,
            10,
            callback_group=self.parent.cbk_legs,
        )
        self.pub_back_to_ros2_structure = self.parent.create_publisher(
            Float64,
            f"read_{self.corrected_name}",
            10,
        )
        self.activeAngleCheckTMR = self.parent.create_timer(
            timer_period_sec=0.2, callback=self.activeAngleCheckCBK
        )

    #     self.after3secTMR = self.parent.create_timer(
    #         timer_period_sec=3, callback=self.after3sec_tmrCBK
    #     )
    #
    # @error_catcher
    # def after3sec_tmrCBK(self):
    #     if self.stateSensor.position is None:
    #         self.parent.pwarn(f"No angles reading after 3s on {self.name}")
    #     self.parent.destroy_timer(self.after3secTMR)
    def info_when_angle_received(self):
        self.ang_receivedTMR = self.parent.create_timer(1, self.info_if_angle)

    def info_if_angle(self):
        if self.stateSensor.position is not None:
            self.parent.pinfo(
                f"{bcolors.OKGREEN}Angle received on {self.name}{bcolors.ENDC}"
            )
            self.parent.destroy_timer(self.ang_receivedTMR)

    def checkAngle(self, angle: Optional[float]) -> bool:
        if self.IGNORE_LIM or angle is None:
            return True
        return self.lower <= angle <= self.upper

    @error_catcher
    def activeAngleCheckCBK(self):
        if self.IGNORE_LIM or self.checkAngle(self.stateSensor.position):
            return
        self.set_speed_cbk(None)

    def applyAngleLimit(self, angle: float) -> Tuple[float, bool]:
        if self.IGNORE_LIM:
            return angle, True
        out = np.clip(angle, a_min=self.lower, a_max=self.upper)
        return out, out == angle

    def resetAnglesAtZero(self):
        # self.set_angle_cbk(0)
        self.stateCommand.position = 0
        self.stateCommand.time = self.parent.getNow()
        self.angle_updated = True

    def setJSSensor(self, js: JState):
        assert js.name == self.name
        self.stateSensor = js
        # if self.smode:
        # self.set_speed_cbk(None)
        self.publish_back_up_to_ros2()

    @error_catcher
    def set_angle_cbk(self, msg: Union[Float64, float]):
        if isinstance(msg, Float64):
            angle = msg.data
        else:
            angle = msg

        assert isinstance(angle, float)
        angle, isValid = self.applyAngleLimit(angle)

        self.stateCommand.position = angle
        self.stateCommand.time = self.parent.getNow()
        self.angle_updated = True

        self.parent.request_refresh()

    @error_catcher
    def set_speed_cbk(self, msg: Union[Float64, float, None]):
        speed: float
        if isinstance(msg, Float64):
            speed = msg.data
        elif msg is None:
            last_vel = self.stateCommand.velocity
            if last_vel is None:
                speed = 0.0
            else:
                speed = float(last_vel)
        else:
            speed = msg

        assert isinstance(speed, float)

        if self.stateSensor.position is None or self.IGNORE_LIM:
            goingLow = False
            goingHi = False
        else:
            goingLow = self.stateSensor.position <= self.lower and speed <= 0
            goingHi = self.stateSensor.position >= self.upper and speed >= 0

        mustStop = goingLow or goingHi
        if mustStop:
            speed = 0
            positionWasSent = self.stateSensor.position is not None
            if positionWasSent:
                self.set_angle_cbk(self.stateSensor.position)  # type: ignore
            self.parent.pwarn(f"{self.name} limit reached, stopping")

        justContinue = msg is None and not mustStop
        if justContinue:
            return

        self.stateCommand.velocity = speed
        self.speed_updated = True

        self.parent.request_refresh()
        return

    @error_catcher
    def set_effort_cbk(self, msg: Float64):
        effort = msg.data

        self.stateCommand.effort = effort
        self.effort_updated = True

        self.parent.request_refresh()
        return

    def angleToSpeed(self) -> None:
        if not self.smode:
            return
        if self.stateCommand.position is None or self.stateSensor.position is None:
            return
        delta1 = self.stateCommand.position - self.stateSensor.position
        delta2 = np.inf
        # delta2 = 2 * np.pi + delta1
        delta = delta1 if (abs(delta1) < abs(delta2)) else delta2

        if self.stateSensor.velocity is None:
            vel = 0
        else:
            vel = self.stateSensor.velocity

        speedPID = delta * P_GAIN - vel * D_GAIN

        if self.stateCommand.time is None or self.stateSensor.time is None:
            self.set_speed_cbk(speedPID)
            return

        arrivalTime = self.stateCommand.time + Duration(
            seconds=1 / self.parent.MVMT_UPDATE_RATE  # type:ignore
        )
        timeLeft: Duration = arrivalTime - self.stateSensor.time
        timeLeftSafe = max(0.1, rosTime2Float(timeLeft))
        perfectSpeed = delta / timeLeftSafe

        pid_is_slower = abs(speedPID) < abs(perfectSpeed)
        speed = speedPID if pid_is_slower else perfectSpeed
        self.set_speed_cbk(speed)
        return

    def get_stateCommand(self, reset: bool = True) -> JState:

        self.angleToSpeed()
        state_out = JState(name=self.stateCommand.name)
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

    def publish_back_up_to_ros2(self, angle: Optional[float] = None) -> None:
        angle_out: float

        if angle is None:
            if self.stateSensor.position is None:
                return
            else:
                angle_out = self.stateSensor.position
        else:
            angle_out = angle

        msg = Float64()
        msg.data = float(angle_out)
        self.pub_back_to_ros2_structure.publish(msg)


class JointNode(EliaNode):

    def __init__(self):
        # rclpy.init()
        super().__init__("joint_node")  # type: ignore
        self.DISABLE_AUTO_RELOAD = DISABLE_AUTO_RELOAD

        self.NAMESPACE = self.get_namespace()
        self.Alias = "JS"

        self.rem = rem_default
        self.remUnsafe = rem_default
        self.remModification = 0

        self.current_body_xyz: NDArray = np.array([0, 0, 0], dtype=float)
        self.current_body_quat: qt.quaternion = qt.one
        self.body_xyz_queue = np.zeros((0, 3), dtype=float)
        self.body_quat_queue = qt.from_float_array(np.zeros((0, 4), dtype=float))
        self.pubREMAP: Dict[str, Publisher] = {}

        self.setAndBlockForNecessaryClients(
            ["rviz_interface_alive", "driver/init"], all_requiered=False
        )

        self.pinfo(f"""{bcolors.OKBLUE}Interface connected to motors :){bcolors.ENDC}""")

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

        self.declare_parameter("control_rate", float(30))
        self.CONTROL_RATE = (
            self.get_parameter("control_rate").get_parameter_value().double_value
        )

        self.declare_parameter("mvmt_update_rate", float(30))
        self.MVMT_UPDATE_RATE = (
            self.get_parameter("mvmt_update_rate").get_parameter_value().double_value
        )

        self.declare_parameter("ignore_limits", False)
        self.IGNORE_LIM = (
            self.get_parameter("ignore_limits").get_parameter_value().bool_value
        )

        self.declare_parameter("limit_margin", float(0))
        self.MARGIN: float = (
            self.get_parameter("limit_margin").get_parameter_value().double_value
        )

        self.declare_parameter("speed_mode", False)
        self.SPEED_MODE: bool = (
            self.get_parameter("speed_mode").get_parameter_value().bool_value
        )
        # self.SPEED_MODE: bool = True
        # self.pwarn(self.SPEED_MODE)

        self.declare_parameter("start_coord", [0.0, 0.0, 0.0])
        self.START_COORD = np.array(
            self.get_parameter("start_coord").get_parameter_value().double_array_value,
            dtype=float,
        )
        self.current_body_xyz: NDArray = self.START_COORD

        self.declare_parameter("mirror_angles", False)
        self.MIRROR_ANGLES: bool = (
            self.get_parameter("mirror_angles").get_parameter_value().bool_value
        )  # DEBBUG: sends back received angles immediatly

        self.declare_parameter("urdf_path", str())
        self.urdf_path = (
            self.get_parameter("urdf_path").get_parameter_value().string_value
        )

        self.declare_parameter("pure_topic_remap", True)
        self.PURE_REMAP = (
            self.get_parameter("pure_topic_remap").get_parameter_value().bool_value
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

        # self.pinfo(f"Joints controled: {bcolors.OKCYAN}{self.joint_names}{bcolors.ENDC}")
        self.pinfo(
            f"Detected base_link: {bcolors.OKCYAN}{self.baselinkName}{bcolors.ENDC}"
        )

        # V Subscriber V
        #   \  /   #
        #    \/    #
        self.cbk_legs = MutuallyExclusiveCallbackGroup()
        self.jointHandlerL: List[MiniJointHandler] = []
        limits_undefined: List[str] = []
        for index, name in enumerate(self.joint_names):
            jObj = self.joints_objects[index]
            holder = MiniJointHandler(
                name, index, self, jObj, MARGIN=self.MARGIN, IGNORE_LIM=self.IGNORE_LIM
            )
            try:
                self.lower: float = float(jObj.limit.lower)
                self.upper: float = float(jObj.limit.upper)
            except AttributeError:
                limits_undefined.append(jObj.name)
            except TypeError:
                limits_undefined.append(jObj.name)
            self.jointHandlerL.append(holder)
        if limits_undefined:
            self.pinfo(
                f"{bcolors.WARNING}Undefined limits{bcolors.ENDC} "
                f"in urdf for joint {limits_undefined}"
            )
        else:
            self.pinfo(f"{bcolors.OKBLUE}All URDF limits defined{bcolors.ENDC} ")
        self.jointHandlerDic = dict(zip(self.joint_names, self.jointHandlerL))

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

        self.sensor_sub = self.create_subscription(
            JointState,
            "joint_states",
            self.SensorJSRecieved,
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
        self.refresh_timer = self.create_timer(1 / self.CONTROL_RATE, self.__refresh)
        self.go_in_eco = self.create_timer(TIME_TO_ECO_MODE, self.eco_mode)
        self.go_in_eco.cancel()
        self.eco_timer = self.create_timer(ECO_MODE_PERIOD, self.__refresh)
        self.eco_timer.cancel()
        self.firstSpin: Timer = self.create_timer(1 / 100, self.firstSpinCBK)
        self.reloadRemModule: Timer = self.create_timer(
            RELOAD_MODULE_DUR,
            ((lambda: None) if self.DISABLE_AUTO_RELOAD else self.reloadREM),
        )
        self.angle_read_checkTMR = self.create_timer(1, self.angle_read_checkTMRCBK)
        self.angle_read_checkTMR.cancel()
        #    /\    #
        #   /  \   #
        # ^ Timer ^

        self.liveOk = False
        self.reloadREM()

    def angle_read_checkTMRCBK(self):
        undefined: List[str] = []
        defined: List[str] = []
        for name, jobj in self.jointHandlerDic.items():
            if jobj.stateSensor.position is None:
                undefined.append(name)
                jobj.info_when_angle_received()
            else:
                defined.append(name)
        i = f"{bcolors.OKGREEN}all{bcolors.ENDC}"
        if undefined:
            self.pwarn(
                f"No angle readings yet on {list_cyanize(undefined)}. Might not be published."
            )
            i = "some"
        if defined:
            self.pinfo(
                f"{bcolors.OKGREEN}Angle recieved{bcolors.ENDC} on {i} joints {list_cyanize(defined)}"
            )
        if not undefined:
            i = "all joints"
        self.destroy_timer(self.angle_read_checkTMR)

    @error_catcher
    def firstSpinCBK(self):
        self.iAmAlive = self.create_service(Empty, "joint_alive", lambda i, o: o)
        self.destroy_timer(self.firstSpin)

        # send empty command to initialize (notabily Rviz interface)
        empty = JointState(name=self.joint_names)
        empty.header.stamp = self.getNow().to_msg()
        self.joint_state_pub.publish(empty)

        # we should not start at zero when using real robot
        if INIT_AT_ZERO:
            for jointMiniNode in self.jointHandlerL:
                jointMiniNode.resetAnglesAtZero()
        self.angle_read_checkTMR.reset()

    def reloadREM(self):
        if not self.PURE_REMAP:
            return
        if self.DISABLE_AUTO_RELOAD:
            return
        remPath = os.path.join(
            get_src_folder("easy_robot_control"),
            "easy_robot_control",
            "python_package_include",
            "pure_remap.py",
        )
        try:
            currentModTime = os.path.getmtime(remPath)
        except:
            currentModTime = -1
        if currentModTime == self.remModification:
            return
        else:
            self.remModification = currentModTime
        failed = False
        if self.remUnsafe == rem_default:
            try:
                self.remUnsafe = import_module_from_path("pure_remap", remPath)
            except:
                failed = True
        else:
            try:
                importlib.reload(self.remUnsafe)
            except:
                failed = True

        with open(os.devnull, "w") as fnull:
            original_stdout = sys.stdout  # Save the original stdout
            original_stderr = sys.stderr  # Save the original stderr
            # if self.rem == self.remUnsafe:
            sys.stdout = fnull  # Redirect stdout to devnull
            sys.stderr = fnull  # Redirect stderr to devnull
            result = pytest.main(
                [
                    remPath,
                    "--disable-warnings",
                    "-q",
                    "--tb=short",
                    "--cache-clear",
                    # "--continue-on-collection-errors",
                ]
            )
            sys.stdout = original_stdout  # Restore original stdout
            sys.stderr = original_stderr  # Restore original stderr
        if result != 0 or failed:
            t = "Tests" if not failed else "Import"
            self.perror(
                f"{t} failed on [{remPath}] error code {EXIT_CODE_TEST[result]}\n"
                f"run `pytest {remPath}` to generate the bug report\n"
                f"falling back onto build time library until valid /src lib is available"
            )
            self.rem = rem_default
        else:
            self.pinfo(
                f"{bcolors.OKGREEN}pure_remap.py loaded from live /src directory{bcolors.ENDC}"
            )
            self.rem = self.remUnsafe
        self.updateMapping()

    def updateMapping(self):

        for k in self.rem.remap_topic_com.keys():
            notMyJob = not k in self.joint_names
            if notMyJob:
                continue
            keyExists = k in self.pubREMAP.keys()
            if keyExists:
                isSameTopic = self.rem.remap_topic_com[k] == self.pubREMAP[k].topic_name
                if isSameTopic:
                    continue
                else:
                    self.destroy_publisher(self.pubREMAP[k])
            self.pubREMAP[k] = self.create_publisher(
                Float64, self.rem.remap_topic_com[k], 10
            )
        toBeDeletedBuf: List[str] = []
        for k, pub in self.pubREMAP.items():
            hasBeenDeleted = k not in self.rem.remap_topic_com.keys()
            if hasBeenDeleted:
                self.destroy_publisher(pub)
                toBeDeletedBuf.append(k)
        for k in toBeDeletedBuf:
            del self.pubREMAP[k]

        comType = "speed" if self.SPEED_MODE else "position"
        self.pinfo(
            f"Duplicating {bcolors.OKCYAN}{comType}{bcolors.ENDC} commands from joint: "
            f"{bcolors.OKCYAN}{list(self.pubREMAP.keys())}{bcolors.ENDC} onto topics: "
            f"{bcolors.OKCYAN}{[self.rem.remap_topic_com[k] for k in self.pubREMAP]}{bcolors.ENDC}"
        )
        co = list(set(list(self.rem.remap_com.keys())) & set(self.joint_names))
        self.pinfo(
            f"Remapping names of {self.joint_state_pub.topic_name} (motor) from joint: "
            f"{bcolors.OKCYAN}{co}{bcolors.ENDC} onto state name: "
            f"{bcolors.OKCYAN}{[self.rem.remap_com[k] for k in co]}{bcolors.ENDC}"
        )

        co = list(set(list(self.rem.remap_sens.values())) & set(self.joint_names))
        inverted_dict = {v: k for k, v in self.rem.remap_sens.items()}
        self.pinfo(
            f"Remapping names of {self.sensor_sub.topic_name} (sensor) from state name: "
            f"{bcolors.OKCYAN}{[inverted_dict[k] for k in co]}{bcolors.ENDC} onto joint: "
            f"{bcolors.OKCYAN}{co}{bcolors.ENDC}"
        )

    def remap_JointState_sens(self, js: JointState) -> None:
        if not self.PURE_REMAP:
            return
        pos_list = list(js.position).copy()
        name_list = list(js.name).copy()
        shapedPos: List[float] = [
            (
                self.rem.shaping_sens[name](pos)
                if (name in self.rem.shaping_sens.keys())
                else pos
            )
            for (name, pos) in zip(name_list, pos_list)
        ]
        js.position = shapedPos
        new = [
            self.rem.remap_sens[n] if (n in self.rem.remap_sens.keys()) else n
            for n in name_list
        ]
        js.name = new

    def remap_JointState_com(self, js: JointState) -> None:
        if not self.PURE_REMAP:
            return
        pos_list = list(js.position).copy()
        name_list = list(js.name).copy()
        shapedPos: List[float] = [
            (
                self.rem.shaping_com[name](pos)
                if (name in self.rem.shaping_com.keys())
                else pos
            )
            for (name, pos) in zip(name_list, pos_list)
        ]
        js.position = shapedPos
        mappedName = [
            self.rem.remap_com[n] if (n in self.rem.remap_com.keys()) else n
            for n in name_list
        ]
        js.name = mappedName

    @error_catcher
    def SensorJSRecieved(self, jsReading: JointState) -> None:
        self.remap_JointState_sens(jsReading)
        jointsHandled: List[str] = list(self.jointHandlerDic.keys())
        areAngle = len(jsReading.position) > 0
        areVelocity = len(jsReading.velocity) > 0
        areEffort = len(jsReading.effort) > 0

        stamp: Time
        if jsReading.header.stamp is None:
            stamp = self.getNow()
        else:
            stamp = Time.from_msg(jsReading.header.stamp)

        nothingInside = not (areAngle or areVelocity or areEffort)
        if nothingInside:
            return

        for index, name in enumerate(jsReading.name):
            isResponsable = name in jointsHandled
            if not isResponsable:
                continue  # skips
            handler: MiniJointHandler = self.jointHandlerDic[name]
            js = JState(name=name, time=stamp)

            if areAngle:
                js.position = jsReading.position[index]
            if areVelocity:
                js.velocity = jsReading.velocity[index]
            if areEffort:
                js.effort = jsReading.effort[index]

            handler.setJSSensor(js)

    def __pull_states(self) -> List[JState]:
        allStates: List[JState] = []

        for jointMiniNode in self.jointHandlerL:
            state: JState = jointMiniNode.get_stateCommand()
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

            workingJS.name.append(state.name)
            if state.position is not None:
                workingJS.position.append(state.position)
            if state.velocity is not None:
                workingJS.velocity.append(state.velocity)
            if state.effort is not None:
                workingJS.effort.append(state.effort)

        withoutNone: List[JointState] = list(outDic.values())
        return withoutNone

    def pub_current_jointstates(self, time_stamp: Optional[Time] = None):
        if time_stamp is None:
            now: Time = self.get_clock().now()
        else:
            now: Time = time_stamp
        time_now_stamp = now.to_msg()

        allStates: List[JState] = self.__pull_states()
        self.send_pure(allStates)
        ordinatedJointStates: List[JointState] = self.__stateOrderinator3000(allStates)
        for jsMSG in ordinatedJointStates:
            jsMSG.header.stamp = time_now_stamp
            self.remap_JointState_com(jsMSG)
            self.joint_state_pub.publish(jsMSG)
            if self.MIRROR_ANGLES:
                self.SensorJSRecieved(jsMSG)

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

        self.pub_current_jointstates(now)

        self.tf_broadcaster.sendTransform(body_transform)

        if self.go_in_eco.is_canceled() and self.eco_timer.is_canceled():
            self.go_in_eco.reset()
        return

    def send_pure(self, all_states: List[JState]) -> None:
        if not self.PURE_REMAP:
            return

        for js in all_states:
            if js.name is None:
                continue
            isRemapped = js.name in self.pubREMAP.keys()
            if not isRemapped:
                continue
            data = js.position
            if self.SPEED_MODE or data is None:
                data = js.velocity
            if data is None:
                data = js.effort
            if data is None:
                continue

            data = (
                self.rem.shaping_topic_com[js.name](data)
                if js.name in self.rem.shaping_topic_com.keys()
                else data
            )

            pub = self.pubREMAP[js.name]
            pub.publish(
                Float64(
                    data=float(data),
                )
            )

    def __pop_and_load_body(self):
        empty = self.body_xyz_queue.shape[0] <= 0
        if not empty:
            self.current_body_xyz = self.body_xyz_queue[0, :]
            self.current_body_quat = self.body_quat_queue[0]
            self.body_xyz_queue = np.delete(self.body_xyz_queue, 0, axis=0)
            self.body_quat_queue = np.delete(self.body_quat_queue, 0, axis=0)

    @error_catcher
    def robot_body_pose_cbk(self, msg: Transform):
        tra, quat = self.tf2np(msg)
        self.current_body_xyz = tra
        self.current_body_quat = quat
        self.request_refresh()

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

        samples = int(self.MOVEMENT_TIME * self.CONTROL_RATE)
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
            self.eco_timer.cancel()

    @error_catcher
    def eco_mode(self):
        if self.eco_timer.is_canceled():
            # self.pwarn("eco mode")
            self.refresh_timer.cancel()
            self.eco_timer.reset()


def main(args=None):
    myMain(JointNode)


if __name__ == "__main__":
    main()
