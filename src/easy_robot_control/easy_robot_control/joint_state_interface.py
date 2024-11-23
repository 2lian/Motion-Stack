import matplotlib

from easy_robot_control.calibration import csv_to_dict, update_csv

matplotlib.use("Agg")  # fix for when there is no display
import importlib.util
import os
import sys
from dataclasses import dataclass
from datetime import datetime

# from pytest import ExitCode
from typing import Dict, Final, List, Optional, Tuple

import numpy as np
import pytest
import python_package_include.pure_remap
import quaternion as qt
import tf2_ros
from custom_messages.srv import SendJointState
from EliaNode import (
    EliaNode,
    Joint,
    bcolors,
    error_catcher,
    get_src_folder,
    list_cyanize,
    loadAndSet_URDF,
    myMain,
    replace_incompatible_char_ros2,
    rosTime2Float,
)
from geometry_msgs.msg import Transform, TransformStamped
from numpy.typing import NDArray
from python_package_include.joint_state_util import (
    JState,
    impose_state,
    intersect_names,
    js_changed,
    js_copy,
    js_from_ros,
    stateOrderinator3000,
)
from rclpy.clock import Clock
from rclpy.node import (
    MutuallyExclusiveCallbackGroup,
    Publisher,
    ReentrantCallbackGroup,
    Service,
    Timer,
    Union,
)
from rclpy.time import Duration, Time
from scipy.spatial import geometric_slerp
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from std_srvs.srv import Empty as EmptySrv

rem_default = python_package_include.pure_remap
DISABLE_AUTO_RELOAD = False  # s
RELOAD_MODULE_DUR = 1  # s
P_GAIN = 3.5
D_GAIN = 0.00005  # can be improved
INIT_AT_ZERO = False  # dangerous
CLOSE_ENOUGH = np.deg2rad(0.25)
LATE = 0.3

WORKSPACE_PATH = os.path.abspath(
    os.path.join(get_src_folder("easy_robot_control"), "../..")
)
RECOVERY_PATH = os.path.join(
    WORKSPACE_PATH,
    "angle_recovery",
)
if not os.path.exists(RECOVERY_PATH):
    os.makedirs(RECOVERY_PATH)
OFFSET_PATH = os.path.join(RECOVERY_PATH, "offset.csv")
ANGLE_PATH = os.path.join(
    RECOVERY_PATH,
    f"off{datetime.now().isoformat(timespec='seconds').replace(':', '-')}.csv",
)

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

TOL_NO_CHANGE: Final[JState] = JState(
    name="",
    time=Time(seconds=ECO_MODE_PERIOD),
    position=np.deg2rad(0.1),
    velocity=np.deg2rad(0.1),
    effort=np.deg2rad(0.1),
)

RVIZ_SPY_RATE = 2  # Hz


def import_module_from_path(module_name, file_path):
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module  # Add the module to sys.modules

    # Load the module
    spec.loader.exec_module(module)

    return module


class JointHandler:
    """This handles a single joint.
    The main purpose is to update stateSensor and stateCommand. As well as getting the
    newest values for those (in order to not continuously publish unchanging data).
    """

    def __init__(
        self,
        name: str,
        parent_node: "JointNode",
        joint_object: Joint,
        IGNORE_LIM: bool = False,
        MARGIN: float = 0.0,
    ):
        self.name = name
        self._corrected_name = replace_incompatible_char_ros2(name)
        self.parent = parent_node
        self.joint_object = joint_object
        self._smode = self.parent.SPEED_MODE
        self.IGNORE_LIM = IGNORE_LIM
        self.MARGIN = MARGIN
        self.offset: float = 0.0
        self.lower: float = -np.inf
        self.upper: float = np.inf
        self.load_limit(self.IGNORE_LIM)

        assert self.name == self.joint_object.name

        self.stateCommand = JState(name=self.name)
        self.fresh_command = self.stateCommand
        self.stateSensor = JState(name=self.name)
        self.fresh_sensor = self.stateSensor

        self._activeAngleCheckTMR = self.parent.create_timer(
            timer_period_sec=0.2, callback=self._activeAngleCheckCBK
        )
        self._ang_speakupTMR: Optional[Timer] = None

    def load_limit(self, ignore: bool, jobj: Optional[Joint] = None):
        """Loads the limit from the (urdf) joint object

        Args:
            ignore: if limits should be ignored
        """
        if jobj is None:
            jobj = self.joint_object
        if not ignore:
            try:
                self.lower: float = jobj.limit.lower + self.MARGIN
                self.upper: float = jobj.limit.upper - self.MARGIN
            except AttributeError:
                self.IGNORE_LIM = True
                self.lower: float = -np.inf
                self.upper: float = np.inf
        else:
            self.lower: float = -np.inf
            self.upper: float = np.inf
        assert self.lower <= self.upper

    @property
    def limit_rejected(self) -> bool:
        """
        Returns: if the limit was rejected when loading (often when not defined in urdf)
        """
        return self.parent.IGNORE_LIM != self.IGNORE_LIM

    def speakup_when_angle(self):
        """start a verbose check every seconds for new angles"""
        if self._ang_speakupTMR is None:
            self._ang_speakupTMR = self.parent.create_timer(1, self._ang_speakupTMRCBK)
        if self._ang_speakupTMR.is_canceled():
            self._ang_speakupTMR.reset()
        self._ang_speakupTMRCBK()

    def _ang_speakupTMRCBK(self):
        """if angle is defined, prints happy meseage and cancel the timer"""
        if self.stateSensor.position is not None:
            self.parent.pinfo(
                f"{bcolors.OKGREEN}Angle received on {self.name}{bcolors.ENDC}"
            )
            if self._ang_speakupTMR is not None:
                self.parent.destroy_timer(self._ang_speakupTMR)

    def checkAngle(self, angle: Optional[float]) -> bool:
        """True is angle is valid or None"""
        if self.IGNORE_LIM or angle is None:
            return True
        return self.lower <= angle <= self.upper

    @error_catcher
    def _activeAngleCheckCBK(self):
        """check the sensor angle for validity, changes the speed if invalid"""
        if self.checkAngle(self.stateSensor.position):
            return
        self._update_speed_cmd(None)

    def applyAngleLimit(self, angle: float) -> Tuple[float, bool]:
        """Clamps the angle between the joints limits"""
        if self.IGNORE_LIM:
            return angle, True
        out = np.clip(angle, a_min=self.lower, a_max=self.upper)
        return out, out == angle

    def resetAnglesAtZero(self):
        self._update_angle_cmd(0)

    def _process_command(self, js: JState) -> JState:
        """Processes incomming jstate in view of storing it as the command.
        it applies angle limits, checks, speed ...

        Args:
            js: incomming unprocessed JState

        Returns:
            Processed JState, ready to be used
        """
        js = js_copy(js)
        if js.time is not None:
            js.time = js.time
        else:
            js.time = self.parent.getNow()

        if js.position is not None:
            js.position = self.process_angle_command(js.position)
        if js.velocity is not None:
            v = self.process_velocity_command(js.velocity)
            emergency = v is None
            if emergency and self.stateSensor.position is not None:
                js.position = self.process_angle_command(self.stateSensor.position)
            if emergency:
                v = 0
            js.velocity = v
        if js.effort is not None:
            js.effort = self.process_effort_command(js.effort)
        return js

    def update_js_command(self, js: JState):
        """Updates the stateCommand to a new js."""
        assert js.name == self.name
        js = self._process_command(js)

        self.stateCommand = impose_state(self.stateCommand, js)
        self.fresh_command = impose_state(self.fresh_command, js)
        self.parent.request_refresh()

    def is_new_jssensor(self, js: JState):
        """True if js is different enough from the last received.
        Also true if stateSensor is more the TOL_NO_CHANGE.time old relative to the new
        """
        d = js_copy(TOL_NO_CHANGE)
        if self.stateSensor.time is not None and TOL_NO_CHANGE.time is not None:
            # black magic to keep publishing in sync when no changes
            # We basically refresh every t=N*dt, and not dt after the previous
            ts = self.stateSensor.time.nanoseconds
            dt = TOL_NO_CHANGE.time.nanoseconds
            d.time = Time(nanoseconds=dt - ts % dt)
        something_changed = js_changed(js, self.stateSensor, delta=d)
        return something_changed

    def setJSSensor(self, js: JState):
        """Updates the stateSensor to a new js."""
        assert js.name == self.name

        if not self.is_new_jssensor(js):
            return

        self.stateSensor = js  # no processing for sensor
        self.fresh_sensor = impose_state(self.fresh_sensor, js)
        self.sensor_updated = True

    def process_angle_command(self, angle: float) -> float:
        """This runs on new js before updating stateCommand"""
        angle_in = angle
        angle, isValid = self.applyAngleLimit(angle)
        if not isValid:
            self.parent.pwarn(
                f"{bcolors.OKCYAN}{self.name}{bcolors.WARNING} clipped to limit. "
                f"{angle_in:.2f} -> {angle:.2f}"
            )
        return angle + self.offset

    @error_catcher
    def _update_angle_cmd(self, msg: Union[Float64, float]):
        """Updates stateCommand by providing only an angle.
        should be avoided as the timestamp will be set to now.
        """
        if isinstance(msg, Float64):
            angle = msg.data
        else:
            angle = msg

        assert isinstance(angle, float)

        new_js = JState(self.name)
        new_js.position = angle
        new_js.time = self.parent.getNow()
        self.update_js_command(new_js)

    def process_velocity_command(self, speed: float) -> Optional[float]:
        """This runs on new js before updating stateCommand"""
        if self.stateSensor.position is None or self.IGNORE_LIM:
            goingLow = False
            goingHi = False
        else:
            goingLow = self.stateSensor.position <= self.lower and speed <= 0
            goingHi = self.stateSensor.position >= self.upper and speed >= 0

        mustStop = goingLow or goingHi
        if mustStop:
            return None
        return speed

    @error_catcher
    def _update_speed_cmd(self, msg: Union[Float64, float, None]):
        """Updates stateCommand by providing only an speed.
        should be avoided as the timestamp will be set to now.
        """
        speed: float
        if isinstance(msg, Float64):
            speed = msg.data
            self.stateCommand.position = None

        elif msg is None:
            last_vel = self.stateCommand.velocity
            if last_vel is None:
                speed = 0.0
            else:
                speed = float(last_vel)
        else:
            speed = float(msg)
        assert isinstance(speed, float)

        new_js = JState(self.name)
        new_js.velocity = speed
        new_js.time = self.parent.getNow()
        self.update_js_command(new_js)

    def process_effort_command(self, eff: float) -> float:
        """This runs on new js before updating stateCommand"""
        return eff

    @error_catcher
    def set_effortCBK(self, msg: Union[Float64, float]):
        """Updates stateCommand by providing only an effort.
        should be avoided as the timestamp will be set to now.
        """
        if isinstance(msg, Float64):
            effort = msg.data
        else:
            effort = msg

        new_js = JState(self.name)
        new_js.effort = effort
        new_js.time = self.parent.getNow()
        self.update_js_command(new_js)

    def _angle_feedback(self) -> None:
        """PID updating the speed command based on last stateSensor"""
        if not self._smode:
            return
        if self.stateCommand.position is None or self.stateSensor.position is None:
            return
        delta = self.stateCommand.position - self.stateSensor.position
        small_angle = abs(delta) < CLOSE_ENOUGH
        if small_angle:
            self._update_speed_cmd(0)
            return

        if self.stateSensor.velocity is None:
            vel = 0
        else:
            vel = self.stateSensor.velocity

        speedPID = delta * P_GAIN - vel * D_GAIN

        if self.stateCommand.time is None or self.stateSensor.time is None:
            self._update_speed_cmd(speedPID)
            return

        period = 1 / self.parent.MVMT_UPDATE_RATE + LATE
        arrivalTime = self.stateCommand.time + Duration(
            seconds=period  # type:ignore
        )
        # time left to reach the target
        timeLeft: Duration = arrivalTime - self.stateSensor.time
        # if less than 5% of the time left to reach, we will go slower and not freak out
        # with infinite speed
        timeLeftSafe = max(0.05 * period, rosTime2Float(timeLeft))
        perfectSpeed = delta / timeLeftSafe

        pid_is_slower = abs(speedPID) < abs(perfectSpeed)
        # pick the slower of the two methodes.
        # when far away we move at constant speed to reach the destination on the next
        # command. If close, or when the PID wants to slow down, the PID is used
        speed = speedPID if pid_is_slower else perfectSpeed
        self._update_speed_cmd(speed)
        return

    def get_fresh_sensor(self, reset: bool = True) -> JState:
        """returns sensor data that is newer than the last time it was called.
        if the sensor data didn't changed enough to trigger a refresh, this will
        be full of None. If a refresh occured, the None will be replaced by the non-None
        values in the new sensor data.

        example: if you stop sending speed sensor data after sending a bunch of speeds.
        This speed will switch to None, it will not  continue to be the last received
        speed.
        This last received speed is still available in stateSensor.
        """
        out = self.fresh_sensor
        if out.position is not None:
            out.position -= self.offset
        if reset:
            self.fresh_sensor = JState(name=self.name)
        return out

    def get_freshCommand(self, reset: bool = True) -> JState:
        """returns command data that is newer than the last time it was called.
        full of None is not newer"""
        self._angle_feedback()
        out = self.fresh_command
        if reset:
            self.fresh_command = JState(name=self.name)
        return out


class JointNode(EliaNode):

    def __init__(self):
        # rclpy.init()
        super().__init__("joint")  # type: ignore
        self.DISABLE_AUTO_RELOAD = DISABLE_AUTO_RELOAD

        self.NAMESPACE = self.get_namespace()

        self.declare_parameter("leg_number", 0)
        self.leg_num = (
            self.get_parameter("leg_number").get_parameter_value().integer_value
        )
        self.Alias = f"J{self.leg_num}"

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

        self.declare_parameter("add_joints", [""])
        self.ADD_JOINTS: List[str] = list(
            self.get_parameter("add_joints").get_parameter_value().string_array_value
        )
        if self.ADD_JOINTS == [""]:
            self.ADD_JOINTS = []

        # self.SPEED_MODE: bool = True
        # self.pwarn(self.SPEED_MODE)

        self.declare_parameter("start_coord", [0.0, 0.0, 0.0])
        self.START_COORD = np.array(
            self.get_parameter("start_coord").get_parameter_value().double_array_value,
            dtype=float,
        )
        if np.isnan(self.START_COORD).any():
            self.current_body_xyz: NDArray = np.array([0, 0, 0], dtype=float)
            self.dont_handle_body = True
        else:
            self.current_body_xyz: NDArray = self.START_COORD
            self.dont_handle_body = False

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

        self.declare_parameter("start_effector_name", str(f""))
        self.start_effector: str | None = (
            self.get_parameter("start_effector_name").get_parameter_value().string_value
        )
        if self.start_effector == "":
            self.start_effector = None

        self.declare_parameter("end_effector_name", str(self.leg_num))
        end_effector: str = (
            self.get_parameter("end_effector_name").get_parameter_value().string_value
        )

        self.end_effector_name: Union[str, int]
        if end_effector.isdigit():
            self.end_effector_name = int(end_effector)
        else:
            if end_effector == "ALL":
                self.end_effector_name = None
            elif end_effector == "":
                self.end_effector_name = self.leg_num
            else:
                self.end_effector_name = end_effector

        #    /\    #
        #   /  \   #
        # ^ Params ^
        self.pinfo(f"chain: {self.start_effector} -> {self.end_effector_name}")
        # self.perror(f"{self.start_effector==self.end_effector_name}")

        # self.end_effector_name = None
        # self.start_effector = None
        (
            self.model,
            self.ETchain,
            self.joint_names,
            self.joints_objects,
            self.last_link,
        ) = loadAndSet_URDF(self.urdf_path, self.end_effector_name, self.start_effector)
        # if end_effector == "ALL":
        # self.end_effector_name = self.leg_num

        try:  # kill me
            if isinstance(self.end_effector_name, str) and isinstance(
                self.start_effector, str
            ):
                (  # don't want to see this
                    model2,
                    ETchain2,
                    joint_names2,
                    joints_objects2,
                    last_link2,
                ) = loadAndSet_URDF(
                    self.urdf_path, self.start_effector, self.end_effector_name
                )
                if len(joint_names2) > len(self.joint_names) and len(
                    joints_objects2
                ) > len(self.joints_objects):
                    joint_names2.reverse()
                    self.joint_names = joint_names2
                    joints_objects2.reverse()
                    self.joints_objects = joints_objects2
        except:
            self.pinfo(f"link tree could not be reversed")
        # self.baselinkName = self.model.base_link.name # base of the whole model
        if self.start_effector is None:
            self.baselinkName = self.model.base_link.name
        else:
            self.baselinkName = self.start_effector

        if self.baselinkName != self.model.base_link.name:
            self.pinfo(
                f"base_link forced to `{self.baselinkName}` "
                f"instead of the tf root `{self.model.base_link.name}`, "
                f"this can render part of the tf tree missing, or worse"
            )
        #
        # if not self.joint_names:
        #     self.dont_handle_body = True
        # else:
        #     self.dont_handle_body = False

        self.joint_names += self.ADD_JOINTS
        self.joints_objects += [
            Joint(
                joint_type="continuous",
                parent=None,
                child=None,
                name=jn,
                # limit=[0, 0],
            )
            for jn in self.ADD_JOINTS
        ]

        # self.pinfo(f"Joints controled: {bcolors.OKCYAN}{self.joint_names}{bcolors.ENDC}")
        ee = self.last_link.name if self.last_link is not None else "all joints"
        self.pinfo(
            f"Using base_link: {bcolors.OKCYAN}{self.baselinkName}{bcolors.ENDC}"
            f", to ee:  {bcolors.OKCYAN}{ee}{bcolors.ENDC}"
        )

        self.cbk_legs = MutuallyExclusiveCallbackGroup()
        self.jointHandlerL: List[JointHandler] = []
        self.jointHandlerDic: Dict[str, JointHandler] = {}
        limits_undefined: List[str] = []
        # self.pwarn([j.name for j in self.joints_objects])
        # self.pwarn(self.joint_names)
        for index, name in enumerate(self.joint_names):
            jObj = None
            for j in self.joints_objects:
                if j.name == name:
                    jObj = j
                    break
            assert jObj is not None

            handler = JointHandler(
                name, self, jObj, MARGIN=self.MARGIN, IGNORE_LIM=self.IGNORE_LIM
            )
            if handler.limit_rejected:
                limits_undefined.append(jObj.name)
            self.jointHandlerL.append(handler)
            self.jointHandlerDic[name] = handler
        if limits_undefined:
            self.pinfo(
                f"{bcolors.WARNING}Undefined limits{bcolors.ENDC} "
                f"in urdf for joint {limits_undefined}"
            )
        else:
            self.pinfo(f"{bcolors.OKBLUE}All URDF limits defined{bcolors.ENDC} ")
        self.jointHandlerDic = dict(zip(self.joint_names, self.jointHandlerL))

        # V Subscriber V
        #   \  /   #
        #    \/    #
        self.js_from_lvl2SUB = self.create_subscription(
            JointState,
            "joint_set",
            self.js_from_lvl2,
            10,
            callback_group=self.cbk_legs,
        )

        self.js_from_lvl0SUB = self.create_subscription(
            JointState,
            "joint_states",
            self.js_from_lvl0,
            10,
        )
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

        if self.dont_handle_body:
            self.destroy_subscription(self.smooth_body_pose_sub)
            self.destroy_subscription(self.body_pose_sub)

        #    /\    #
        #   /  \   #
        # ^ Subscriber ^

        # V Publisher V
        #   \  /   #
        #    \/    #
        self.jsPUB_to_lvl2 = self.create_publisher(
            JointState,
            "joint_read",
            10,
            callback_group=self.cbk_legs,
        )

        self.jsPUB_to_lvl0 = self.create_publisher(JointState, "joint_commands", 10)
        # self.body_pose_pub = self.create_publisher(
        # TFMessage,
        # '/BODY', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.rviz_spyPUB = self.create_publisher(JointState, "rviz_spy", 10)
        #    /\    #
        #   /  \   #
        # ^ Publisher ^

        # V Service V
        #   \  /   #
        #    \/    #
        self.iAmAlive: Optional[Service] = None
        self.set_offsetSRV: Service = self.create_service(
            SendJointState, "set_offset", self.set_offsetSRVCBK
        )
        self.go_zero_all: Service = self.create_service(
            EmptySrv, "go_zero_all", self.go_zero_allCBK
        )
        #    /\    #
        #   /  \   #
        # ^ Service ^

        # V Timer V
        #   \  /   #
        #    \/    #
        self._send_commandTMR = self.create_timer(
            1 / self.MVMT_UPDATE_RATE, self._send_commandTMRCBK
        )
        self._send_readTMR = self.create_timer(
            1 / self.MVMT_UPDATE_RATE, self._send_sensorTMRCBK
        )
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
        self.angle_read_checkTMR = self.create_timer(0.05, self.angle_read_checkTMRCBK)
        self.angle_read_checkTMR.cancel()
        # This must not start until user asks for it
        self.save_current_angleTMR = self.create_timer(3, self.save_current_angle)
        self.rviz_spyTMR = self.create_timer(1 / RVIZ_SPY_RATE, self.rviz_spyTMRCBK)
        #    /\    #
        #   /  \   #
        # ^ Timer ^

        self.liveOk = False
        self.reloadREM()

    def send_to_lvl0(self, states: List[JState]):
        """Sends states to lvl0 (commands for motors).
        Change/overload this method with what you need"""
        stamp = self.getNow().to_msg()
        msgs = stateOrderinator3000(states)
        for msg in msgs:
            msg.header.stamp = stamp
            self.jsPUB_to_lvl0.publish(msg)

    def send_to_lvl2(self, states: List[JState]):
        """Sends states to lvl2 (states for ik).
        Change/overload this method with what you need"""
        stamp = self.getNow().to_msg()
        msgs = stateOrderinator3000(states)
        for msg in msgs:
            msg.header.stamp = stamp
            self.jsPUB_to_lvl2.publish(msg)

    @error_catcher
    def js_from_lvl0(self, msg: JointState):
        """Callbk when a JointState arrives from the lvl0 (states from motor).
        Converts it into a list of states, then hands it to the general function
        """
        jointsHandled: List[str] = list(self.jointHandlerDic.keys())
        msg = intersect_names(msg, jointsHandled)
        if msg.header.stamp is None:
            msg.header.stamp = self.getNow().to_msg()
        states = js_from_ros(msg)
        self._coming_from_lvl0(states)

    @error_catcher
    def js_from_lvl2(self, msg: JointState):
        """Callbk when a JointState arrives from the lvl2 (commands from ik).
        Converts it into a list of states, then hands it to the general function
        """
        jointsHandled: List[str] = list(self.jointHandlerDic.keys())
        msg = intersect_names(msg, jointsHandled)
        if msg.header.stamp is None:
            msg.header.stamp = self.getNow().to_msg()
        states = js_from_ros(msg)
        self._coming_from_lvl2(states)

    def _coming_from_lvl2(self, states: List[JState]):
        """Processes incomming commands from lvl2 ik.
        Call this function after processing the ros message"""
        stamp = None
        # self.remap_JointState_sens(js_reading)
        for s in states:
            if s.time is None:
                stamp = self.getNow() if stamp is None else stamp
                s.time = stamp
        self._push_commands(states)

    def _coming_from_lvl0(self, states: List[JState]):
        """Processes incomming sensor states from lvl0 motors.
        Call this function after processing the ros message"""
        stamp = None
        # self.remap_JointState_sens(js_reading)
        for s in states:
            if s.time is None:
                stamp = self.getNow() if stamp is None else stamp
                s.time = stamp
        self._push_sensors(states)

    @error_catcher
    def _send_sensorTMRCBK(self):
        """pulls and resets fresh sensor data, then sends it to lvl2"""
        states_sensor = self._pull_sensors()
        self.send_to_lvl2(states_sensor)

    @error_catcher
    def _send_commandTMRCBK(self):
        """pulls and resets fresh command data, then sends it to lvl0"""
        # return
        states_sensor = self._pull_commands()
        self.send_to_lvl0(states_sensor)

    def rviz_spyTMRCBK(self):
        out = JointState()
        out.name = []
        out.position = []
        out.header.stamp = self.get_clock().now().to_msg()
        for jobj in self.jointHandlerDic.values():
            out.name.append(jobj.name)
            if jobj.stateCommand.position is None:
                out.position.append(0)
            else:
                out.position.append(jobj.stateCommand.position - jobj.offset)
        self.rviz_spyPUB.publish(out)

    def save_current_angle(self):
        for name, jobj in self.jointHandlerDic.items():
            if jobj.stateSensor.position is None:
                continue
            update_csv(ANGLE_PATH, name, -(jobj.stateSensor.position - jobj.offset))

    def save_current_offset(self):
        """DO NOT DO THIS AUTOMATICALLY, IT COULD BE DESTRUCTIVE OF VALUABLE INFO"""
        user_disp = ""
        old_d = csv_to_dict(OFFSET_PATH)
        if old_d is None:
            old_d = {}
        for name, jobj in self.jointHandlerDic.items():
            old = old_d.get(name)
            if old is None:
                old = 0.0
            if np.isclose(old, jobj.offset):
                continue
            update_csv(OFFSET_PATH, name, jobj.offset)
            user_disp += f"{name}: {old:.4f}->{jobj.offset:.4f}\n"
        self.pinfo(f"{OFFSET_PATH} updated \n{user_disp}")

    def load_offset(self, off: Optional[Dict[str, float]] = None):
        if off is None:  # just in case we wanna load a custom offset
            off = csv_to_dict(OFFSET_PATH)
        if off is None:
            self.pwarn(f"Cannot load offsets, {OFFSET_PATH} does not exist")
            return
        unknown_names: List[str] = []
        for name, off_value in off.items():
            handler = self.jointHandlerDic.get(name)
            if handler is None:
                unknown_names.append(name)
                continue

            if np.isnan(off_value):
                handler.offset = 0.0
            else:
                handler.offset += off_value
        if unknown_names:
            self.pwarn(
                f"Unknown joints when loading offsets: {list_cyanize(unknown_names)}"
            )

    def set_offsetSRVCBK(
        self, req: SendJointState.Request, res: SendJointState.Response
    ) -> SendJointState.Response:
        # TEST
        # req.js.name = ["leg1base_link-link2"]
        # req.js.position = [0.1]
        if len(req.js.name) < 1:
            return res
        h = list(self.jointHandlerDic.values())
        valid_names = [x._corrected_name for x in h]
        urdf_names = [x.name for x in h]
        for ind in range(len(req.js.name)):
            if req.js.name[ind] in valid_names:
                urdf_ind = valid_names.index(req.js.name[ind])
                req.js.name[ind] = urdf_names[urdf_ind]
        js = req.js
        unknown_names: List[str] = []
        res.success = True
        res.message = ""
        for ind, name in enumerate(js.name):
            handler = self.jointHandlerDic.get(name)
            if handler is None:
                res.success = False
                unknown_names.append(name)
                continue
            if ind >= len(js.position):
                res.message += "Name and position array, different length. "
                res.success = False
                continue

            pos = js.position[ind]
            if np.isnan(pos):
                handler.offset = 0.0
            else:
                handler.offset += pos

        if unknown_names:
            as_string = ", ".join(unknown_names)
            res.message += f"Joints unknown: {as_string}"
        self.save_current_offset()
        return res

    def defined_undefined(self) -> Tuple[List[str], List[str]]:
        undefined: List[str] = []
        defined: List[str] = []
        for name, jobj in self.jointHandlerDic.items():
            if jobj.stateSensor.position is None:
                undefined.append(name)
            else:
                defined.append(name)
        return undefined, defined

    def angle_read_checkTMRCBK(self):
        less_than_1s = self.getNow() - self.node_start < Duration(seconds=1)
        expired = not less_than_1s
        undefined, defined = self.defined_undefined()
        i = f"{bcolors.OKGREEN}all{bcolors.ENDC}"
        if undefined:
            if less_than_1s:
                return  # waits 1 seconds before printing if there are missing angles
            for jobj in [self.jointHandlerDic[name] for name in undefined]:
                jobj.speakup_when_angle()
            self.pwarn(
                f"No angle readings yet on {list_cyanize(undefined)}. "
                f"Might not be published."
            )
            i = "some"
        if defined:
            self.pinfo(
                f"{bcolors.OKGREEN}Angle recieved{bcolors.ENDC} on {i} "
                f"joints {list_cyanize(defined)}"
            )

        if expired or (not undefined):
            self.destroy_timer(self.angle_read_checkTMR)

    @error_catcher
    def firstSpinCBK(self):
        self.iAmAlive = self.create_service(Empty, "joint_alive", lambda i, o: o)
        self.destroy_timer(self.firstSpin)
        self.node_start: Time = self.getNow()

        # send empty command to initialize (notabily Rviz interface)
        empty = JointState(name=self.joint_names)
        empty.header.stamp = self.getNow().to_msg()
        self.jsPUB_to_lvl0.publish(empty)

        self.load_offset()

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
            f"{list_cyanize(list(self.pubREMAP.keys()))} onto topics: "
            f"{list_cyanize([self.rem.remap_topic_com[k] for k in self.pubREMAP])}"
        )
        co = list(set(list(self.rem.remap_com.keys())) & set(self.joint_names))
        self.pinfo(
            f"Remapping names of {self.jsPUB_to_lvl0.topic_name} (motor) from joint: "
            f"{list_cyanize(co)} onto state name: "
            f"{list_cyanize([self.rem.remap_com[k] for k in co])}"
        )

        co = list(set(list(self.rem.remap_sens.values())) & set(self.joint_names))
        inverted_dict = {v: k for k, v in self.rem.remap_sens.items()}
        self.pinfo(
            f"Remapping names of {self.js_from_lvl0SUB.topic_name} (sensor) from state name: "
            f"{list_cyanize([inverted_dict[k] for k in co])} onto joint: "
            f"{list_cyanize(co)}"
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

    def _push_commands(self, states: List[JState]) -> None:
        for js in states:
            if js.name is None:
                continue
            handler = self.jointHandlerDic.get(js.name)
            if handler is None:
                continue
            handler.update_js_command(js)

    def _push_sensors(self, states: List[JState]) -> None:
        for js in states:
            if js.name is None:
                continue
            handler = self.jointHandlerDic.get(js.name)
            if handler is None:
                continue
            handler.setJSSensor(js)

    @error_catcher
    def SensorJSRecieved(self, js_reading: JointState) -> None:
        return
        jointsHandled: List[str] = list(self.jointHandlerDic.keys())
        js_reading = intersect_names(js_reading, jointsHandled)
        self.remap_JointState_sens(js_reading)
        if js_reading.header.stamp is None:
            js_reading.header.stamp = self.getNow()

        states = js_from_ros(js_reading)
        self._push_sensors(states)

    def _pull_sensors(self, reset=True) -> List[JState]:
        allStates: List[JState] = []

        for handler in self.jointHandlerL:
            state: JState = handler.get_fresh_sensor(reset)
            allEmpty: bool = (
                (state.velocity is None)
                and (state.position is None)
                and (state.effort is None)
            )
            if not allEmpty:
                allStates.append(state)

        return allStates

    def _pull_commands(self) -> List[JState]:
        allStates: List[JState] = []

        for jointMiniNode in self.jointHandlerL:
            state: JState = jointMiniNode.get_freshCommand()
            allEmpty: bool = (
                (state.velocity is None)
                and (state.position is None)
                and (state.effort is None)
            )
            if allEmpty:
                continue
            allStates.append(state)

        return allStates

    def pub_current_jointstates(self, time_stamp: Optional[Time] = None):
        return
        if time_stamp is None:
            now: Time = self.get_clock().now()
        else:
            now: Time = time_stamp
        time_now_stamp = now.to_msg()

        allStates: List[JState] = self._pull_commands()
        self.send_pure(allStates)
        ordinatedJointStates: List[JointState] = stateOrderinator3000(allStates)
        for jsMSG in ordinatedJointStates:
            jsMSG.header.stamp = time_now_stamp
            self.remap_JointState_com(jsMSG)
            self.jsPUB_to_lvl0.publish(jsMSG)
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

        if not self.dont_handle_body:
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
        is_moving_because_speed = np.any(
            [
                not np.isclose(j.stateCommand.velocity, 0)
                for j in self.jointHandlerDic.values()
                if j.stateCommand.velocity is not None
            ]
        )
        if is_moving_because_speed:
            return
        if self.eco_timer.is_canceled():
            # self.pwarn("eco mode")
            self.refresh_timer.cancel()
            self.eco_timer.reset()

    @error_catcher
    def joint_setCBK(self, js: JointState):
        if js.header.stamp is None:
            js.header.stamp = self.getNow().to_msg()
        if Time.from_msg(js.header.stamp).nanoseconds == 0:
            js.header.stamp = self.getNow().to_msg()
        self._push_commands(js_from_ros(js))

    @error_catcher
    def go_zero_allCBK(self, req: EmptySrv.Request, resp: EmptySrv.Response):
        for j in self.jointHandlerDic.values():
            j.resetAnglesAtZero()
        return resp


def main(args=None):
    myMain(JointNode)


if __name__ == "__main__":
    main()
