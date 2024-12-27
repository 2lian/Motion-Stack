"""Node and its object of level 1

.. author::
    Elian NEPPEL

"""

from typing import Dict, Final, Iterable, List, Optional, Tuple

import numpy as np
import quaternion as qt
from easy_robot_control.utils.state_remaper import StateRemapper, empty_remapper
from roboticstoolbox.tools.urdf.urdf import Joint as RTBJoint

from motion_stack.core.utils.printing import TCOL
from motion_stack.core.utils.time import NANOSEC, Time

from .utils.joint_state import JState, impose_state, jattr, jdata, js_changed, jstamp
from .utils.robot_parsing import get_limit
from .utils.static_executor import FlexNode

P_GAIN = 3.5
D_GAIN = 0.00005  # can be improved
INIT_AT_ZERO = False  # dangerous
CLOSE_ENOUGH = np.deg2rad(0.25)
LATE = 0.3

EXIT_CODE_TEST = {
    0: "OK",
    1: "TESTS_FAILED",
    2: "INTERRUPTED",
    3: "INTERNAL_ERROR",
    4: "USAGE_ERROR",
    5: "NO_TESTS_COLLECTED",
}

ECO_MODE_PERIOD: float = 0.5  # seconds

TOL_NO_CHANGE: Final[JState] = JState(
    name="",
    time=Time(ECO_MODE_PERIOD * NANOSEC),
    position=np.deg2rad(0.1),
    velocity=np.deg2rad(0.1),
    effort=np.deg2rad(0.1),
)


class JointHandler:
    """This handles a single joint.
    The main purpose is to update stateSensor and stateCommand. As well as getting the
    newest values for those (in order to not continuously publish unchanging data).
    """

    name: str
    parent: "JointNode"
    joint_object: RTBJoint
    upper: float = np.inf
    lower: float = -np.inf
    ignore_limits: bool = False

    command: JState
    fresh_command: JState
    sensor: JState
    fresh_sensor: JState

    def __init__(
        self,
        name: str,
        parent_node: "JointNode",
        joint_object: RTBJoint,
        IGNORE_LIM: bool = False,
        MARGIN: float = 0.0,
    ):
        self.name = name
        self.parent = parent_node
        self.joint_object = joint_object
        self._smode = self.parent.SPEED_MODE  #: to place in an injection
        self.IGNORE_LIM = IGNORE_LIM
        self.MARGIN = MARGIN
        self.load_limit()

        self.command = JState(name=self.name)
        self.fresh_command = self.command
        self.sensor = JState(name=self.name)
        self.fresh_sensor = self.sensor

        self.critical_check()

    def load_limit(self):
        """Loads the limit from the (urdf) joint object

        Args:
            ignore: if limits should be ignored
        """
        if self.ignore_limits:
            self.lower: float = -np.inf
            self.upper: float = np.inf
            return
        self.lower, self.upper = get_limit(self.joint_object)

    def critical_check(self):
        """Raises error is clearly wrong data"""
        assert self.name == self.joint_object.name
        assert self.lower <= self.upper

    @property
    def no_limit(self) -> bool:
        """
        Returns: True if the joint has not limits
        """
        return (self.lower, self.upper) == (-np.inf, np.inf)

    @property
    def command_ready(self) -> bool:
        return self.command.is_initialized

    @property
    def sensor_ready(self) -> bool:
        return self.sensor.is_initialized

    def checkAngle(self, angle: Optional[float]) -> bool:
        """True is angle is valid or None"""
        if self.IGNORE_LIM or angle is None:
            return True
        return self.lower <= angle <= self.upper

    def _activeAngleCheckCBK(self):
        """check the sensor angle for validity, changes the speed if invalid"""
        no_problem = self.checkAngle(self.sensor.position)
        if no_problem:
            return
        stop = self.sensor.copy()
        stop.time = self.parent.now()
        stop.velocity = 0
        stop.effort = 0
        self.update_js_command(stop)

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
        js = js.copy()
        if js.time is not None:
            js.time = js.time
        else:
            js.time = self.parent.now()

        if js.position is not None:
            js.position = self.process_angle_command(js.position)
        if js.velocity is not None:
            v = self.process_velocity_command(js.velocity)
            emergency = v is None
            if emergency and self.sensor.position is not None:
                js.position = self.process_angle_command(self.sensor.position)
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

        self.command = impose_state(self.command, js)
        self.fresh_command = impose_state(self.fresh_command, js)

    def is_new_jssensor(self, js: JState):
        """True if js is different enough from the last received.
        Also true if stateSensor is more the TOL_NO_CHANGE.time old relative to the new
        """
        d = TOL_NO_CHANGE.copy()
        if self.sensor.time is not None and TOL_NO_CHANGE.time is not None:
            # black magic to keep publishing in sync when no changes
            # We basically refresh every t=N*dt, and not dt after the previous
            ts = self.sensor.time
            dt = TOL_NO_CHANGE.time
            d.time = Time(dt - ts % dt)
        something_changed = js_changed(js, self.sensor, delta=d)
        return something_changed

    def setJSSensor(self, js: JState):
        """Updates the stateSensor to a new js."""
        assert js.name == self.name

        if not self.is_new_jssensor(js):
            return

        self.sensor = js  # no processing for sensor
        self.fresh_sensor = impose_state(self.fresh_sensor, js)
        self.sensor_updated = True

    def process_angle_command(self, angle: float) -> float:
        """This runs on new js before updating stateCommand"""
        angle_in = angle
        angle, isValid = self.applyAngleLimit(angle)
        if not isValid:
            self.parent.warn(
                f"{TCOL.OKCYAN}{self.name}{TCOL.WARNING} clipped to limit. "
                f"{angle_in:.2f} -> {angle:.2f}"
            )
        return angle

    def _update_angle_cmd(self, angle: float, time: Optional[Time] = None):
        """Updates stateCommand by providing only an angle.
        should be avoided as the timestamp will be set to now.
        """
        assert isinstance(angle, float)
        if time is None:
            time = self.parent.now()

        new_js = JState(self.name)
        new_js.position = angle
        new_js.time = self.parent.now()
        self.update_js_command(new_js)

    def process_velocity_command(self, speed: float) -> Optional[float]:
        """This runs on new js before updating stateCommand"""
        if self.sensor.position is None or self.IGNORE_LIM:
            goingLow = False
            goingHi = False
        else:
            goingLow = self.sensor.position <= self.lower and speed <= 0
            goingHi = self.sensor.position >= self.upper and speed >= 0

        mustStop = goingLow or goingHi
        if mustStop:
            return None
        return speed

    def _update_speed_cmd(self, speed: float, stop_other_commands: bool = False):
        """Updates stateCommand by providing only an speed.
        should be avoided as the timestamp will be set to now.
        """
        assert isinstance(speed, float)

        new_js = JState(self.name)
        new_js.velocity = speed
        new_js.time = self.parent.now()
        if stop_other_commands:
            self.command.position = None
            self.command.effort = None
        else:
            self.update_js_command(new_js)

    def process_effort_command(self, eff: float) -> float:
        """This runs on new js before updating stateCommand"""
        return eff

    def set_effortCBK(self, effort: float):
        """Updates stateCommand by providing only an effort.
        should be avoided as the timestamp will be set to now.
        """
        new_js = JState(self.name)
        new_js.effort = effort
        new_js.time = self.parent.now()
        self.update_js_command(new_js)

    def _angle_feedback(self) -> None:
        """PID updating the speed command based on last stateSensor"""
        if not self._smode:
            return
        if self.command.position is None or self.sensor.position is None:
            return
        delta = self.command.position - self.sensor.position
        small_angle = abs(delta) < CLOSE_ENOUGH
        if small_angle:
            self._update_speed_cmd(0)
            return

        if self.sensor.velocity is None:
            vel = 0
        else:
            vel = self.sensor.velocity

        speedPID = delta * P_GAIN - vel * D_GAIN

        if self.command.time is None or self.sensor.time is None:
            self._update_speed_cmd(speedPID)
            return

        period = Time(1 / self.parent.MVMT_UPDATE_RATE + LATE) * NANOSEC
        arrivalTime = self.command.time + period
        # time left to reach the target
        timeLeft: Time = arrivalTime - self.sensor.time
        # if less than 5% of the time left to reach, we will go slower and not freak out
        # with infinite speed
        timeLeftSafe = max(period * 0.05, timeLeft)
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
        out = self.fresh_sensor.copy()
        if reset:
            self.fresh_sensor = JState(name=self.name)
        return out

    def get_freshCommand(self, reset: bool = True) -> JState:
        """returns command data that is newer than the last time it was called.
        full of None is not newer"""
        self._angle_feedback()
        out = self.fresh_command.copy()
        if reset:
            self.fresh_command = JState(name=self.name)
        return out


class JointNode(FlexNode):
    """Lvl1"""

    #: Remapping around any joint state communication of lvl0
    lvl0_remap: StateRemapper
    #: Remapping around any joint state communication of lvl2
    lvl2_remap: StateRemapper

    def __init__(self):
        # rclpy.init()
        self.lvl0_remap: StateRemapper = empty_remapper
        self.lvl2_remap: StateRemapper = empty_remapper

        self.declare_parameter("leg_number", 0)
        self.leg_num = (
            self.get_parameter("leg_number").get_parameter_value().integer_value
        )
        self.Alias = f"J{self.leg_num}"

        self.current_body_xyz: NDArray = np.array([0, 0, 0], dtype=float)
        self.current_body_quat: qt.quaternion = qt.one
        self.body_xyz_queue = np.zeros((0, 3), dtype=float)
        self.body_quat_queue = qt.from_float_array(np.zeros((0, 4), dtype=float))

        self.wait_for_lower_level(["rviz_interface_alive"], all_requiered=False)

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
        cleanup = set(self.ADD_JOINTS)
        cleanup -= {""}
        self.ADD_JOINTS = list(cleanup)

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
            if handler.no_limit:
                limits_undefined.append(jObj.name)
            self.jointHandlerDic[name] = handler
        if limits_undefined:
            self.pinfo(
                f"{bcolors.WARNING}Undefined limits{bcolors.ENDC} "
                f"in urdf for joint {limits_undefined}"
            )
        else:
            self.pinfo(f"{bcolors.OKBLUE}All URDF limits defined{bcolors.ENDC} ")

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
        self.go_zero_all: Service = self.create_service(
            EmptySrv, "go_zero_all", self.go_zero_allCBK
        )
        self.adveriserSVR: Service = self.create_service(
            ReturnJointState, "advertise_joints", self.advertiserSRVCBK
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
        self.__bodyTMR = self.create_timer(1 / self.CONTROL_RATE, self.__bodyTMRCBK)
        self.firstSpin: Timer = self.create_timer(1 / 100, self.firstSpinCBK)
        self.angle_read_checkTMR = self.create_timer(0.05, self.angle_read_checkTMRCBK)
        self.angle_read_checkTMR.cancel()
        #    /\    #
        #   /  \   #
        # ^ Timer ^

    def send_to_lvl0(self, states: List[JState]):
        """Sends states to lvl0 (commands for motors).
        This function is executed every time data needs to be sent down.
        Change/overload this method with what you need"""
        stamp = self.getNow().to_msg()
        msgs = stateOrderinator3000(states)
        for msg in msgs:
            msg.header.stamp = stamp
            self.jsPUB_to_lvl0.publish(msg)

    def send_to_lvl2(self, states: List[JState]):
        """Sends states to lvl2 (states for ik).
        This function is executed every time data needs to be sent up.
        Change/overload this method with what you need"""
        stamp = self.getNow().to_msg()
        msgs = stateOrderinator3000(states)
        for msg in msgs:
            msg.header.stamp = stamp
            self.jsPUB_to_lvl2.publish(msg)

    @error_catcher
    def js_from_lvl0(self, msg: JointState):
        """Callback when a JointState arrives from the lvl0 (states from motor).
        Converts it into a list of states, then hands it to the general function
        """
        if msg.header.stamp is None:
            msg.header.stamp = self.getNow().to_msg()
        states = js_from_ros(msg)
        self.coming_from_lvl0(states)

    @error_catcher
    def js_from_lvl2(self, msg: JointState):
        """Callback when a JointState arrives from the lvl2 (commands from ik).
        Converts it into a list of states, then hands it to the general function
        """
        if msg.header.stamp is None:
            msg.header.stamp = self.getNow().to_msg()
        states = js_from_ros(msg)
        self.coming_from_lvl2(states)

    def coming_from_lvl2(self, states: List[JState]):
        """Processes incomming commands from lvl2 ik.
        Call this function after processing the ros message"""
        stamp = None
        self.lvl2_remap.unmap(states)
        for s in states:
            if s.time is None:
                stamp = self.getNow() if stamp is None else stamp
                s.time = stamp
        self._push_commands(states)

    def coming_from_lvl0(self, states: List[JState]):
        """Processes incomming sensor states from lvl0 motors.
        Call this function after processing the ros message.
        Always do super().coming_from_lvl0(states) before your code,
        Unless you know what you are doing"""
        stamp = None
        self.lvl0_remap.unmap(states)
        for s in states:
            if s.time is None:
                stamp = self.getNow() if stamp is None else stamp
                s.time = stamp
        self._push_sensors(states)

    @error_catcher
    def _send_sensorTMRCBK(self):
        """pulls and resets fresh sensor data, then sends it to lvl2"""
        states = self._pull_sensors()
        self.lvl2_remap.map(states)
        self.send_to_lvl2(states)

    @error_catcher
    def _send_commandTMRCBK(self):
        """pulls and resets fresh command data, then sends it to lvl0"""
        states = self._pull_commands()
        self.lvl0_remap.map(states)
        self.send_to_lvl0(states)

    def advertiserSRVCBK(
        self, req: ReturnJointState.Request, res: ReturnJointState.Response
    ) -> ReturnJointState.Response:
        """Sends an JointState mainly to advertise the names of the joints"""
        names: List[str] = [h.sensor.name for h in self.jointHandlerDic.values()]
        none2nan = lambda x: x if x is not None else np.nan
        res.js = JointState(
            name=names,
            position=[none2nan(h.sensor.position) for h in self.jointHandlerDic.values()],
            velocity=[none2nan(h.sensor.velocity) for h in self.jointHandlerDic.values()],
            effort=[none2nan(h.sensor.effort) for h in self.jointHandlerDic.values()],
        )

        res.js.header.stamp = self.getNow().to_msg()
        return res

    def defined_undefined(self) -> Tuple[List[str], List[str]]:
        """Return joints with and without poistion data received yet

        Returns:
            Tuple(List[joint names that did not receive any data],
            List[joint names that have data])
        """
        undefined: List[str] = []
        defined: List[str] = []
        for name, jobj in self.jointHandlerDic.items():
            if jobj.sensor.position is None:
                undefined.append(name)
            else:
                defined.append(name)
        return undefined, defined

    @error_catcher
    def angle_read_checkTMRCBK(self):
        """Checks that all joints are receiving data.
        After 1s, if not warns the user, and starts the verbose check on the joint handler.
        """
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

        # we should not start at zero when using real robot
        if INIT_AT_ZERO:
            for jointMiniNode in self.jointHandlerDic.values():
                jointMiniNode.resetAnglesAtZero()
        self.angle_read_checkTMR.reset()

    def _push_commands(self, states: List[JState]) -> None:
        for js in states:
            if js.name is None:
                continue
            handler = self.jointHandlerDic.get(js.name)
            if handler is None:
                continue
            handler.update_js_command(js)

    def _push_sensors(self, states: Iterable[JState]) -> None:
        for js in states:
            if js.name is None:
                continue
            handler = self.jointHandlerDic.get(js.name)
            if handler is None:
                continue
            handler.setJSSensor(js)

    def _pull_sensors(self, reset=True) -> List[JState]:
        allStates: List[JState] = []

        for handler in self.jointHandlerDic.values():
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

        for jointMiniNode in self.jointHandlerDic.values():
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

    def __bodyTMRCBK(self, time_stamp: Optional[Time] = None):
        if time_stamp is None:
            now: Time = self.get_clock().now()
        else:
            now: Time = time_stamp
        time_now_stamp = now.to_msg()

        self.__pop_and_load_body()
        xyz = self.current_body_xyz.copy()
        rot = self.current_body_quat.copy()
        msgTF = np2tf(xyz, rot)

        body_transform = TransformStamped()
        body_transform.header.stamp = time_now_stamp
        body_transform.header.frame_id = "world"
        body_transform.child_frame_id = f"{self.FRAME_PREFIX}{self.baselinkName}"
        body_transform.transform = msgTF

        if not self.dont_handle_body:
            self.tf_broadcaster.sendTransform(body_transform)

        return

    def __pop_and_load_body(self):
        empty = self.body_xyz_queue.shape[0] <= 0
        if not empty:
            self.current_body_xyz = self.body_xyz_queue[0, :]
            self.current_body_quat = self.body_quat_queue[0]
            self.body_xyz_queue = np.delete(self.body_xyz_queue, 0, axis=0)
            self.body_quat_queue = np.delete(self.body_quat_queue, 0, axis=0)

    @error_catcher
    def robot_body_pose_cbk(self, msg: Transform):
        tra, quat = tf2np(msg)
        self.current_body_xyz = tra
        self.current_body_quat = quat

    def smoother(self, x: NDArray) -> NDArray:
        """smoothes the interval [0, 1] to have a soft start and end
        (derivative is zero)
        """
        # x = (1 - np.cos(x * np.pi)) / 2
        # x = (1 - np.cos(x * np.pi)) / 2
        return x

    @error_catcher
    def smooth_body_trans(self, request: Transform):
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
        return

    @error_catcher
    def go_zero_allCBK(self, req: EmptySrv.Request, resp: EmptySrv.Response):
        for j in self.jointHandlerDic.values():
            j.resetAnglesAtZero()
        return resp


def main(args=None):
    myMain(JointNode)


if __name__ == "__main__":
    main()
