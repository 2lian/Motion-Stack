"""
Node and its object of level 1.
"""

from typing import Callable, Dict, Final, Iterable, List, Optional, Set, Tuple, Union

import numpy as np
from nptyping import NDArray
from roboticstoolbox.tools.urdf.urdf import Joint as RTBJoint

from motion_stack.api.injection.remapper import StateRemapper

from .utils.joint_state import JState, impose_state, jattr, jdata, js_changed, jstamp
from .utils.printing import TCOL, list_cyanize
from .utils.robot_parsing import get_limit, load_set_urdf, load_set_urdf_raw, make_ee
from .utils.static_executor import FlexNode
from .utils.time import NANOSEC, Time


class JointHandler:
    """This handles a single joint.
    The main purpose is to update stateSensor and stateCommand. As well as getting the
    newest values for those (in order to not continuously publish unchanging data).

    Args:
        name: name of the joint (in the URDF)
        parent_node: Parent object handling several joints and messages
        joint_object: raw joint object from rtb, extracted from the URDF
        IGNORE_LIM: If true, joint limits are ignored
        MARGIN: Adds a margin to the joints limits
    """

    _name: str  #: name of the joint (in the URDF)
    _parent: "JointCore"  #: Parent object handling several joints and messages
    _joint_object: RTBJoint
    _upper: float = np.inf
    _lower: float = -np.inf
    _ignore_limits: bool = False

    _command: JState  #: current command state
    _fresh_command: JState  #: fresh command state (old data is replaced by None)
    _sensor: JState  #: current sensor state
    _fresh_sensor: JState  #: fresh sensor state (old data replaced by None)

    _failed_init_advertized: bool = False
    _init_advertized: bool = False

    #: tolerance for two state to be identical. Time is also considered,
    #: so 2 states far from each other in time will be considered different
    #: and trigger an update
    TOL_NO_CHANGE: Final[JState] = JState(
        name="",
        time=Time(sec=1),
        position=np.deg2rad(0.1),
        velocity=np.deg2rad(0.1),
        effort=np.deg2rad(0.1),
    )

    #: if true enable a PID for speed control. Will be deprecated in favor of an injection
    _smode: bool
    PID_P = 3  #: P gain of the PID for speed mode. TO BE DEPRECATED
    PID_D = 0.32  #: D gain of the PID for speed mode. TO BE DEPRECATED
    PID_LATE = 0.0  #: Target will be reached late for smoother motion. TO BE DEPRECATED
    PID_CLOSE_ENOUGH = np.deg2rad(0.1)  #: TO BE DEPRECATED

    def __init__(
        self,
        name: str,
        parent_node: "JointCore",
        joint_object: RTBJoint,
        IGNORE_LIM: bool = False,
        MARGIN: float = 0.0,
    ):
        self._name = name
        self._parent = parent_node
        self._joint_object = joint_object
        self._smode = self._parent.SPEED_MODE  #: to place in an injection
        self.IGNORE_LIM = IGNORE_LIM
        self.MARGIN = MARGIN
        self._load_limit()

        self._command = JState(name=self._name)
        self._fresh_command = self._command
        self._sensor = JState(name=self._name)
        self._fresh_sensor = self._sensor

        self._critical_check()

    @property
    def command(self) -> JState:
        """Current command state"""
        return self._command

    @property
    def sensor(self) -> JState:
        """current sendor state"""
        return self._sensor

    @property
    def name(self) -> str:
        """Name of the joint"""
        return self._name

    def _load_limit(self):
        """Loads the limit from the (urdf) joint object

        Args:
            ignore: if limits should be ignored
        """
        if self._ignore_limits:
            self._lower: float = -np.inf
            self._upper: float = np.inf
            return
        self._lower, self._upper = get_limit(self._joint_object)

    def _critical_check(self):
        """Raises error is clearly wrong data"""
        assert self._name == self._joint_object.name
        assert self._lower <= self._upper

    @property
    def no_limit(self) -> bool:
        """
        True if the joint has not limits
        """
        return (self._lower, self._upper) == (-np.inf, np.inf)

    @property
    def command_ready(self) -> bool:
        """
        True if no commands have been received
        """
        return self._command.is_initialized

    @property
    def sensor_ready(self) -> bool:
        """
        True if no sensor data have been received
        """
        return self._sensor.is_initialized

    def _checkAngle(self, angle: Optional[float]) -> bool:
        """True is angle is valid or None"""
        if self.IGNORE_LIM or angle is None:
            return True
        return self._lower <= angle <= self._upper

    def _activeAngleCheckCBK(self):
        """check the sensor angle for validity, changes the speed if invalid"""
        no_problem = self._checkAngle(self._sensor.position)
        if no_problem:
            return
        stop = self._sensor.copy()
        stop.time = self._parent.now()
        stop.velocity = 0
        stop.effort = 0
        self.set_js_command(stop)

    def _applyAngleLimit(self, angle: float) -> Tuple[float, bool]:
        """Clamps the angle between the joints limits"""
        if self.IGNORE_LIM:
            return angle, True
        out = np.clip(angle, a_min=self._lower, a_max=self._upper)
        return out, out == angle

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
            js.time = self._parent.now()

        if js.position is not None:
            js.position = self._process_angle_command(js.position)
        if js.velocity is not None:
            v = self._process_velocity_command(js.velocity)
            emergency = v is None
            if emergency and self._sensor.position is not None:
                js.position = self._process_angle_command(self._sensor.position)
            if emergency:
                v = 0
            js.velocity = v
        if js.effort is not None:
            js.effort = self._process_effort_command(js.effort)
        return js

    def set_js_command(self, js: JState):
        """Updates the stateCommand to a new js."""
        assert js.name == self._name
        js = self._process_command(js)

        self._command = impose_state(self._command, js)
        self._fresh_command = impose_state(self._fresh_command, js)

    def is_new_jssensor(self, js: JState):
        """True if js is different enough from the last received.
        Also true if stateSensor is more the TOL_NO_CHANGE.time old relative to the new
        """
        d = self.TOL_NO_CHANGE.copy()
        if self._sensor.time is not None and self.TOL_NO_CHANGE.time is not None:
            # black magic to keep publishing in sync when no changes
            # We basically refresh every t=N*dt, and not dt after the previous
            ts = self._sensor.time
            dt = self.TOL_NO_CHANGE.time
            d.time = Time(dt - ts % dt)
        something_changed = js_changed(js, self._sensor, delta=d)
        return something_changed

    def set_js_sensor(self, js: JState):
        """Updates the stateSensor to a new js."""
        assert js.name == self._name

        if not self.is_new_jssensor(js):
            return

        self._sensor = js  # no processing for sensor
        self._fresh_sensor = impose_state(self._fresh_sensor, js)
        self.sensor_updated = True

    def _process_angle_command(self, angle: float) -> float:
        """This runs on new js before updating stateCommand"""
        angle_in = angle
        angle, isValid = self._applyAngleLimit(angle)
        if not isValid:
            self._parent.warn(
                f"{TCOL.OKCYAN}{self._name}{TCOL.WARNING} clipped to limit. "
                f"{angle_in:.2f} -> {angle:.2f}"
            )
        return angle

    def set_angle_cmd(self, angle: float, time: Optional[Time] = None):
        """Updates stateCommand by providing only an angle.
        should be avoided as the timestamp will be set to now.
        """
        assert isinstance(angle, float)
        if time is None:
            time = self._parent.now()

        new_js = JState(self._name)
        new_js.position = angle
        new_js.time = self._parent.now()
        self.set_js_command(new_js)

    def _process_velocity_command(self, speed: float) -> Optional[float]:
        """This runs on new js before updating stateCommand"""
        if self._sensor.position is None or self.IGNORE_LIM:
            goingLow = False
            goingHi = False
        else:
            goingLow = self._sensor.position <= self._lower and speed <= 0
            goingHi = self._sensor.position >= self._upper and speed >= 0

        mustStop = goingLow or goingHi
        if mustStop:
            return None
        return speed

    def set_speed_cmd(self, speed: float, stop_other_commands: bool = False):
        """Updates stateCommand by providing only an speed.
        should be avoided as the timestamp will be set to now.
        """
        if isinstance(speed, int):
            speed = float(speed)
        assert isinstance(speed, float)

        new_js = JState(self._name)
        new_js.velocity = speed
        new_js.time = self._parent.now()
        if stop_other_commands:
            self._command.position = None
            self._command.effort = None
        else:
            self.set_js_command(new_js)

    def _process_effort_command(self, eff: float) -> float:
        """This runs on new js before updating stateCommand"""
        return eff

    def set_effort_cmd(self, effort: float):
        """Updates stateCommand by providing only an effort.
        should be avoided as the timestamp will be set to now.
        """
        new_js = JState(self._name)
        new_js.effort = effort
        new_js.time = self._parent.now()
        self.set_js_command(new_js)

    def _angle_feedback(self) -> None:
        """PID updating the speed command based on last stateSensor"""
        if not self._smode:
            return
        if self._command.position is None or self._sensor.position is None:
            return
        delta = self._command.position - self._sensor.position
        small_angle = abs(delta) < self.PID_CLOSE_ENOUGH
        if small_angle:
            self.set_speed_cmd(0)
            return

        if self._command.velocity is None:
            vel_comm = 0
        else:
            vel_comm = self._command.velocity

        if self._sensor.velocity is None:
            vel_sens = 0
        else:
            vel_sens = self._sensor.velocity

        vel = vel_comm - vel_sens

        speedPID = delta * self.PID_P + vel * self.PID_D

        if self._command.time is None or self._sensor.time is None:
            self.set_speed_cmd(speedPID)
            return

        period = Time(sec=1 / self._parent.MVMT_UPDATE_RATE + self.PID_LATE)
        arrivalTime = self._command.time + period
        # time left to reach the target
        timeLeft: Time = arrivalTime - self._sensor.time
        # if less than 5% of the time left to reach, we will go slower and not freak out
        # with infinite speed
        timeLeftSafe = Time(nano=max(0.05 * period.nano(), timeLeft.nano()))
        perfectSpeed = delta / timeLeftSafe.sec()

        pid_is_slower = abs(speedPID) < abs(perfectSpeed)
        # pick the slower of the two methodes.
        # when far away we move at constant speed to reach the destination on the next
        # command. If close, or when the PID wants to slow down, the PID is used
        speed = speedPID if pid_is_slower else perfectSpeed
        self.set_speed_cmd(speed)
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
        out = self._fresh_sensor.copy()
        if reset:
            self._fresh_sensor = JState(name=self._name)
        return out

    def get_fresh_command(self, reset: bool = True) -> JState:
        """returns command data that is newer than the last time it was called.
        full of None is not newer"""
        self._angle_feedback()
        out = self._fresh_command.copy()
        if reset:
            self._fresh_command = JState(name=self._name)
        return out


class JointCore(FlexNode):
    """Lvl1"""

    #: Remapping around any joint state communication of lvl0. Overwritable
    lvl0_remap: StateRemapper
    #: Remapping around any joint state communication of lvl2. Overwritable
    lvl2_remap: StateRemapper
    #: leg number identifier, deduced from the parameters
    leg_num: int

    send_to_lvl0_callbacks: List[Callable[[List[JState]], None]] = []
    send_to_lvl2_callbacks: List[Callable[[List[JState]], None]] = []

    #: duration after which joints with no sensor data are displayed (warning)
    SENS_VERBOSE_TIMEOUT: int = 1

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.lvl0_remap = StateRemapper()  # empty
        self.lvl2_remap = StateRemapper()  # empty

        # V Params V
        #   \  /   #
        #    \/    #
        self.leg_num = self.ms_param["leg_number"]
        self.spinner.alias = f"J{self.leg_num}"

        self.MOVEMENT_TIME = self.ms_param["std_movement_time"]
        self.CONTROL_RATE = self.ms_param["control_rate"]
        self.MVMT_UPDATE_RATE = self.ms_param["mvmt_update_rate"]
        self.IGNORE_LIM = self.ms_param["ignore_limits"]
        self.MARGIN: float = self.ms_param["limit_margin"]
        self.SPEED_MODE: bool = self.ms_param["speed_mode"]
        self.ADD_JOINTS: List[str] = list(self.ms_param["add_joints"])
        self.urdf_path = self.ms_param["urdf_path"]
        self.urdf_raw = self.ms_param["urdf"]
        self.start_effector: str | None = self.ms_param["start_effector_name"]
        end_effector: str = self.ms_param["end_effector_name"]

        cleanup = set(self.ADD_JOINTS)
        cleanup -= {""}
        self.ADD_JOINTS = list(cleanup)

        if self.start_effector == "":
            self.start_effector = None

        self.end_effector_name = make_ee(end_effector, self.leg_num)

        #    /\    #
        #   /  \   #
        # ^ Params ^
        # self.info(f"chain: {self.start_effector} -> {self.end_effector_name}")
        # self.perror(f"{self.start_effector==self.end_effector_name}")

        # self.end_effector_name = None
        # self.start_effector = None
        (
            self.model,
            self.ETchain,
            self.joint_names,
            self.joints_objects,
            self.last_link,
        ) = load_set_urdf_raw(self.urdf_raw, self.end_effector_name, self.start_effector)
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
                ) = load_set_urdf_raw(
                    self.urdf_raw, self.start_effector, self.end_effector_name
                )
                if len(joint_names2) > len(self.joint_names) and len(
                    joints_objects2
                ) > len(self.joints_objects):
                    joint_names2.reverse()
                    self.joint_names = joint_names2
                    joints_objects2.reverse()
                    self.joints_objects = joints_objects2
        except:
            self.info(f"link tree could not be reversed")
        # self.baselinkName = self.model.base_link.name # base of the whole model
        if self.start_effector is None:
            self.baselinkName = self.model.base_link.name
        else:
            self.baselinkName = self.start_effector

        if self.baselinkName != self.model.base_link.name:
            self.info(
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
            RTBJoint(
                joint_type="continuous",
                parent=None,
                child=None,
                name=jn,
                # limit=[0, 0],
            )
            for jn in self.ADD_JOINTS
        ]

        # self.pinfo(f"Joints controled: {TCOL.OKCYAN}{self.joint_names}{TCOL.ENDC}")

        ee = self.last_link.name if self.last_link is not None else "all joints"
        self.info(
            f"Using base_link: {TCOL.OKCYAN}{self.baselinkName}{TCOL.ENDC}"
            f", to ee:  {TCOL.OKCYAN}{ee}{TCOL.ENDC}"
        )

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
            self.info(
                f"{TCOL.WARNING}Undefined limits{TCOL.ENDC} "
                f"in urdf for joint {limits_undefined}"
            )
        else:
            self.info(f"{TCOL.OKBLUE}All URDF limits defined{TCOL.ENDC} ")

    def send_to_lvl0(self, states: List[JState]):
        """Sends states to lvl0 (commands for motors).
        This function is executed every time data needs to be sent down.

        .. Important::

            Change/overload this method with what you need.

            Or put what you want to execute in self.send_to_lvl0_callbacks
        """
        # self.info(f"sending {states}")
        for f in self.send_to_lvl0_callbacks:
            # self.info(f"sending through {f.__name__}")
            f(states)

    def send_to_lvl2(self, states: List[JState]):
        """Sends states to lvl2 (states for ik).
        This function is executed every time data needs to be sent up.

        .. Important::

            Change/overload this method with what you need.

            Or put what you want to execute in self.send_to_lvl0_callbacks
        """
        # self.info(f"sending {states}")
        for f in self.send_to_lvl2_callbacks:
            # self.info(f"sending through {f.__name__}")
            f(states)

    def coming_from_lvl2(self, states: List[JState]):
        """Processes incomming commands from lvl2 ik.
        Call this function after processing the data into a List[JState]

        .. Caution::

            Overloading this is not advised, but if you do, always do
            super().coming_from_lvl0(states) before your code.
            Unless you know what you are doing
        """
        # self.info(f"received {states}")
        stamp = None
        self.lvl2_remap.unmap(states)
        for s in states:
            if s.time is None:
                stamp = self.now() if stamp is None else stamp
                s.time = stamp
        self._push_commands(states)

    def coming_from_lvl0(self, states: List[JState]):
        """Processes incomming sensor states from lvl0 motors.
        Call this function after processing the data into a List[JState]

        .. Caution::

            Overloading this is not advised, but if you do, always do
            super().coming_from_lvl0(states) before your code.
            Unless you know what you are doing
        """
        stamp = None
        self.lvl0_remap.unmap(states)
        for s in states:
            if s.time is None:
                stamp = self.now() if stamp is None else stamp
                s.time = stamp
        self._push_sensors(states)

    def send_sensor_up(self):
        """pulls and resets fresh sensor data, applies remapping, then sends it to lvl2"""
        states = self._pull_sensors()
        self.lvl2_remap.map(states)
        self.send_to_lvl2(states)

    def send_command_down(self):
        """pulls and resets fresh command data, applies remapping, then sends it to lvl0"""
        states = self._pull_commands()
        self.lvl0_remap.map(states)
        self.send_to_lvl0(states)

    @property
    def _sensors_missing(self) -> Set[str]:
        return {
            name for name, jobj in self.jointHandlerDic.items() if not jobj.sensor_ready
        }

    @property
    def _sensors_ready(self) -> Set[str]:
        return self._all_joint_names - self._sensors_missing

    @property
    def _all_joint_names(self) -> Set[str]:
        return set(self.jointHandlerDic.keys())

    def _advertize_success(self, names: Iterable[str]):
        if not names:
            return
        self.info(
            f"{TCOL.OKGREEN}Angle recieved{TCOL.ENDC} on " f"joints {list_cyanize(names)}"
        )
        if not self._sensors_missing:
            self.info(f"{TCOL.OKGREEN}Angle recieved on ALL joints :){TCOL.ENDC}")
        for n in names:
            jobj = self.jointHandlerDic[n]
            jobj._failed_init_advertized = True
            jobj._init_advertized = True

    def _advertize_failure(self, names: Iterable[str]):
        if not names:
            return
        self.warn(
            f"No angle readings yet on {list_cyanize(names)}. "
            f"Might not be published. :("  # )
        )
        if not self._sensors_missing:
            self.info(f"{TCOL.OKGREEN}Angle recieved on ALL joints :){TCOL.ENDC}")
        for n in names:
            jobj = self.jointHandlerDic[n]
            jobj._failed_init_advertized = True

    def sensor_check_verbose(self) -> bool:
        """Checks that all joints are receiving data.
        After TIMEOUT, if not, warns the user.

        Returns:
            True if all joints have angle data
        """
        less_than_timeout = self.now() - self.startup_time < Time(
            sec=self.SENS_VERBOSE_TIMEOUT
        )
        defined = self._sensors_ready
        undefined = self._sensors_missing
        fail_done = {
            name
            for name, jobj in self.jointHandlerDic.items()
            if jobj._failed_init_advertized
        }
        success_done = {
            name for name, jobj in self.jointHandlerDic.items() if jobj._init_advertized
        }

        succes_to_be_advertized = defined - success_done
        failure_to_be_advertized = undefined - fail_done

        all_are_ready = not bool(undefined)
        if less_than_timeout and not all_are_ready:
            return False

        self._advertize_success(succes_to_be_advertized)
        self._advertize_failure(failure_to_be_advertized)

        return all_are_ready

    def send_empty_command_to_lvl0(self):
        """Sends a command to lvl0 with no data.

        Usefull to initialize lvl0 by giving only the joint names."""
        js: List[JState] = [JState(name=n) for n in self._all_joint_names]
        self.send_to_lvl0(js)

    def _push_commands(self, states: List[JState]) -> None:
        for js in states:
            if js.name is None:
                continue
            joint = self.jointHandlerDic.get(js.name)
            if joint is None:
                continue
            joint.set_js_command(js)

    def _push_sensors(self, states: Iterable[JState]) -> None:
        """Sets sensor state on several joints"""
        for js in states:
            if js.name is None:
                continue
            handler = self.jointHandlerDic.get(js.name)
            if handler is None:
                continue
            handler.set_js_sensor(js)

    def _pull_sensors(self, reset=True) -> List[JState]:
        """Gets fresh sensor state for all joints and resets it"""
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
        """Gets fresh commands  state for all joints and resets it"""
        allStates: List[JState] = []

        for jointMiniNode in self.jointHandlerDic.values():
            state: JState = jointMiniNode.get_fresh_command()
            allEmpty: bool = (
                (state.velocity is None)
                and (state.position is None)
                and (state.effort is None)
            )
            if allEmpty:
                continue
            allStates.append(state)

        return allStates

    def all_go_zero(self):
        """Sends command of angle=0 to all joints"""
        states = [JState(name=n, position=0) for n in self._all_joint_names]
        self.coming_from_lvl2(states)
