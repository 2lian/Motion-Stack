"""
Node and its object of level 1.
"""

from abc import abstractmethod
# import lkjlkjlkj
from typing import Dict, Final, Iterable, List, Optional, Set, Tuple, Union

import numpy as np
import quaternion as qt
from roboticstoolbox.tools.urdf.urdf import Joint as RTBJoint

from .utils.joint_state import JState, impose_state, jattr, jdata, js_changed, jstamp
from .utils.printing import TCOL, list_cyanize
from .utils.robot_parsing import get_limit, load_set_urdf
from .utils.state_remapper import StateRemapper, empty_remapper
from .utils.static_executor import FlexNode
from .utils.time import NANOSEC, Time


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

    _failed_init_advertized: bool = False
    _init_advertized: bool = False

    ECO_MODE_PERIOD: float = 0.5  #: if no change in this interval, no state update

    #: tolerance for state to be identical
    TOL_NO_CHANGE: Final[JState] = JState(
        name="",
        time=Time(sec=ECO_MODE_PERIOD),
        position=np.deg2rad(0.1),
        velocity=np.deg2rad(0.1),
        effort=np.deg2rad(0.1),
    )

    #: if true enable a PID for speed control. Will be deprecated in favor of an injection
    _smode: bool
    PID_P = 3.5  #: P gain of the PID for speed mode. TO BE DEPRECATED
    PID_D = 0.00005  #: D gain of the PID for speed mode. TO BE DEPRECATED
    PID_LATE = 0.3  #: Target will be reached late for smoother motion. TO BE DEPRECATED
    PID_CLOSE_ENOUGH = np.deg2rad(0.25)  #: TO BE DEPRECATED

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
        Returns: 
            True if the joint has not limits
        """
        return (self.lower, self.upper) == (-np.inf, np.inf)

    @property
    def command_ready(self) -> bool:
        """
        Returns:
            True if no commands have been received
        """
        return self.command.is_initialized

    @property
    def sensor_ready(self) -> bool:
        """
        Returns:
            True if no sensor data have been received
        """
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
        d = self.TOL_NO_CHANGE.copy()
        if self.sensor.time is not None and self.TOL_NO_CHANGE.time is not None:
            # black magic to keep publishing in sync when no changes
            # We basically refresh every t=N*dt, and not dt after the previous
            ts = self.sensor.time
            dt = self.TOL_NO_CHANGE.time
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
        small_angle = abs(delta) < self.PID_CLOSE_ENOUGH
        if small_angle:
            self._update_speed_cmd(0)
            return

        if self.sensor.velocity is None:
            vel = 0
        else:
            vel = self.sensor.velocity

        speedPID = delta * self.PID_P - vel * self.PID_D

        if self.command.time is None or self.sensor.time is None:
            self._update_speed_cmd(speedPID)
            return

        period = Time(1 / self.parent.MVMT_UPDATE_RATE + self.PID_LATE) * NANOSEC
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
    leg_num: int
    start_time: Time

    #: duration after which joints with no sensor data are displayed
    SENS_VERBOSE_TIMEOUT: int = 1

    def __init__(self):
        # rclpy.init()
        self.lvl0_remap: StateRemapper = empty_remapper
        self.lvl2_remap: StateRemapper = empty_remapper

        self.leg_num = self.ms_param["leg_number"]
        self.Alias = f"J{self.leg_num}"

        self.start_time = self.now()

        self.info(f"""{TCOL.OKBLUE}Interface connected to motors :){TCOL.ENDC}""")

        # V Params V
        #   \  /   #
        #    \/    #

        self.MOVEMENT_TIME = self.ms_param["std_movement_time"]
        self.FRAME_PREFIX = self.ms_param["frame_prefix"]
        self.CONTROL_RATE = self.ms_param["control_rate"]
        self.MVMT_UPDATE_RATE = self.ms_param["mvmt_update_rate"]
        self.IGNORE_LIM = self.ms_param["ignore_limits"]
        self.MARGIN: float = self.ms_param["limit_margin"]
        self.SPEED_MODE: bool = self.ms_param["speed_mode"]
        self.ADD_JOINTS: List[str] = list(self.ms_param["add_joints"])
        self.urdf_path = self.ms_param["urdf_path"]
        self.start_effector: str | None = self.ms_param["start_effector_name"]
        end_effector: str = self.ms_param["end_effector_name"]

        cleanup = set(self.ADD_JOINTS)
        cleanup -= {""}
        self.ADD_JOINTS = list(cleanup)

        # self.SPEED_MODE: bool = True
        # self.pwarn(self.SPEED_MODE)

        self.START_COORD = np.array(self.ms_param["start_coord"], dtype=float)

        if np.isnan(self.START_COORD).any():
            self.current_body_xyz: NDArray = np.array([0, 0, 0], dtype=float)
            self.dont_handle_body = True
        else:
            self.current_body_xyz: NDArray = self.START_COORD
            self.dont_handle_body = False

        if self.start_effector == "":
            self.start_effector = None

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
        self.info(f"chain: {self.start_effector} -> {self.end_effector_name}")
        # self.perror(f"{self.start_effector==self.end_effector_name}")

        # self.end_effector_name = None
        # self.start_effector = None
        (
            self.model,
            self.ETchain,
            self.joint_names,
            self.joints_objects,
            self.last_link,
        ) = load_set_urdf(self.urdf_path, self.end_effector_name, self.start_effector)
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
                ) = load_set_urdf(
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

    @abstractmethod
    def send_to_lvl0(self, states: List[JState]):
        """Sends states to lvl0 (commands for motors).
        This function is executed every time data needs to be sent down.
        Change/overload this method with what you need

        .. Note::

            This function is left to be implemented by the executor.
        """
        ...

    @abstractmethod
    def send_to_lvl2(self, states: List[JState]):
        """Sends states to lvl2 (states for ik).
        This function is executed every time data needs to be sent up.
        Change/overload this method with what you need

        .. Note::

            This function is left to be implemented by the executor.
        """
        ...

    def coming_from_lvl2(self, states: List[JState]):
        """Processes incomming commands from lvl2 ik.
        Call this function after processing the data into a List[JState]

        .. Caution::

            Overloading this is not advised, but if you do, always do
            super().coming_from_lvl0(states) before your code.
            Unless you know what you are doing
        """
        stamp = None
        self.lvl2_remap.unmap(states)
        for s in states:
            if s.time is None:
                stamp = self.now() if stamp is None else stamp
                s.time = stamp
        self.__push_commands(states)

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
        self.__push_sensors(states)

    def send_sensor_up(self):
        """pulls and resets fresh sensor data, applies remapping, then sends it to lvl2"""
        states = self.__pull_sensors()
        self.lvl2_remap.map(states)
        self.send_to_lvl2(states)

    def send_command_down(self):
        """pulls and resets fresh command data, applies remapping, then sends it to lvl0"""
        states = self.__pull_commands()
        self.lvl0_remap.map(states)
        self.send_to_lvl0(states)

    @property
    def __sensors_missing(self) -> Set[str]:
        return {
            name for name, jobj in self.jointHandlerDic.items() if not jobj.sensor_ready
        }

    @property
    def __sensors_ready(self) -> Set[str]:
        return self.__all_joint_names - self.__sensors_missing

    @property
    def __all_joint_names(self) -> Set[str]:
        return set(self.jointHandlerDic.keys())

    def __advertize_success(self, names: Iterable[str]):
        if not names:
            return
        self.info(
            f"{TCOL.OKGREEN}Angle recieved{TCOL.ENDC} on " f"joints {list_cyanize(names)}"
        )
        if not self.__sensors_missing:
            self.info(f"{TCOL.OKGREEN}Angle recieved on ALL joints :){TCOL.ENDC}")
        for n in names:
            jobj = self.jointHandlerDic[n]
            jobj._failed_init_advertized = True
            jobj._init_advertized = True

    def __advertize_failure(self, names: Iterable[str]):
        if not names:
            return
        self.warn(
            f"No angle readings yet on {list_cyanize(names)}. "
            f"Might not be published. :("  # )
        )
        if not self.__sensors_missing:
            self.info(f"{TCOL.OKGREEN}Angle recieved on ALL joints :){TCOL.ENDC}")
        for n in names:
            jobj = self.jointHandlerDic[n]
            jobj._failed_init_advertized = True

    def sensor_check_verbose(self) -> bool:
        """Checks that all joints are receiving data.
        After 1s, if not, warns the user.

        Returns:
            True if all joints have angle data
        """
        less_than_timeout = self.now() - self.startup_time < Time(
            sec=self.SENS_VERBOSE_TIMEOUT
        )
        defined = self.__sensors_ready
        undefined = self.__sensors_missing
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

        self.__advertize_success(succes_to_be_advertized)
        self.__advertize_failure(failure_to_be_advertized)

        return all_are_ready

    def send_empty_command_to_lvl0(self):
        """Sends a command to lvl0 with no data.

        Usefull to initialize lvl0 by giving only the joint names."""
        js: List[JState] = [JState(name=n) for n in self.__all_joint_names]
        self.send_to_lvl0(js)

    def __push_commands(self, states: List[JState]) -> None:
        for js in states:
            if js.name is None:
                continue
            joint = self.jointHandlerDic.get(js.name)
            if joint is None:
                continue
            joint.update_js_command(js)

    def __push_sensors(self, states: Iterable[JState]) -> None:
        """Sets sensor state on several joints"""
        for js in states:
            if js.name is None:
                continue
            handler = self.jointHandlerDic.get(js.name)
            if handler is None:
                continue
            handler.setJSSensor(js)

    def __pull_sensors(self, reset=True) -> List[JState]:
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

    def __pull_commands(self) -> List[JState]:
        """Gets fresh commands  state for all joints and resets it"""
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

    def all_go_zero(self):
        """Sends command of angle=0 to all joints"""
        states = [JState(name=n, position=0) for n in self.__all_joint_names]
        self.coming_from_lvl2(states)
