"""
Python API to sync the movement of several joints.

Note:
    The ros2 implementation is available in :py:mod:`.ros2.joint_api`.

This high level API alows for multi-joint control and syncronization (over several legs). This is the base class where, receiving and sending data to motion stack lvl1 is left to be implemented.
"""

import copy
import warnings
from abc import ABC, abstractmethod
from typing import Awaitable, Callable, Dict, List, Optional, Set, Tuple, Type, Union

import nptyping as nt
import numpy as np
from nptyping import NDArray, Shape

from ..core.utils.hypersphere_clamp import clamp_to_sqewed_hs
from ..core.utils.joint_state import JState

# [Preliminary] flag to provide Yamcs logging. Can be changed to reading an environment variable?
DEBUG_PRINT = False
YAMCS_LOGGING = False

#: placeholder type for a Future (ROS2 Future, asyncio or concurrent)
FutureType = Awaitable


class SensorSyncWarning(Warning):
    pass


class JointSyncer(ABC):
    r"""One instance controls and syncronises several joints, safely executing trajectory to targets.

    Note:
        This class is an abstract base class, the ros2 implementation is available in :py:class:`.ros2.joint_api.JointSyncerRos`. Hence,  parts of this class are left to be implmented by the interface/runtime: \ :py:meth:`.JointSyncer.FutureT`, :py:meth:`.JointSyncer.sensor`, :py:meth:`.JointSyncer.send_to_lvl2`.

    Important:
        \ :py:meth:`.JointSyncer.execute` must be called to compute, update and send the command.

        One object instance can only execute one target at a time. However, the limbs or targets can change between calls of the same instance, before the previous task is done.

    The trajectory interpolates between two points:

        - The last position (if None: uses sensor, else: last sub-target). This is handled automatically, however ``clear`` resets the last position to None.
        - The input target.

    Several interpolation strategies to reach the target are available:

     - LERP: :py:meth:`.JointSyncer.lerp`
     - ASAP: :py:meth:`.JointSyncer.asap`
     - Unsafe: :py:meth:`.JointSyncer.unsafe`
     - Speed: :py:meth:`.JointSyncer.speed`

    Args:
        interpolation_delta: (rad) During movement, how much error is allowed from the path. if exceeded, movement slows down.
        on_target_delta: (rad) Delta at which the trajectory/task is considered finished and the Future is switched to ``done``.
    """

    _COMMAND_DONE_DELTA: float = np.deg2rad(0.01)

    def __init__(
        self,
        interpolation_delta: float = np.deg2rad(5),
        on_target_delta: float = np.deg2rad(4),
    ) -> None:
        self._interpolation_delta: float = interpolation_delta
        self._on_target_delta: float = on_target_delta
        #: Future of the latest task/trajectory that was run.
        self.last_future = self.FutureT()
        #: When true, the command messages sill stop being published when the sensor data is on target. When false, it stops after sending the target command once. True is improves reliability at the expense of more messages sent, better for lossy networks.
        self.SEND_UNTIL_DONE = True

        self._previous: Dict[str, float] = {}
        self._last_valid: Dict[str, float] = {}
        self._trajectory_task = lambda *_: None

        global YAMCS_LOGGING
        if YAMCS_LOGGING:
            try:
                from ygw_client import YGWClient, get_operator
                self.ygw_client = YGWClient(host="localhost", port=7901)  # one port per ygw client. See yamcs-moonshot/ygw-leg/config.yaml
                self.operator = get_operator()
            except Exception as e:
                warnings.warn(
                    f"Failed to connect to YGW client: {e}. "
                    "Yamcs logging will be disabled for this JointSyncer instance",
                    UserWarning,
                )
                self.ygw_client = None
                YAMCS_LOGGING = False
                
        if DEBUG_PRINT:
            print("===============")
            print("JointSyncer initialized")
            print(f"OPERATOR: {self.operator}")
            # counters to reduce debug printing frequency
            self.DECIMATION_FACTOR = 1
            self.ptime_to_lvl1 = 0
            self.ptime_make_motion = 0
            self.ptime_sensor = 0
                            
    def execute(self):
        """Executes one step of the task/trajectory.

        This must be called frequently."""
        self._trajectory_task()

    def clear(self):
        """Resets the trajectory starting point onto the current sensor positions.

        Important:
            Use when:
                - The trajectory is stuck unable to interpolate.
                - External motion happened, thus the last position used by the syncer is no longer valid.


        Caution:
            At task creation, if the syncer applies `clear()` automatically, `SensorSyncWarning` is issued. Raise the warning as an error to interupt operation if needed.
        """
        self._previous = {}
        self._last_valid = {}

    def lerp(self, target: Dict[str, float]) -> FutureType:
        """Starts executing a lerp trajectory toward the target.

        LERP: all joints reach the target at the same time.

        Args:
            target: ``key`` = joint name ; ``value`` = joint angle

        Returns:
            Future of the task. Done when sensors are on target.
        """
        return self._make_motion(target, self.lerp_toward)

    def asap(self, target: Dict[str, float]) -> FutureType:
        """Starts executing a asap trajectory toward the target.

        ASAP: joints will reach their tagets indepently, as fast as possible

        Args:
            target: ``key`` = joint name ; ``value`` = joint angle

        Returns:
            Future of the task. Done when sensorare on target.
        """
        return self._make_motion(target, self.asap_toward)

    def unsafe(self, target: Dict[str, float]) -> FutureType:
        """Starts executing a unsafe trajectory toward the target.

        Unsafe: Similar to ASAP except the final target is sent directly to the motor, so the movement will not stop in case of crash, errors, network issue AND you cannot cancel it.

        Args:
            target: ``key`` = joint name ; ``value`` = joint angle

        Returns:
            Future of the task. Done when sensorare on target.
        """
        return self._make_motion(target, self.unsafe_toward)

    def speed_safe(
        self, target: Dict[str, float], delta_time: Union[float, Callable[[], float]]
    ) -> FutureType:
        """Starts executing a speed safe trajectory at the target speeds.

        Speed Safe: Moves the joints at a given set speed and keeps them in sync positon-wise.

        Warning:
            This method is in early developpement and hasn't been thouroughly tested.

        Note:
            This sends position commands and not speed commands. This is to avoid dangerous joint runaway if issue arises.

        Args:
            target: key = joint name ; value = joint speed
            delta_time: Function giving the elapsed time in seconds (float) since the last time it was called. A constant float value can also be used but it is not recommanded.

        Returns:
            Future of the task. This future will never be done unless when canceled.

        """
        if not callable(delta_time):
            delta_time = lambda x=delta_time: x

        track = set(target.keys())
        start = self._previous_point(track)

        def speed(target) -> bool:
            dt = delta_time()
            offset = {jname: start[jname] + target[jname] * dt for jname in track}
            self.lerp_toward(offset)
            return False

        return self._make_motion(target, speed)

    def ready(self, joints: Union[Set, Dict]) -> Tuple[bool, Set]:
        """Returns wether a movement using those joints is possible or not.

        Args:
            joints: Joints that one wants to use for a movement

        Returns:
            - True if movement is possible
            - Missing joints

        """
        if isinstance(joints, Dict):
            joints = set(joints.keys())
        joints_available = set(only_position(self._sensor).keys())
        missing = joints - joints_available
        return len(missing) == 0, missing

    def abs_from_rel(self, offset: Dict[str, float]) -> Dict[str, float]:
        """Absolute position of the joints that correspond to the given relative offset.

        Example:
            *joint_1* is at 45 deg, offset is 20 deg. Return will be 65 deg.

        Args:
            offset: Relative positions.

        Returns:
            Absolute position.
        """
        track = set(offset.keys())
        prev = self._previous_point(track)
        return {name: prev[name] + offset[name] for name in track}

    ## [Temporary] Dummy print function as placeholder to YGW logging
    def dummy_print_jstate(self, data:List[JState], prefix: str = ""):
        str_to_send: List[str] = [f"High : "]
        for joint_state in data:
            if joint_state.position is None:
                continue  # skips empty states with no angles
            str_to_send.append(
                f"{prefix} {joint_state.name} "
                f"| {np.rad2deg(joint_state.position):.1f}"
            )
        print("\n".join(str_to_send))
        
    def dummy_print_target(self, data:Dict[str, float], prefix: str = ""):
        str_to_send: List[str] = [f"High : "]
        for joint_name, joint_angle in data.items():
            str_to_send.append(
                f"{prefix} {joint_name} "
                f"| {np.rad2deg(joint_angle):.1f}"
            )
        print("\n".join(str_to_send))
        
    def dummy_print_sensor(self, data:Dict[str, JState], prefix: str = ""):
        str_to_send: List[str] = [f"High : "]
        for joint_name, jstate in data.items():
                str_to_send.append(
                    f"{prefix} {joint_name} "
                    f"| {np.rad2deg(jstate.position):.1f}"
                )
        print("\n".join(str_to_send))
    
    def _send_to_lvl1(self, states: List[JState]):
        self.send_to_lvl1(states)
        
        if YAMCS_LOGGING:
            self.ygw_client.publish_jstates(
                name="joint_syncer_send_to_lvl1_states",
                states=states,
            )
        if DEBUG_PRINT:
            self.ptime_to_lvl1 += 1
            if self.ptime_to_lvl1 % (self.DECIMATION_FACTOR * 100) == 0:
                self.dummy_print_jstate(states, prefix="send: high -> lvl1:")
                
    @abstractmethod
    def send_to_lvl1(self, states: List[JState]):
        """Sends motor command data to lvl1.

        Important:
            This method must be implemented by the runtime/interface.

        Note:
            Default ROS2 implementation: :py:meth:`.ros2.joint_api.JointSyncerRos.send_to_lvl1`

        Args:
            states: Joint state data to be sent to lvl1
        """
        pass

    @property
    @abstractmethod
    def FutureT(self) -> Type[FutureType]:
        """Class of Future to use: ROS2 Future, asyncio or concurrent.

        Important:
            This method must be implemented by the runtime/interface.

        Default ROS2 implementation::
            return rclpy.task.Future

        Returns:
            The Future class (not an instance).
        """
        ...

    @property
    def _sensor(self) -> Dict[str, JState]:
        sensor_values = self.sensor  # type: Dict[str, JState]
        
        if YAMCS_LOGGING:
            self.ygw_client.publish_dict(
                group="joint_syncer_sensor_values",
                data=sensor_values,
                operator=self.operator
            )    
        if DEBUG_PRINT:
            self.ptime_sensor += 1
            if self.ptime_sensor % (self.DECIMATION_FACTOR * 100) == 0:
                self.dummy_print_sensor(sensor_values, prefix="sensor: 100x lvl1 -> high:")

        return sensor_values
    
    @property
    @abstractmethod
    def sensor(self) -> Dict[str, JState]:
        """Is called when sensor data is need.

        Important:
            This method must be implemented by the runtime/interface.

        Note:
            Default ROS2 implementation: :py:meth:`.ros2.joint_api.JointSyncerRos.sensor`

        Returns:

        """
        ...

    def _previous_point(self, track: Set[str]) -> Dict[str, float]:
        """
        Args:
            track: Joints to consider.

        Returns:
            Previous point in the trajectory. If missing data, uses sensor.

        """
        not_included = track - set(self._previous.keys())
        if not not_included:
            return self._previous
        sensor = only_position(self._sensor)
        available = set(sensor.keys())
        for name in not_included & available:
            self._previous[name] = sensor[name]
        return self._previous

    def _update_previous_point(self, data: Dict[str, float]) -> None:
        self._previous.update(data)
        return

    def _get_last_valid(self, track: Set[str]) -> Dict[str, float]:
        """
        Args:
            track: Joints to consider.

        Returns:
            Previous point in the trajectory. If missing data, uses sensor.

        """
        missing = track - set(self._last_valid.keys())
        if not missing:
            return self._last_valid
        sensor = self._sensor
        available = set(sensor.keys())
        for name in missing & available:
            val = sensor[name].position
            if val is None:
                continue
            self._last_valid[name] = val
        return self._last_valid

    def _set_last_valid(self, data: Dict[str, float]) -> None:
        data = copy.deepcopy(data)
        self._last_valid.update(data)
        return

    def _previous_and_sensor(
        self, track: Set[str]
    ) -> Tuple[Dict[str, float], Dict[str, float]]:
        possible, missing = self.ready(track)
        assert (
            possible
        ), f"Sensor has no data about the following joints: {missing}. Unable to interpolate. Available joints are: {set(self._sensor.keys())}."

        previous = self._previous_point(track)
        assert (
            set(previous.keys()) >= track
        ), f"Previous step does not have required joint data"

        center = only_position(self._sensor)
        return previous, center

    def _get_lerp_step(
        self, start: Dict[str, float], target: Dict[str, float]
    ) -> Tuple[Dict[str, float], bool]:
        """Result of a single step of lerp."""
        track = set(target.keys())
        order = list(target.keys())

        previous, center = self._previous_and_sensor(track)
        previous = start

        clamped = clamp_to_sqewed_hs(
            start=_order_dict2arr(order, previous),
            center=_order_dict2arr(order, center),
            end=_order_dict2arr(order, target),
            radii=np.full((len(order),), self._interpolation_delta),
        )
        validity = not bool(np.any(np.isnan(clamped)))
        return dict(zip(order, clamped)), validity

    def _get_asap_step(
        self, start: Dict[str, float], target: Dict[str, float]
    ) -> Tuple[Dict[str, float], bool]:
        """Result of a single step of asap."""
        track = set(target.keys())
        order = list(target.keys())

        previous, center = self._previous_and_sensor(track)

        start_arr = _order_dict2arr(order, previous)
        center_arr = _order_dict2arr(order, center)
        end_arr = _order_dict2arr(order, target)

        clamped = end_arr

        # clamped = np.clip(
        #     a=clamped,
        #     a_max=start_arr + self.INTERPOLATION_DELTA,
        #     a_min=start_arr - self.INTERPOLATION_DELTA,
        # )
        #
        clamped = np.clip(
            a=clamped,
            a_max=center_arr + self._interpolation_delta,
            a_min=center_arr - self._interpolation_delta,
        )

        return dict(zip(order, clamped)), True

    def _get_unsafe_step(
        self, start: Dict[str, float], target: Dict[str, float]
    ) -> Tuple[Dict[str, float], bool]:
        """Result of a single step of unsafe."""
        return target, True

    def _traj_toward(
        self,
        target: Dict[str, float],
        step_func: Callable[
            [Dict[str, float], Dict[str, float]], Tuple[Dict[str, float], bool]
        ],
        start: Optional[Dict[str, float]] = None,
    ) -> bool:
        """Executes a step of the given step function."""
        order = list(target.keys())
        target_ar = _order_dict2arr(order, target)
        prev = self._previous_point(set(order))
        prev_ar = _order_dict2arr(order, prev)

        if len(target) == 0:
            return True
        if start is None:
            start = prev

        is_on_final = bool(
            np.linalg.norm(target_ar - prev_ar) < self._COMMAND_DONE_DELTA
        )

        if not is_on_final:
            next, valid = step_func(start, target)
            if valid:
                self._set_last_valid(next)
            else:
                sens = only_position(self._sensor)
                next, valid = step_func(sens, self._get_last_valid(set(order)))
        else:
            next = target

        if self.SEND_UNTIL_DONE:
            self.send_to_lvl1(
                [JState(name, position=pos) for name, pos in next.items()]
            )
            self._update_previous_point(next)

        if not is_on_final:
            return False

        s = _order_dict2arr(order, only_position(self._sensor))
        on_target = bool(
            np.linalg.norm(target_ar - s, ord=np.inf) < self._on_target_delta
        )
        return on_target

    def unsafe_toward(self, target: Dict[str, float]) -> bool:
        """Executes one single unsafe step.

        Args:
            target: key = joint name ; value = joint angle

        Returns:
            True if trajectory finished
        """
        return self._traj_toward(target, self._get_unsafe_step)

    def asap_toward(self, target: Dict[str, float]) -> bool:
        """Executes one single asap step.

        Args:
            target: key = joint name ; value = joint angle

        Returns:
            True if trajectory finished
        """
        return self._traj_toward(target, self._get_asap_step)

    def lerp_toward(self, target: Dict[str, float]) -> bool:
        """Executes one single lerp step.

        Args:
            target: key = joint name ; value = joint angle

        Returns:
            True if trajectory finished
        """
        return self._traj_toward(target, self._get_lerp_step)

    def _make_motion(
        self, target: Dict[str, float], toward_func: Callable[[Dict[str, float]], bool]
    ) -> FutureType:
        """Makes the trajectory task using the toward function.

        Args:
            target: key = joint name ; value = joint angle
            toward_func: Function executing a step toward the target.

        Returns:
            Future of the task. Done when sensors are on target.
        """
        
        if YAMCS_LOGGING:
            self.ygw_client.publish_dict(
                group="joint_syncer_make_motion_target",
                data=target,
                operator=self.operator,  
            )
        if DEBUG_PRINT:
            self.ptime_make_motion += 1
            if self.ptime_make_motion % self.DECIMATION_FACTOR == 0:
                self.dummy_print_target(target, prefix="_make_motion: high -> lvl1:")


        tracked = set(target.keys())
        order = list(target.keys())
        prev, center = self._previous_and_sensor(tracked)
        prev_arr = _order_dict2arr(order, prev)
        sens_arr = _order_dict2arr(order, center)
        has_moved_since_last_time = bool(
            # np.linalg.norm(targ_arr - sens_arr, ord=np.inf) < self._interpolation_delta
            np.linalg.norm(prev_arr - sens_arr, ord=np.inf)
            < self._interpolation_delta
        )

        if not has_moved_since_last_time:
            warnings.warn(
                "Syncer is out of sync with sensor data (something else than this syncer might have moved the joints). `syncer.clear()` will be called automatically, thus the trajectory will resstart from the current sensor position. Raise this warning as an error to interupt operations.",
                SensorSyncWarning,
            )
            self.clear()
            tracked = set(target.keys())
            order = list(target.keys())
            prev, center = self._previous_and_sensor(tracked)
            prev_arr = _order_dict2arr(order, prev)
            sens_arr = _order_dict2arr(order, center)

        future = self.FutureT()
        self.last_future.cancel()
        if len(target) == 0:
            future.set_result(True)
            return future


        stop = [False]

        def step_toward_target(future=future, stop=stop):
            if stop[0]:
                return
            if future.cancelled():
                stop[0] = True
                return
            if future.done():
                stop[0] = True
                return

            move_done = toward_func(target)
            if move_done:
                future.set_result(move_done)

        self._trajectory_task = step_toward_target
        self.last_future = future
        # self.execute()
        return future

    def __del__(self):
        self.last_future.cancel()


def only_position(js_dict: Union[Dict[str, JState], List[JState]]) -> Dict[str, float]:
    """Extract positions from a dict or list of JState. None is ignored"""
    if isinstance(js_dict, list):
        return {js.name: js.position for js in js_dict if js.position is not None}
    else:
        return {n: js.position for n, js in js_dict.items() if js.position is not None}


def _order_dict2arr(
    order: List[str], data: Dict[str, float]
) -> NDArray[Shape["N"], nt.Floating]:
    """return array given the order"""
    return np.array([data[n] for n in order])
