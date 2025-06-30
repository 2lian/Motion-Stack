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

        self._previous: Dict[str, float] = {}
        self._last_valid: Dict[str, float] = {}
        self._trajectory_task = lambda *_: None

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
            return self.lerp_toward(offset)

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
        joints_available = set(only_position(self.sensor).keys())
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
        missing = track - set(self._previous.keys())
        if not missing:
            return self._previous
        sensor = only_position(self.sensor)
        available = set(sensor.keys())
        for name in missing & available:
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
        sensor = self.sensor
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

    def _previous_and_center(
        self, track: Set[str]
    ) -> Tuple[Dict[str, float], Dict[str, float]]:
        center = only_position(self.sensor)
        possible, missing = self.ready(track)
        assert (
            possible
        ), f"Sensor does not have required joint data, missing: {missing}."

        previous = self._previous_point(track)
        assert (
            set(previous.keys()) >= track
        ), f"Previous step does not have required joint data"

        return previous, center

    def _get_lerp_step(
        self, start: Dict[str, float], target: Dict[str, float]
    ) -> Tuple[Dict[str, float], bool]:
        """Result of a single step of lerp."""
        track = set(target.keys())
        order = list(target.keys())

        previous, center = self._previous_and_center(track)
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

        previous, center = self._previous_and_center(track)

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
        command_done = bool(
            np.linalg.norm(target_ar - prev_ar) < self._COMMAND_DONE_DELTA
        )

        if start is None:
            start = prev

        if not command_done:
            next, valid = step_func(start, target)
            if valid:
                self._set_last_valid(next)
            else:
                sens = {
                    k: v.position
                    for k, v in self.sensor.items()
                    if v.position is not None
                }
                next, valid = step_func(sens, self._get_last_valid(set(order)))
            self.send_to_lvl1(
                [JState(name, position=pos) for name, pos in next.items()]
            )
            self._update_previous_point(next)

        if not command_done:
            return False

        s = _order_dict2arr(order, only_position(self.sensor))
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
        future = self.FutureT()

        tracked = set(target.keys())
        order = list(target.keys())
        prev, center = self._previous_and_center(tracked)
        target_ar = _order_dict2arr(order, prev)
        prev_ar = _order_dict2arr(order, center)
        if len(target) == 0:
            future.set_result(True)
            return future
        inrange = bool(
            np.linalg.norm(target_ar - prev_ar, ord=np.inf) < self._interpolation_delta
        )
        if not inrange:
            warnings.warn(
                "Syncer is out of sync with sensor data (something else than this syncer might have moved the joints). `syncer.clear()` will be called automatically, thus the trajectory will resstart from the current sensor position. Raise this warning as an error to interupt operations.",
                SensorSyncWarning,
            )
            self.clear()

        def step_toward_target():
            if future.cancelled():
                return
            if future.done():
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
    """Extract velocities from a dict or list of JState. None is ignored"""
    if isinstance(js_dict, list):
        return {js.name: js.position for js in js_dict if js.position is not None}
    else:
        return {n: js.position for n, js in js_dict.items() if js.position is not None}


def _order_dict2arr(
    order: List[str], data: Dict[str, float]
) -> NDArray[Shape["N"], nt.Floating]:
    """return array given the order"""
    return np.array([data[n] for n in order])
