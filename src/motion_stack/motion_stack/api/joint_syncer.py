"""
Python API to sync the movement of several joints.

Note:
    The ros2 implementation is available in :py:class:`.ros2.joint_api`.

This high level API alows for multi-joint control and syncronization (over several legs). This is the base class where, receiving and sending data to motion stack lvl1 is left to be implemented.
"""

from abc import ABC, abstractmethod
from typing import Awaitable, Callable, Dict, List, Union

import nptyping as nt
import numpy as np
from nptyping import NDArray, Shape

from ..core.utils.hypersphere_clamp import clamp_to_sqewed_hs
from ..core.utils.joint_state import JState

#: placeholder type for a Future (ROS2 Future, asyncio or concurrent)
FutureType = Awaitable


class JointSyncer(ABC):
    """One instance controls and syncronises several joints, safely executing trajectory to a target.

    Note:
        This class is an abstract base class, the ros2 implementation is available in :py:class:`.ros2.joint_api.JointSyncerRos`.

    The trajectory interpolates between two points:

     - The last position (sensor if None, else, last sub-target). This is handled automatically, however ``clear`` resets it on the current sensor pose.
     - The input target.

    Several interpolation strategies are available:

     - LERP: all joints reach the target at the same time.
     - ASAP: joints will reach their tagets indepently, as fast as possible
     - Unsafe: Similar to ASAP except the final target is sent directly to the motor, so the movement will not stop in case of crash, errors, network issue AND you cannot cancel it.

    One object instance can only execute one trajectory at a time. However, the joints controled can change between calls of the same instance.

    ``execute`` must be called to compute,update and send the command.

    Trajectory tasks return a Future that is 'done' when the sensors are on target.

    This class is an abstractclass, the ros2 implementation is available in :py:class:`.ros2.joint_api.JointSyncerRos`. Hence,  parts of this class are left to be implmented by the interface/runtime:

     - FutureT: Class of Future class to use, ROS2 Future, asyncio or concurrent.
     - sensor: is called when new sensor data is need.
     - send_to_lvl1: is called when command needs to be sent.

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
        self._trajectory_task = lambda *_: None

    def execute(self):
        """Executes one step of the task/trajectory.

        This must be called frequently."""
        self._trajectory_task()

    def lerp(self, target: Dict[str, float]) ->FutureType:
        """Starts executing a lerp trajectory toward the target.

        Args:
            target: key = joint name ; value = joint angle

        Returns:
            Future of the task. Done when sensors are on target.
        """
        return self._make_motion(target, self.lerp_toward)

    def asap(self, target: Dict[str, float]) ->FutureType:
        """Starts executing a asap trajectory toward the target.

        Args:
            target: key = joint name ; value = joint angle

        Returns:
            Future of the task. Done when sensorare on target.
        """
        return self._make_motion(target, self.asap_toward)

    def unsafe(self, target: Dict[str, float]) ->FutureType:
        """Starts executing a unsafe trajectory toward the target.

        Args:
            target: key = joint name ; value = joint angle

        Returns:
            Future of the task. Done when sensorare on target.
        """
        return self._make_motion(target, self.unsafe_toward)

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
    def FutureT(self) -> type[FutureType]:
        """Class of Future to use: ROS2 Future, asyncio or concurrent.

        Important:
            This method must be implemented by the runtime/interface.

        Default ROS2 implementation::
            return rclpy.task.Future

        Returns:
            The Future class (not an instance).
        """
        ...

    def abs_from_offset(self, offset: Dict[str, float]) -> Dict[str, float]:
        """Absolute position of the joints that correspond to the given relative offset.

        Example:
            *joint_1* is at 45 deg, offset is 20 deg. Return will be 65 deg.

        Args:
            offset: Offset dictionary, keys are the joint names, value the offset in rad.

        Returns:
            Absolute position.
        """
        track = set(offset.keys())
        prev = self._previous_point(track)
        return {name: prev[name] + offset[name] for name in track}

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

    def clear(self):
        """Resets the trajectory starting point onto the current sensor positions.

        Mainly usefull when the trajectory is stuck because it could not reach a target.
        """
        self._previous = {}

    def _previous_point(self, track: set[str]) -> Dict[str, float]:
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

    def _get_lerp_step(self, target: Dict[str, float]) -> Dict[str, float]:
        """Result of a single step of lerp."""
        track = set(target.keys())
        order = list(target.keys())

        center = only_position(self.sensor)
        assert (
            set(center.keys()) >= track
        ), f"Sensor does not have required joint data. target joints {track} is not a subsets of the sensor set {set(center.keys())}."

        previous = self._previous_point(track)
        assert (
            set(previous.keys()) >= track
        ), f"Previous step does not have required joint data"

        clamped = clamp_to_sqewed_hs(
            start=_order_dict2arr(order, previous),
            center=_order_dict2arr(order, center),
            end=_order_dict2arr(order, target),
            radii=np.full((len(order),), self._interpolation_delta),
        )
        return dict(zip(order, clamped))

    def _get_asap_step(self, target: Dict[str, float]) -> Dict[str, float]:
        """Result of a single step of asap."""
        track = set(target.keys())
        order = list(target.keys())

        center = only_position(self.sensor)
        assert (
            set(center.keys()) >= track
        ), f"Sensor does not have required joint data. target joints {track} is not a subsets of the sensor set {set(center.keys())}."

        previous = self._previous_point(track)
        assert (
            set(previous.keys()) >= track
        ), f"Previous step does not have required joint data"

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

        return dict(zip(order, clamped))

    def _get_unsafe_step(self, target: Dict[str, float]) -> Dict[str, float]:
        """Result of a single step of unsafe."""
        track = set(target.keys())

        center = only_position(self.sensor)
        assert set(center.keys()) <= track, f"Sensor does not have required joint data"

        return target

    def _traj_toward(
        self,
        target: Dict[str, float],
        step_func: Callable[[Dict[str, float]], Dict[str, float]],
    ) -> bool:
        """Executes a step of the given step function."""
        order = list(target.keys())
        target_ar = _order_dict2arr(order, target)
        prev_ar = _order_dict2arr(order, self._previous_point(set(order)))
        command_done = bool(
            np.linalg.norm(target_ar - prev_ar) < self._COMMAND_DONE_DELTA
        )

        if not command_done:
            next = step_func(target)
            self.send_to_lvl1([JState(name, position=pos) for name, pos in next.items()])
            self._update_previous_point(next)

            # n = _order_dict2arr(order, next)
        else:
            pass
            # n = prev_ar

        if not command_done:
            return False

        s = _order_dict2arr(order, only_position(self.sensor))
        on_target = bool(np.linalg.norm(target_ar - s) < self._on_target_delta)
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
    ) ->FutureT:
        """Makes the trajectory task using the toward function.

        Args:
            target: key = joint name ; value = joint angle
            toward_func: Function executing a step toward the target.

        Returns:
            Future of the task. Done when sensorare on target.
        """
        future = self.FutureT()

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

    def speed_safe(
        self, target: Dict[str, float], delta_time: Union[float, Callable[[], float]]
    ) ->FutureT:
        """NOT TESTED. USE AT YOUR OWN RISK

        Args:
            target:
            delta_time:

        Returns:

        """
        if not callable(delta_time):
            delta_time = lambda x=delta_time: x

        def speed(target) -> bool:
            offset = {k: v * delta_time() for k, v in target.items()}
            return self.asap_toward(self.abs_from_offset(offset))

        return self._make_motion(target, speed)


def only_position(js_dict: Union[Dict[str, JState], List[JState]]) -> Dict[str, float]:
    """Extract velocities from a dict or list of JState. None is ignored"""
    if isinstance(js_dict, list):
        return {js.name: js.position for js in js_dict if js.position is not None}
    else:
        return {n: js.position for n, js in js_dict.items() if js.position is not None}


def _order_dict2arr(
    order: List[str], data: dict[str, float]
) -> NDArray[Shape["N"], nt.Floating]:
    """return array given the order"""
    return np.array([data[n] for n in order])
