"""
Python API to sync the movement of several end_effectors.
This requires ik lvl2 to be running.

Note:
    The ros2 implementation is available in :py:mod:`.ros2.ik_api`.

This high level API alows for multi-end-effector control and syncronization (over several legs). This is the base class where, receiving and sending data to motion stack lvl2 is left to be implemented.
"""

call_cnt = 0

import copy
import warnings
from abc import ABC, abstractmethod
from typing import Awaitable, Callable, Dict, List, Optional, Set, Tuple, Type, Union

import nptyping as nt
import numpy as np

from motion_stack.core.utils.time import Time

from ..core.utils.hypersphere_clamp import clamp_multi_xyz_quat, fuse_xyz_quat
from ..core.utils.math import (
    Flo3,
    Quaternion,
    angle_with_unit_quaternion,
    qt,
    qt_normalize,
)
from ..core.utils.pose import Pose, VelPose, XyzQuat
from ..core.utils.joint_state import JState

# [Preliminary] flag to provide Yamcs logging. Can be changed to reading an environment variable?
DEBUG_PRINT = False
YAMCS_LOGGING = True
if YAMCS_LOGGING:
    from ygw_client import YGWClient

#: placeholder type for a Future (ROS2 Future, asyncio or concurrent)
FutureType = Awaitable

#: A type alias representing the limb number (end effector index).
LimbNumber = int

#: A dictionary mapping limb numbers to their corresponding poses.
MultiPose = Dict[LimbNumber, Pose]


class SensorSyncWarning(Warning):
    pass


class IkSyncer(ABC):
    r"""One instance controls and syncronises several limbs end-effectors, safely executing trajectory to targets.

    Note:
        This class is an abstract base class, the ros2 implementation is available in :py:class:`.ros2.joint_api.IkSyncerRos`. Hence,  parts of this class are left to be implmented by the interface/runtime: \ :py:meth:`.IkSyncer.FutureT`, :py:meth:`.IkSyncer.sensor`, :py:meth:`.IkSyncer.send_to_lvl2`.

    Important:
        \ :py:meth:`.IkSyncer.execute` must be called to compute, update and send the command.

        One object instance can only execute one target at a time. However, the limbs or targets can change between calls of the same instance, before the previous task is done.

    The trajectory interpolates between two points:

        - The last position (if None: uses sensor, else: last sub-target). This is handled automatically, however ``clear`` resets the last position to None.
        - The input target.

    Several interpolation strategies are available:

     - LERP: :py:meth:`.IkSyncer.lerp`
     - ASAP: :py:meth:`.IkSyncer.asap`
     - Unsafe: :py:meth:`.IkSyncer.unsafe`

    Args:
        interpolation_delta: (mm, rad) During movement, how much divergence is allowed from the path. If exceeded, movement slows down.
        on_target_delta: (mm, rad) Delta at which the trajectory/task is considered finished and the Future is switched to ``done``.
    """

    _COMMAND_DONE_DELTA: XyzQuat[float, float] = XyzQuat(0.01, np.deg2rad(0.01))

    def __init__(
        self,
        interpolation_delta: XyzQuat[float, float] = XyzQuat(40, np.deg2rad(4)),
        on_target_delta: XyzQuat[float, float] = XyzQuat(40, np.deg2rad(4)),
    ) -> None:
        self._interpolation_delta: XyzQuat[float, float] = interpolation_delta
        self._on_target_delta: XyzQuat[float, float] = on_target_delta
        #: Future of the latest task/trajectory that was run.
        self.last_future: FutureType = self.FutureT()
        #: When true, the command messages sill stop being published when the sensor data is on target. When false, it stops after sending the target command once. True is improves reliability at the expense of more messages sent, better for lossy networks.
        self.SEND_UNTIL_DONE = True

        self.__previous: MultiPose = {}
        self._last_sent: MultiPose = {}
        self._last_valid: MultiPose = {}
        self._trajectory_task = lambda *_: None

        if YAMCS_LOGGING:
            self.ygw_client = YGWClient(host="localhost", port=7902)  # one port per ygw client. See yamcs-moonshot/ygw-leg/config.yaml
            # Get operator name as metadata to logged commands
            import os, socket, getpass
            operator = os.getenv("OPERATOR")
            if not operator:
                limb_id = os.getenv("LIMB_ID")
                if limb_id and limb_id != "ALL":
                    operator = limb_id
                else:
                    operator = f"{getpass.getuser()}@{socket.gethostname()}"
            self.operator = operator
        if DEBUG_PRINT:
            print(f"Operator: {self.operator}")
            # counters to reduce debug printing frequency
            self.DECIMATION_FACTOR = 1
            self.ptime_to_lvl2 = 0
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
        self.__previous = {}
        self._last_valid = {}

    def lerp(self, target: MultiPose) -> FutureType:
        """Starts executing a lerp trajectory toward the target.

        LERP: all joints reach the target at the same time.

        Returns:
            Future of the task. Done when sensors are on target.
        """
        return self._make_motion(target, self.lerp_toward)

    def asap(self, target: MultiPose) -> FutureType:
        """Starts executing a asap trajectory toward the target.

        ASAP: (Not Implemented) joints will reach their tagets indepently, as fast as possible.

        Returns:
            Future of the task. Done when sensors are on target.
        """
        return self._make_motion(target, self.asap_toward)

    def unsafe(self, target: MultiPose) -> FutureType:
        """Starts executing a unsafe trajectory toward the target.

        Unsafe: Similar to ASAP except the final target is sent directly to the IK, so the movement will not stop in case of crash, errors, network issue AND you cannot cancel it.

        Returns:
            Future of the task. Done when sensors are on target.
        """
        return self._make_motion(target, self.unsafe_toward)

    def speed_safe(
        self,
        target: Dict[LimbNumber, VelPose],
        delta_time: Union[float, Callable[[], float]],
    ) -> FutureType:
        """
        A Cartesian speed‐safe trajectory that interprets angular velocity
        as a rotation vector (axis * rad/s) instead of a quaternion.

        Args:
            target: Mapping limb → VelPose, where
                • VelPose.lin is linear speed (mm/s)
                • VelPose.rvec is rotational speed vector (axis * rad/s)
            delta_time: Either a fixed Δt (s) or a zero‐arg callable returning Δt.

        Returns:
            A Future that continuously steps the motion until cancelled.
        """
        if not callable(delta_time):
            delta_time = lambda x=delta_time: x

        target_mp = {
            limb: Pose(
                Time(0), target[limb].lin, qt.from_rotation_vector(target[limb].rvec)
            )
            for limb in target.keys()
        }

        def _step_speed(target_mp: MultiPose, start: MultiPose | None = None) -> bool:
            dt = delta_time()
            rel_offsets: MultiPose = {}

            for limb, vp in target.items():
                Δxyz = vp.lin * dt
                Δquat = qt.from_rotation_vector(vp.rvec * dt)
                rel_offsets[limb] = Pose(Time(0), Δxyz, qt_normalize(Δquat))

            abs_targets = self.abs_from_rel(rel_offsets)
            self.lerp_toward(abs_targets)
            return False

        return self._make_motion(target_mp, _step_speed)

    def abs_from_rel(self, offset: MultiPose) -> MultiPose:
        """Absolute position of the MultiPose that corresponds to the given relative offset.

        Example:
            *joint_1* is at 45 deg, offset is 20 deg. Return will be 65 deg.

        Args:
            offset: Relative postion.

        Returns:
            Absolute position.
        """
        track = set(offset.keys())
        prev = self._previous_point(track)
        return {
            key: Pose(
                offset[key].time,
                prev[key].xyz + qt.rotate_vectors(prev[key].quat, offset[key].xyz),
                prev[key].quat * offset[key].quat,
            )
            for key in track
        }

    ## [Temporary] Dummy print function as placeholder to YGW logging       
    def dummy_print_multipose(self, data: MultiPose, prefix: str = ""):
        str_to_send: List[str] = [f"High : "]
        for limb, pose in data.items():
            str_to_send.append(
                f"{prefix} limb {limb} | "
                f"xyz: {pose.xyz} | quat: {pose.quat}"
            )
        print("\n".join(str_to_send))
        
    def _send_to_lvl2(self, ee_targets: MultiPose):
        self.send_to_lvl2(ee_targets)
        
        if YAMCS_LOGGING:
            self.ygw_client.publish_dict(
                group="ik_syncer_send_to_lvl2_ee_targets",
                data=ee_targets,
                operator=self.operator                                
            )
        if DEBUG_PRINT:
            self.ptime_to_lvl2 += 1
            if self.ptime_to_lvl2 % (self.DECIMATION_FACTOR * 100) == 0:
                self.dummy_print_multipose(ee_targets, prefix="send: high -> lvl2:")

    @abstractmethod
    def send_to_lvl2(self, ee_targets: MultiPose):
        """Sends ik command to lvl2.

        Important:
            This method must be implemented by the runtime/interface.

        Note:
            Default ROS2 implementation: :py:meth:`.ros2.ik_api.IkSyncerRos.send_to_lvl2`

        Args:
            states: Ik target to be sent to lvl1
        """
        ...

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
    def _sensor(self) -> MultiPose:
        sensor_values = self.sensor  # type: MultiPose
        
        if YAMCS_LOGGING:
            self.ygw_client.publish_dict(
                group="ik_syncer_sensor_values",
                data=sensor_values,
                operator=self.operator                                
            )
        if DEBUG_PRINT:
            self.ptime_sensor += 1
            if self.ptime_sensor % (self.DECIMATION_FACTOR * 50) == 0:
                self.dummy_print_multipose(sensor_values, prefix="sensor: lvl2 -> high:")

        return sensor_values
    
    @property
    @abstractmethod
    def sensor(self) -> MultiPose:
        """Is called when sensor data is needed.

        Important:
            This method must be implemented by the runtime/interface.

        Note:
            Default ROS2 implementation: :py:meth:`.ros2.ik_api.IkSyncerRos.sensor`

        Returns:

        """
        ...

    def _previous_point(self, track: Set[LimbNumber]) -> MultiPose:
        """
        Args:
            track: Joints to consider.

        Returns:
            Previous point in the trajectory. If missing data, uses sensor.

        """
        missing = track - set(self.__previous.keys())
        if not missing:
            return self.__previous
        sensor = self._sensor
        available = set(sensor.keys())
        for name in missing & available:
            self.__previous[name] = sensor[name]
        return self.__previous

    def _update_previous_point(self, data: MultiPose) -> None:
        data = copy.deepcopy(data)
        self.__previous.update(data)
        return

    def _get_last_valid(self, track: Set[LimbNumber]) -> MultiPose:
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
            self._last_valid[name] = sensor[name]
        return self._last_valid

    def _set_last_valid(self, data: MultiPose) -> None:
        data = copy.deepcopy(data)
        self._last_valid.update(data)
        return

    def _center_and_previous(self, track: Set[LimbNumber]):
        center = self._sensor
        assert (
            set(center.keys()) >= track
        ), f"Sensor does not have required end-effector data. "
        f"target joints {track} is not a subsets of the sensor set {set(center.keys())}."

        previous = self._previous_point(track)
        assert (
            set(previous.keys()) >= track
        ), f"Previous step does not have required end-effector data. "
        f"Target joints {track} is not a subsets of the sensor set {set(previous.keys())}."
        return center, previous

    def _get_lerp_step(
        self, start: MultiPose, target: MultiPose
    ) -> Tuple[MultiPose, bool]:
        """Result of a single step of lerp."""
        track = set(target.keys())
        order = list(target.keys())

        center, _ = self._center_and_previous(track)
        previous = start

        clamped = clamp_multi_xyz_quat(
            start=_order_dict2list(order, previous),
            center=_order_dict2list(order, center),
            end=_order_dict2list(order, target),
            radii=self._interpolation_delta,
        )
        validity = not bool(np.any(np.isnan(fuse_xyz_quat(clamped))))
        out_dict = {k: Pose(Time(0), v.xyz, v.quat) for k, v in zip(order, clamped)}
        return out_dict, validity

    def _get_asap_step(
        self, start: MultiPose, target: MultiPose
    ) -> Tuple[MultiPose, bool]:
        """Result of a single step of asap."""
        return NotImplemented, True

    def _get_unsafe_step(
        self, start: MultiPose, target: MultiPose
    ) -> Tuple[MultiPose, bool]:
        """Result of a single step of unsafe."""
        return target, True

    def _traj_toward(
        self,
        target: MultiPose,
        step_func: Callable[[MultiPose, MultiPose], Tuple[MultiPose, bool]],
        start: Optional[MultiPose] = None,
    ) -> bool:
        """Executes a step of the given step function."""
        order = list(target.keys())
        tracked = set(order)
        prev = self._previous_point(tracked)

        if len(target) == 0:
            return True
        if start is None:
            start = prev

        is_on_final = _multipose_close(
            set(target.keys()), target, prev, atol=self._COMMAND_DONE_DELTA
        )

        if not is_on_final:
            next, valid = step_func(start, target)
            if valid:
                self._set_last_valid(next)
            else:
                sens, _ = self._center_and_previous(tracked)
                next, valid = step_func(sens, self._get_last_valid(set(order)))
        else:
            next = target

        if self.SEND_UNTIL_DONE:
            self._send_to_lvl2(next)
            self._update_previous_point(next)

        if not is_on_final:
            return False

        on_target = _multipose_close(
            set(target.keys()), target, self._sensor, atol=self._on_target_delta
        )
        return on_target

    def unsafe_toward(
        self, target: MultiPose, start: Optional[MultiPose] = None
    ) -> bool:
        """Executes one single unsafe step.

        Returns:
            True if trajectory finished
        """
        return self._traj_toward(target, self._get_unsafe_step, start=start)

    def asap_toward(self, target: MultiPose, start: Optional[MultiPose] = None) -> bool:
        """Executes one single asap step.

        Returns:
            True if trajectory finished
        """
        return self._traj_toward(target, self._get_asap_step, start=start)

    def lerp_toward(self, target: MultiPose, start: Optional[MultiPose] = None) -> bool:
        """Executes one single lerp step.

        Returns:
            True if trajectory finished
        """
        return self._traj_toward(target, self._get_lerp_step, start=start)

    def _define_pose(self, multi_pose: MultiPose):
        """Replaces in place None with the latest send data or sensor."""
        prev = self._previous_point(set(multi_pose.keys()))
        for key, pose in multi_pose.items():
            if pose.time is None:
                pose.time = Time(0)
            if pose.xyz is None:
                pose.xyz = prev[key].xyz
            if pose.quat is None:
                pose.quat = prev[key].quat

    def _make_motion(
        self,
        target: MultiPose,
        toward_func: Callable[[MultiPose, Optional[MultiPose]], bool],
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
                group="ik_syncer_make_motion_target",
                data=target,
                operator=self.operator,                                
            )
        if DEBUG_PRINT:
            self.ptime_make_motion += 1
            if self.ptime_make_motion % (self.DECIMATION_FACTOR * 100) == 0:
                self.dummy_print_multipose(target, prefix="_make_motion: high -> lvl2:")
        
        future = self.FutureT()
        self.last_future.cancel()

        track = set(target.keys())
        sens, prev = self._center_and_previous(track)
        has_moved_since_last_time = _multipose_close(
            track, prev, sens, atol=self._interpolation_delta
        )
        if not has_moved_since_last_time:
            warnings.warn(
                "Syncer is out of sync with sensor data (something else than this syncer might have moved the end-effectors). `syncer.clear()` will be called automatically, thus the trajectory will resstart from the current sensor position. Raise this warning as an error to interupt operations.",
                SensorSyncWarning,
                stacklevel=3,
            )
            self.clear()
            sens, prev = self._center_and_previous(track)

        self._define_pose(target)
        target = copy.deepcopy(target)
        prev = copy.deepcopy(prev)

        stop = [False]
        global call_cnt
        call_cnt += 1

        def step_toward_target(future=future, stop=stop):
            if stop[0]:
                return
            if future.cancelled():
                stop[0] = True
                return
            if future.done():
                stop[0] = True
                return

            move_done = toward_func(target, prev)
            if move_done:
                future.set_result(move_done)

        self._trajectory_task = step_toward_target
        self.last_future = future
        # self.execute()
        return future

    def __del__(self):
        self.last_future.cancel()


def _order_dict2list(
    order: List[LimbNumber], data: MultiPose
) -> List[XyzQuat[Flo3, Quaternion]]:
    """return List[XyzQuat[Flo3, Quaternion]] given the order"""
    return [XyzQuat(data[k].xyz, data[k].quat) for k in order]


def _multipose_close(
    track: Set[int], a: MultiPose, b: MultiPose, atol: XyzQuat[float, float]
):
    """True if all tracked poses are close"""
    for key in track:
        close_enough = (a[key] - b[key]).close2zero(atol=atol)
        if not close_enough:
            return False
    return True
