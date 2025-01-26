"""
Provides api to sync the movement of several joints
"""

from abc import abstractmethod
from typing import (
    Any,
    Callable,
    Dict,
    Final,
    List,
    Literal,
    Optional,
    Sequence,
    Tuple,
    TypeVar,
    Union,
    overload,
)

import nptyping as nt
import numpy as np
from nptyping import NDArray, Shape

from motion_stack.core.utils.hypersphere_clamp import clamp_to_sqewed_hs
from motion_stack.core.utils.joint_state import JState, impose_state


def order_dict2arr(
    order: List[str], data: dict[str, float]
) -> NDArray[Shape["N"], nt.Floating]:
    return np.array([data[n] for n in order])


def only_position(js_dict: Dict[str, JState]) -> Dict[str, float]:
    return {n: js.position for n, js in js_dict.items() if js.position is not None}


class JointSyncer:
    DELTA = np.deg2rad(10)
    _DONE_DELTA = np.deg2rad(0.01)

    def __init__(self) -> None:
        self._previous: Dict[str, float] = {}
        self._trajectory_task = lambda *_: None
        pass

    @abstractmethod
    def send_to_lvl1(self, states: List[JState]): ...

    @property
    @abstractmethod
    def future_type(self) -> type["Future"]: ...

    def abs_from_offset(self, offset: Dict[str, float]) -> Dict[str, float]:
        track = set(offset.keys())
        prev = self._previous_point(track)
        return {name: prev[name] + offset[name] for name in track}

    @property
    @abstractmethod
    def sensor(self) -> Dict[str, JState]:
        """The sensor property."""
        ...

    def clear(self):
        self._previous = {}

    def _previous_point(self, track: set[str]) -> Dict[str, float]:
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
        track = set(target.keys())
        order = list(target.keys())

        center = only_position(self.sensor)
        assert set(center.keys()) >= track, f"Sensor does not have required joint data. target joints {track} is not a subsets of the sensor set {set(center.keys())}."

        previous = self._previous_point(track)
        assert (
            set(previous.keys()) >= track
        ), f"Previous step does not have required joint data"

        clamped = clamp_to_sqewed_hs(
            start=order_dict2arr(order, previous),
            center=order_dict2arr(order, center),
            end=order_dict2arr(order, target),
            radii=np.full((len(order),), self.DELTA),
        )
        return dict(zip(order, clamped))

    def _get_asap_step(self, target: Dict[str, float]) -> Dict[str, float]:
        track = set(target.keys())
        order = list(target.keys())

        center = only_position(self.sensor)
        assert set(center.keys()) >= track, f"Sensor does not have required joint data. target joints {track} is not a subsets of the sensor set {set(center.keys())}."

        previous = self._previous_point(track)
        assert (
            set(previous.keys()) >= track
        ), f"Previous step does not have required joint data"

        start_arr = order_dict2arr(order, previous)
        center_arr = order_dict2arr(order, center)
        end_arr = order_dict2arr(order, target)

        clamped = np.clip(
            a=end_arr,
            a_max=center_arr + self.DELTA,
            a_min=center_arr - self.DELTA,
        )

        clamped = np.clip(
            a=clamped,
            a_max=start_arr + self.DELTA,
            a_min=start_arr - self.DELTA,
        )
        return dict(zip(order, clamped))

    def _get_unsafe_step(self, target: Dict[str, float]) -> Dict[str, float]:
        track = set(target.keys())

        center = only_position(self.sensor)
        assert set(center.keys()) <= track, f"Sensor does not have required joint data"

        return target

    def _traj_toward(
        self,
        target: Dict[str, float],
        step_func: Callable[[Dict[str, float]], Dict[str, float]],
    ) -> bool:
        next = step_func(target)
        order = list(target.keys())
        self.send_to_lvl1([JState(name, position=pos) for name, pos in next.items()])
        self._update_previous_point(next)

        t = order_dict2arr(order, target)
        n = order_dict2arr(order, next)
        return bool(np.linalg.norm(t - n) < self._DONE_DELTA)

    def unsafe_toward(self, target: Dict[str, float]) -> bool:
        return self._traj_toward(target, self._get_asap_step)

    def asap_toward(self, target: Dict[str, float]) -> bool:
        return self._traj_toward(target, self._get_asap_step)

    def lerp_toward(self, target: Dict[str, float]) -> bool:
        return self._traj_toward(target, self._get_lerp_step)

    def _make_motion(
        self, target: Dict[str, float], toward_func: Callable[[Dict[str, float]], bool]
    ) -> "Future":
        future = self.future_type()

        def step_toward_target():
            if future.cancelled():
                return
            if future.done():
                return

            move_done = toward_func(target)
            if move_done:
                future.set_result(move_done)

        self._trajectory_task = step_toward_target
        self.execute()
        return future

    def lerp(self, target: Dict[str, float]) -> "Future":
        return self._make_motion(target, self.lerp_toward)

    def asap(self, target: Dict[str, float]) -> "Future":
        return self._make_motion(target, self.asap_toward)

    def unsafe(self, target: Dict[str, float]) -> "Future":
        return self._make_motion(target, self.unsafe_toward)

    def speed_safe(
        self, target: Dict[str, float], delta_time: Union[float, Callable[[], float]]
    ) -> "Future":
        if not callable(delta_time):
            delta_time = lambda x=delta_time: x

        def speed(target) -> bool:
            offset = {k: v * delta_time() for k, v in target.items()}
            return self.asap_toward(self.abs_from_offset(offset))

        return self._make_motion(target, speed)

    def update_and_exec(self, states: List[JState]):
        self.update_state(states)
        self.execute()

    def execute(self):
        self._trajectory_task()
