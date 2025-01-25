"""
Provides api anc controllers to control leg ik and joints directly

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

from abc import abstractmethod
from copy import deepcopy
from dataclasses import dataclass
from typing import (
    Any,
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
from easy_robot_control.EliaNode import (
    Client,
    EliaNode,
    error_catcher,
    get_src_folder,
    np2tf,
    rosTime2Float,
    tf2np,
)
from geometry_msgs.msg import Transform
from motion_stack_msgs.srv import ReturnJointState, TFService
from nptyping import NDArray, Shape
from rclpy import Node
from rclpy.publisher import Publisher
from rclpy.task import Future, Task
from rclpy.time import Duration, Time
from std_srvs.srv import Empty

from motion_stack.core.utils.hypersphere_clamp import clamp_to_sqewed_hs, clamp_xyz_quat
from motion_stack.core.utils.joint_state import JState, impose_state
from motion_stack.core.utils.math import Quaternion, qt, qt_repr

float_formatter = "{:.1f}".format
np.set_printoptions(formatter={"float_kind": float_formatter})

AvailableMvt = Literal["shift", "transl", "rot", "hop"]

ALLWOED_DELTA_JOINT = np.deg2rad(7)  # for joint motor control

# ik2 commands cannot be further than ALLOWED_DELTA_XYZ | ALLOWED_DELTA_QUAT away
# from the current tip pose
ALLOWED_DELTA_XYZ = 50  # mm ;
ALLOWED_DELTA_QUAT = np.deg2rad(5)  # rad ; same but for rotation


MVT2SRV: Final[Dict[AvailableMvt, str]] = {
    "shift": "shift",
    "transl": "rel_transl",
    "rot": "rot",
    "hop": "rel_hop",
}


def array_on_order(
    order: List[str], data: dict[str, float]
) -> NDArray[Shape["N"], nt.Floating]:
    return np.array([data[n] for n in order])


def only_position(js_dict: Dict[str, JState]) -> Dict[str, float]:
    return {n: js.position for n, js in js_dict.items() if js.position is not None}


class JointSyncer:
    DELTA = np.deg2rad(1)
    future_type = Future

    def __init__(self) -> None:
        self.sensor: Dict[str, JState] = {}
        self._previous: Dict[str, float] = {}
        self.trajectory_task = lambda *_: None
        pass

    def update_state(self, states: List[JState]):
        self.sensor.update(
            {
                state.name: impose_state(onto=self.sensor.get(state.name), fromm=state)
                for state in states
            }
        )

    def _previous_point(self, track: set[str]) -> Dict[str, float]:
        missing = track - set(self._previous.keys())
        if not missing:
            return self._previous
        sensor = only_position(self.sensor)
        available = set(sensor.keys())
        for name in missing & available:
            self._previous[name] = sensor[name]
        return self._previous

    def get_step_toward(self, target: Dict[str, float]) -> Dict[str, float]:
        track = set(target.keys())
        order = list(target.keys())

        center = only_position(self.sensor)
        assert set(center.keys()) <= track, f"Sensor does not have required joint data"

        previous = self._previous_point(track)
        assert (
            set(previous.keys()) <= track
        ), f"Previous step does not have required joint data"

        clamped = clamp_to_sqewed_hs(
            start=array_on_order(order, previous),
            center=array_on_order(order, center),
            end=array_on_order(order, target),
            radii=np.full(self.DELTA, (len(order),)),
        )

        return dict(zip(order, clamped))

    @abstractmethod
    def send_to_lvl1(self, states: List[JState]): ...

    def step_toward(self, target: Dict[str, float]) -> bool:
        next = self.get_step_toward(target)
        order = list(target.keys())
        t = array_on_order(order, target)
        n = array_on_order(order, next)
        self.send_to_lvl1([JState(name, position=pos) for name, pos in target.items()])
        return bool(np.linalg.norm(t - n) < 0.001)

    def _empty_trajectory(self, *args, **kwargs):
        return

    def lerp(self, target: Dict[str, float]) -> "Future":
        future = self.future_type()

        def step_toward_target():
            if future.cancelled():
                return
            if future.done():
                return

            move_done = self.step_toward(target)
            if move_done:
                future.set_result(move_done)

        self.trajectory_task = step_toward_target
        return future

    def update_and_exec(self, states: List[JState]):
        self.update_state(states)
        self.execute()

    def execute(self):
        self.trajectory_task()
