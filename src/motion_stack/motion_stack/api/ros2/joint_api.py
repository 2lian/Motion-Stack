"""ROS2 API to send/receive joint command/state to lvl1 and syncronise multiple joints."""

import copy
import json
from collections import ChainMap
from dataclasses import asdict
from os import environ
from threading import Lock
from typing import Callable, Dict, List, Optional, Set, Tuple, Type, Union

import numpy as np
import zenoh
from rclpy.node import Node
from rclpy.task import Future, Task
from roboticstoolbox.tools.urdf.urdf import configure_origin
from sensor_msgs.msg import JointState

from ...core.utils.joint_state import JState, impose_state
from ...ros2 import communication as comms
from ...ros2.utils.conversion import delta_time_callable
from ...ros2.utils.executor import error_catcher
from ...ros2.utils.joint_state import publish_jstate, ros2js, ros2js_wrap
from ..joint_syncer import JointSyncer

with open(environ["ZENOH_SESSION_CONFIG_URI"], "r", encoding="utf-8") as file:
    confi_str = file.read()

zenoh_config = zenoh.Config.from_json5(confi_str)


class JointHandler:
    """ROS2 API to send/receive joint command/state to lvl1.

    One instance is limited to a single limb.

    Note:
        To safely execute joint movement to a target, do not directly use this class, but  use :py:class:`.JointSyncerRos`.

    Args:
        node: Spinning node.
        limb_number: Limb number on which to interface with the joints.
    """

    session: zenoh.Session = zenoh.open(zenoh_config)

    def __init__(self, node: Node, limb_number: int) -> None:
        self._lock = Lock()
        with self._lock:
            #: Joint available on the limb
            self.tracked: Set[str] = set()
            #: Limb number
            self.limb_number: int = limb_number
            #: Callback executed when the state sensor updates. Argument is this object instance.
            self.new_state_cbk: List[Callable[["JointHandler"],]] = []
            #: Future becoming done when sensor data is available on all tracked joints
            self.ready: Future = Future()
            self._paired = Future()

            self._node = node
            self._states: Dict[str, JState] = {}
            self._last_task_dic: Dict[str, Task] = dict()

            print("API USING ZENOH")
            # if self.session is None:
            #     with open(
            #         environ["ZENOH_SESSION_CONFIG_URI"], "r", encoding="utf-8"
            #     ) as file:
            #         confi_str = file.read()
            #
            #     zenoh_config= zenoh.Config.from_json5(confi_str)
            #     self.session = zenoh.open(zenoh_config)

            self.querrier = self.session.declare_querier(
                f"ms/{comms.limb_ns(self.limb_number)}/available_joints/**",
            )

            self._zenoh_pub: Dict[str, zenoh.Publisher] = dict()
            self._zenoh_sub = self.session.declare_subscriber(
                f"ms/{comms.limb_ns(self.limb_number)}/{comms.lvl1.output.joint_state.name}/**",
                self._zenoh_sub_cbk,
            )

            self.ready_up()

    def __del__(self):
        self._zenoh_sub.undeclare()
        self.querrier.undeclare()
        for key, val in self._zenoh_pub.items():
            val.undeclare()
            del self._zenoh_pub[key]
        self.session.close()

    @property
    def states(self) -> List[JState]:
        """Accumulated joint state (sensor)."""
        return list(self._states.values())

    def ready_up(self, tracked: Optional[Set[str]] = None) -> Tuple[Future, Future]:
        """Starts timer looking for available joints and their data.

        Args:
            tracked:
                Joints required to be considered ready.

        Returns:

            - [0] Future done when all available joints have data.
            - [1] Future done when the leg replies with the names of the available joints
        """

        self.ready.cancel()
        self._paired.cancel()
        self.ready = Future()
        self._paired = Future()

        if tracked is not None:
            self.tracked = tracked
            self._paired.set_result(self.tracked)
            return self.ready, self._paired

        lock = Lock()

        def reply_processing(rep: zenoh.Reply):
            nonlocal lock
            if rep.ok is None:
                return
            raw_dic = json.loads(rep.ok.payload.to_string())
            names = set(raw_dic.keys())

            with lock:
                self.tracked = self.tracked | names
                self._paired.set_result(self.tracked)
            # print(f"\nGot names {names}\n")

        def new_match(status: zenoh.MatchingStatus):
            if status.matching:
                # new matching queriable
                self.querrier.get(
                    handler=reply_processing,
                )
            else:
                # no match
                pass

        self.querrier.declare_matching_listener(new_match)

        return self.ready, self._paired

    def _zenoh_sub_cbk(self, sample: zenoh.Sample):
        # print(f"Received {sample.kind} ('{sample.key_expr}': '{sample.payload.to_string()}')")
        json_listdic: Dict = json.loads(sample.payload.to_bytes())
        js: JState = JState(**json_listdic)
        with self._lock:
            if self._node.executor is None:
                return
            last_task = self._last_task_dic.get(js.name)
            last_sensor = self._states.get(js.name)
            if last_task is not None:
                if last_sensor is not None:  # cancel if no data
                    if last_sensor.time is not None and js.time is not None:
                        # cancel if no time data
                        if last_sensor.time <= js.time:
                            # cancel if new data
                            last_task.cancel()
                    else:
                        last_task.cancel()
                else:
                    last_task.cancel()
            task = self._node.executor.create_task(self._update_state, js)

            def fin(fut: Future):
                if fut.cancelled():
                    print("cancelled")
                return

            task.add_done_callback(fin)
            self._last_task_dic[js.name] = task

    def _update_state(self, js: JState):
        self._states.update(
            {js.name: impose_state(onto=self._states.get(js.name), fromm=js)}
        )
        if not self.ready.done() and self._paired.done():
            tracked_joint_data_available = set(self._states.keys()) >= self.tracked
            if tracked_joint_data_available:
                self.ready.set_result(True)
        for f in self.new_state_cbk:
            f(self)

    def send(self, states: List[JState]):
        """Sends joint command to lvl1."""
        for js in states:
            pub = self._zenoh_pub.get(js.name)
            if pub is None:
                pub = self.session.declare_publisher(
                    f"ms/{comms.limb_ns(self.limb_number)}/{comms.lvl1.input.joint_target.name}/{js.name}",
                    reliability=zenoh.Reliability.RELIABLE,
                )
                self._zenoh_pub[js.name] = pub
            pub.put(json.dumps(asdict(js), indent=1))


class JointSyncerRos(JointSyncer):
    """Controls and syncronises several joints, safely executing trajectory to a target.

    Important:
        This class is a ROS2 implementation of the base class: :py:class:`.api.joint_syncer.JointSyncer`. Refere to it for documentation.

    Args:
        joint_handlers: ROS2 objects handling joint communications of several limbs.
    """

    def __init__(
        self,
        joint_handlers: List[JointHandler],
        interpolation_delta: float = np.deg2rad(5),
        on_target_delta: float = np.deg2rad(4),
    ) -> None:
        super().__init__(interpolation_delta, on_target_delta)
        self._joint_handlers = joint_handlers

    def execute(self):
        """Executes one step of the task/trajectory.

        This must be called frequently in a ros Timer or something else of your liking.
        """
        return super().execute()

    @property
    def sensor(self) -> Dict[str, JState]:
        """
        Important:
            This class is a ROS2 implementation of the base class: :py:class:`.api.joint_syncer.JointSyncer`. Refere to it for documentation.
        """
        return copy.deepcopy(
            dict(ChainMap(*[jh._states for jh in self._joint_handlers]))
        )

    def send_to_lvl1(self, states: List[JState]):
        """
        Important:
            This class is a ROS2 implementation of the base class: :py:class:`.api.joint_syncer.JointSyncer`. Refere to it for documentation.
        """
        for jh in self._joint_handlers:
            jh.send(states)

    def speed_safe(
        self,
        target: Dict[str, float],
        delta_time: Optional[Union[float, Callable[[], float]]] = None,
    ) -> Future:
        """Overloaded to automatically create the delta_time function.

        Important:
            This class is a ROS2 implementation of the base class: :py:class:`.api.joint_syncer.JointSyncer`. Refere to it for documentation.
        """
        if delta_time is None:
            delta_time_non_float = delta_time_callable(self._joint_handlers[0]._node)
            delta_time_non_float()

            def delta_tim() -> float:
                return delta_time_non_float().sec()

            delta_time = delta_tim

        return super().speed_safe(target, delta_time)

    @property
    def FutureT(self) -> Type[Future]:
        """
        Important:
            This class is a ROS2 implementation of the base class: :py:class:`.api.joint_syncer.JointSyncer`. Refere to it for documentation.
        """
        return Future
