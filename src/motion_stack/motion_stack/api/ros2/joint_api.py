"""ROS2 API to send/receive joint command/state to lvl1 and syncronise multiple joints."""

from collections import ChainMap
from typing import Callable, Dict, List, Optional, Set, Tuple, Type, Union

import numpy as np
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState

from ...core.utils.joint_state import JState, impose_state
from ...ros2 import communication as comms
from ...ros2.utils.conversion import delta_time_callable
from ...ros2.utils.executor import error_catcher
from ...ros2.utils.joint_state import publish_jstate, ros2js, ros2js_wrap
from ..joint_syncer import JointSyncer


class JointHandler:
    """ROS2 API to send/receive joint command/state to lvl1.

    One instance is limited to a single limb.

    Note:
        To safely execute joint movement to a target, do not directly use this class, but  use :py:class:`.JointSyncerRos`.

    Args:
        node: Spinning node.
        limb_number: Limb number on which to interface with the joints.
    """

    def __init__(self, node: Node, limb_number: int) -> None:
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
        self._readSUB = node.create_subscription(
            comms.lvl1.output.joint_state.type,
            f"{comms.limb_ns(self.limb_number)}/{comms.lvl1.output.joint_state.name}",
            ros2js_wrap(self._update_state),
            qos_profile=comms.lvl1.output.joint_state.qos,
        )
        self._setPUB = node.create_publisher(
            comms.lvl1.input.joint_target.type,
            f"{comms.limb_ns(self.limb_number)}/{comms.lvl1.input.joint_target.name}",
            qos_profile=comms.lvl1.input.joint_target.qos,
        )
        self._advertCLI = node.create_client(
            comms.lvl1.output.advertise.type,
            f"{comms.limb_ns(self.limb_number)}/{comms.lvl1.output.advertise.name}",
        )
        self.ready_up()

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
        timeout = 1
        self.ready.cancel()
        self._paired.cancel()
        self.ready = Future()
        self._paired = Future()

        if tracked is not None:
            self.tracked = tracked
            self._paired.set_result(self.tracked)
            return self.ready, self._paired

        @error_catcher
        def call_processingCBK(call):
            if call.cancelled():
                return
            if self._paired.done():
                return
            msg: JointState = call.result().js
            js = ros2js(msg)

            self.tracked = {js.name for js in js}
            self._paired.set_result(self.tracked)

        prev_call = Future()

        @error_catcher
        def tmrCBK():
            if self._paired.done():
                return
            if not self._advertCLI.service_is_ready():
                return
            nonlocal prev_call
            prev_call.cancel()
            prev_call = self._advertCLI.call_async(
                comms.lvl1.output.advertise.type.Request()
            )
            prev_call.add_done_callback(call_processingCBK)

        tmr = self._node.create_timer(timeout, tmrCBK)
        self._paired.add_done_callback(lambda *_: self._node.destroy_timer(tmr))
        return self.ready, self._paired

    def _update_state(self, states: List[JState]):
        self._states.update(
            {
                js.name: impose_state(onto=self._states.get(js.name), fromm=js)
                for js in states
            }
        )
        if not self.ready.done() and self._paired.done():
            tracked_joint_data_available = set(self._states.keys()) >= self.tracked
            if tracked_joint_data_available:
                self.ready.set_result(True)
        for f in self.new_state_cbk:
            f(self)

    def send(self, states: List[JState]):
        """Sends joint command to lvl1."""
        publish_jstate(self._setPUB, states)


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
        return dict(ChainMap(*[jh._states for jh in self._joint_handlers]))

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

            def delta_time() -> float:
                return delta_time_non_float().sec()

        return super().speed_safe(target, delta_time)

    @property
    def FutureT(self) -> Type[Future]:
        """
        Important:
            This class is a ROS2 implementation of the base class: :py:class:`.api.joint_syncer.JointSyncer`. Refere to it for documentation.
        """
        return Future
