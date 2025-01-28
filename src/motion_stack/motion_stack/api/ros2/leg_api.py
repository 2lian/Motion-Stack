from collections import ChainMap
from typing import Callable, Dict, List, Optional, Set, Tuple

from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState

from motion_stack.core.utils.joint_state import JState, impose_state
import motion_stack.ros2.communication as comms
import motion_stack.ros2.ros2_asyncio.ros2_asyncio as rao
from motion_stack.ros2.utils.executor import error_catcher
from motion_stack.ros2.utils.joint_state import publish_jstate, ros2js, ros2js_wrap

from ..joint_syncer import JointSyncer


class JointHandler:
    def __init__(self, node: Node, limb_number: int) -> None:
        self._node = node
        self._states: Dict[str, JState] = {}
        self.tracked = set()
        self.limb_number = limb_number
        self.sub = node.create_subscription(
            comms.lvl1.output.joint_state.type,
            f"{comms.limb_ns(self.limb_number)}/{comms.lvl1.output.joint_state.name}",
            ros2js_wrap(self._update_state),
            10,
        )
        self.new_state_cbk: List[Callable[["JointHandler"],]] = []
        self.pub = node.create_publisher(
            comms.lvl1.input.joint_target.type,
            f"{comms.limb_ns(self.limb_number)}/{comms.lvl1.input.joint_target.name}",
            10,
        )
        self.advert = node.create_client(
            comms.lvl1.output.advertise.type,
            f"{comms.limb_ns(self.limb_number)}/{comms.lvl1.output.advertise.name}",
        )
        self.ready = Future()
        self._paired = Future()
        self.ready_up()

    def ready_up(self, tracked: Optional[Set[str]] = None) -> Tuple[Future, Future]:
        timeout = 3
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
            # print("answer")
            msg: JointState = call.result().js
            js = ros2js(msg)

            self.tracked = {js.name for js in js}
            self._paired.set_result(self.tracked)

        prev_call = Future()

        @error_catcher
        def tmrCBK():
            if self._paired.done():
                return
            nonlocal prev_call
            prev_call.cancel()
            prev_call = self.advert.call_async(comms.lvl1.output.advertise.type.Request())
            prev_call.add_done_callback(call_processingCBK)

        # tmr = self._node.create_timer(
        # timeout, lambda *_: rao.ensure_future(self._node, cbk())
        # )
        tmr = self._node.create_timer(timeout, tmrCBK)
        self._paired.add_done_callback(lambda *_: self._node.destroy_timer(tmr))
        return self.ready, self._paired

    def _update_tracked(self) -> Tuple[Future, Future]:
        out_fut = Future()

        @error_catcher
        def cbk(future: Future):
            if out_fut.cancelled():
                future.cancel()
                return
            if future.cancelled():
                out_fut.cancel()
                return
            msg: JointState = future.result().js
            js = ros2js(msg)
            # print("\nwow\n")

            self.tracked = {js.name for js in js}
            out_fut.set_result(self.tracked)

        call = self.advert.call_async(comms.lvl1.output.advertise.type.Request())
        call.add_done_callback(cbk)

        self._paired.cancel()
        self._paired = out_fut
        self.ready.cancel()
        self.ready = Future()
        return self._paired, self.ready

    @property
    def states(self) -> List[JState]:
        """The states property."""
        return list(self._states.values())

    @states.setter
    def states(self, value: List[JState]):
        self._update_state(value)

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
        publish_jstate(self.pub, states)


class JointSyncerRos(JointSyncer):
    def __init__(self, joint_handlers: List[JointHandler]) -> None:
        super().__init__()
        self.joint_handlers = joint_handlers

    @property
    def sensor(self) -> Dict[str, JState]:
        return dict(ChainMap(*[jh._states for jh in self.joint_handlers]))

    def send_to_lvl1(self, states: List[JState]):
        for jh in self.joint_handlers:
            jh.send(states)

    @property
    def future_type(self) -> Future:
        return Future
