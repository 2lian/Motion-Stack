from collections import ChainMap

from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState

import motion_stack.ros2.communication as comms
from motion_stack.ros2.utils.executor import error_catcher
from motion_stack.ros2.utils.joint_state import publish_jstate, ros2js, ros2js_wrap

from ..joint_syncer import *


class JointHandler:
    def __init__(self, node: Node, limb_number: int) -> None:
        self._node = node
        self._states: Dict[str, JState] = {}
        self.tracked = set()
        self.limb_number = limb_number
        self.sub = node.create_subscription(
            comms.lvl1.output.joint_state.type,
            f"leg{limb_number}/{comms.lvl1.output.joint_state.name}",
            ros2js_wrap(self._update_state),
            1,
        )
        self.new_state_cbk: List[Callable[["JointHandler"],]] = []
        self.pub = node.create_publisher(
            comms.lvl1.input.joint_target.type,
            f"leg{limb_number}/{comms.lvl1.input.joint_target.name}",
            1,
        )
        self.advert = node.create_client(
            comms.lvl1.output.advertise.type,
            f"leg{limb_number}/{comms.lvl1.output.advertise.name}",
        )
        self.ready = Future()
        self.paired = Future()
        self._update_tracked()

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

        self.paired.cancel()
        self.paired = out_fut
        self.ready.cancel()
        self.ready = Future()
        return self.paired, self.ready

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
        if not self.ready.done() and self.paired.done():
            if set(self._states.keys()) >= self.tracked:
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
