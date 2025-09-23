import asyncio
import logging
import time
from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, List, Optional, Union

from motion_stack.core.lvl1_joint import JointCore
from motion_stack.core.utils import static_executor
from motion_stack.core.utils.joint_state import (
    JState,
    JStateBuffer,
    MultiJState,
    multi_to_js_dict,
)
from motion_stack.core.utils.static_executor import PythonSpinner
from motion_stack.core.utils.time import Time
from motion_stack.zenoh.utils.communication import JointStatePub, JointStateSub

logger = logging.getLogger(__name__)


class Lvl1Node(ABC):
    def __init__(
        self,
        sensor_buf: Optional[JState] = None,
        command_buf: Optional[JState] = None,
        batching_ms: float = 1,
        regular_ms: float = 500,
    ):
        logger.debug("lvl1 node instanciated")

        self.batching_ms = batching_ms
        self.regular_ms = regular_ms
        self._link_publishers()
        self._link_subscribers()
        self._link_timers()
        self._link_sensor_check()
        self._on_startup()

        default_jsbuf = JState(
            name="",
            time=Time(static_executor.default_param_dict["joint_buffer"][0][0]),
            position=(static_executor.default_param_dict["joint_buffer"][0][1]),
            velocity=(static_executor.default_param_dict["joint_buffer"][0][2]),
            effort=(static_executor.default_param_dict["joint_buffer"][0][3]),
        )
        self.sensor_buf: JStateBuffer
        self.command_buf: JStateBuffer
        if sensor_buf is None:
            self.sensor_buf = JStateBuffer(default_jsbuf)
        else:
            self.sensor_buf = JStateBuffer(sensor_buf)
        if command_buf is None:
            self.command_buf = JStateBuffer(default_jsbuf)
        else:
            self.command_buf = JStateBuffer(command_buf)

        self.d_required = set()
        self.d_displayed = set()
        logger.debug("lvl1 node initialized")

    @abstractmethod
    def subscribe_to_lvl0(self, lvl0_input: Callable[[MultiJState], Any]): ...

    @abstractmethod
    def subscribe_to_lvl2(self, lvl2_input: Callable[[MultiJState], Any]): ...

    def _link_subscribers(self):

        sensor_flush_scheduled = False

        def flush_to_lvl2():
            nonlocal sensor_flush_scheduled
            b = self.sensor_buf
            js_data = b.pull_urgent()
            sensor_flush_scheduled = False
            self.publish_to_lvl2(js_data)

        def lvl0_to_lvl2(js: MultiJState):
            nonlocal sensor_flush_scheduled
            js = multi_to_js_dict(js)
            b = self.sensor_buf
            b.push(js)
            if sensor_flush_scheduled:
                return
            is_urgent = b._find_urgent(b.last_sent, b._new, b.delta)
            if len(is_urgent) > 0:
                logger.debug(f"lvl0->2 urgent: %s ", set(is_urgent.keys()))
                if self.batching_ms > 0:
                    asyncio.get_event_loop().call_later(
                        self.batching_ms / 1000, flush_to_lvl2
                    )
                elif self.batching_ms < 0:
                    flush_to_lvl2()
                else:
                    asyncio.get_event_loop().call_soon(flush_to_lvl2)
                sensor_flush_scheduled = True

        self.subscribe_to_lvl0(lvl0_to_lvl2)

        command_flush_scheduled = False

        def flush_to_lvl0():
            nonlocal command_flush_scheduled
            b = self.command_buf
            js_data = b.pull_urgent()
            command_flush_scheduled = False
            self.publish_to_lvl0(js_data)

        def lvl2_to_lvl0(js: MultiJState):
            nonlocal command_flush_scheduled
            js = multi_to_js_dict(js)
            b = self.command_buf
            b.push(js)
            if command_flush_scheduled:
                return
            is_urgent = b._find_urgent(b.last_sent, b._new, b.delta)
            if is_urgent:
                logger.debug(f"lvl2->0 urgent: %s ", set(is_urgent.keys()))
                if self.batching_ms > 0:
                    asyncio.get_event_loop().call_later(
                        self.batching_ms / 1000, flush_to_lvl0
                    )
                elif self.batching_ms < 0:
                    flush_to_lvl0()
                else:
                    asyncio.get_event_loop().call_soon(flush_to_lvl0)
                command_flush_scheduled = True

        self.subscribe_to_lvl2(lvl2_to_lvl0)

    @abstractmethod
    def publish_to_lvl0(self, states: Dict[str, JState]): ...

    @abstractmethod
    def publish_to_lvl2(self, states: Dict[str, JState]): ...

    def _link_publishers(self): ...

    @abstractmethod
    def frequently_send_to_lvl2(
        self, send_function: Callable[[], None], period: float
    ): ...

    def _link_timers(self):
        def send():
            b = self.sensor_buf
            to_send = b.pull_new()
            logger.debug(f"non urgent sending of %s", set(to_send.keys()))
            self.publish_to_lvl2(to_send)

        if self.regular_ms > 0:
            self.frequently_send_to_lvl2(send, self.regular_ms / 1000)

    def _link_sensor_check(self):

        async def happy_new_state():
            while True:
                await asyncio.sleep(1 / 3)
                b = self.sensor_buf
                available = set(b.accumulated.keys())
                if len(available) <= 0:
                    continue
                new = self.d_displayed - available
                print(f"YEY new: {new}")

        asyncio.create_task(happy_new_state())

        def angry_new_state():
            b = self.sensor_buf
            available = set(b.accumulated.keys())
            missing = self.d_required - available
            print(f"OH NYO missing: {missing}")

        asyncio.get_event_loop().call_later(3, angry_new_state)

    # @abstractmethod
    def startup_action(self): ...

    def _on_startup(self):
        asyncio.get_event_loop().call_soon(self.startup_action)

def main():
    loop = asyncio.get_event_loop()
    node = Lvl1Node()
    loop.run_forever()


if __name__ == "__main__":
    main()
