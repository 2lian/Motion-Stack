import asyncio
import logging
import time
from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, List, Optional, Union

from motion_stack.core.lvl1_joint import JointCore
from motion_stack.core.lvl1_pipeline import Lvl1Param, lvl1_default
from motion_stack.core.utils import static_executor
from motion_stack.core.utils.joint_state import (
    JState,
    JStateBuffer,
    MultiJState,
    multi_to_js_dict,
)
from motion_stack.core.utils.static_executor import PythonSpinner
from motion_stack.core.utils.time import Time
from motion_stack.zenoh.base_node.lvl1 import Lvl1Node
from motion_stack.zenoh.utils.async_wrapper import LazyPub, ParsedSub
from motion_stack.zenoh.utils.communication import JointStatePub, JointStateSub

logger = logging.getLogger("motion_stack.zenoh.default_node.lvl1")


class DefaultLvl1(Lvl1Node):
    def __init__(self, params: Lvl1Param = lvl1_default):
        super().__init__(params)

        self.lvl0_pub = LazyPub("ms/lvl0/joint_commands")
        self.lvl0_sub: ParsedSub[JState] = ParsedSub("ms/lvl0/joint_states/**")
        self.lvl2_pub = LazyPub("ms/lvl2/joint_read")
        self.lvl2_sub: ParsedSub[JState] = ParsedSub("ms/lvl2/joint_set/**")

    def subscribe_to_lvl0(self, lvl0_input: Callable[[MultiJState], Any]):
        async def sensor_loop():
            async for joint_state in self.lvl0_sub.listen_reliable(queue_size=1000):
                lvl0_input(joint_state)

        asyncio.create_task(sensor_loop())

    def subscribe_to_lvl2(self, lvl2_input: Callable[[MultiJState], Any]):
        async def command_loop():
            async for joint_state in self.lvl2_sub.listen_reliable(queue_size=1000):
                lvl2_input(joint_state)

        asyncio.create_task(command_loop())

    def publish_to_lvl0(self, states: Dict[str, JState]):
        if len(states) <= 0:
            return
        for key, js in states.items():
            self.lvl0_pub.pub(js, key)

    def publish_to_lvl2(self, states: Dict[str, JState]):
        if len(states) <= 0:
            return
        for key, js in states.items():
            self.lvl2_pub.pub(js, key)

    def frequently_send_to_lvl2(self, send_function: Callable[[], None], period: float):
        async def periodic():
            nonlocal period, send_function
            while 1:
                dt = int(period * 1e9) - (time.time_ns() % int(period * 1e9))
                await asyncio.sleep(dt / 1e9)
                send_function()

        asyncio.create_task(periodic())

    def close(self):
        self.lvl0_sub.close()
        self.lvl0_pub.close()
        self.lvl2_pub.close()
        self.lvl2_sub.close()


def main():
    loop = asyncio.get_event_loop()
    node = DefaultLvl1()
    loop.run_forever()


if __name__ == "__main__":
    main()
