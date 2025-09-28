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
from motion_stack.zenoh.utils.communication import JointStatePub, JointStateSub

logger = logging.getLogger("motion_stack.zenoh.default_node.lvl1")

class DefaultLvl1(Lvl1Node):
    def __init__(self, params: Lvl1Param = lvl1_default):
        super().__init__(params)

        self.lvl0_pub = JointStatePub(key="TODO/lvl0/joint_commands")
        self.lvl0_sub = JointStateSub(key="TODO/lvl0/joint_states/**")
        self.lvl2_pub = JointStatePub(key="TODO/lvl2/joint_read")
        self.lvl2_sub = JointStateSub(key="TODO/lvl2/joint_set/**")

    def subscribe_to_lvl0(self, lvl0_input: Callable[[MultiJState], Any]):
        async def sensor_loop():
            nonlocal lvl0_input
            while 1:
                js = await self.lvl0_sub.listen_all()
                lvl0_input(js)

        asyncio.create_task(sensor_loop())

    def subscribe_to_lvl2(self, lvl2_input: Callable[[MultiJState], Any]):
        async def sensor_loop():
            nonlocal lvl2_input
            while 1:
                js = await self.lvl2_sub.listen_all()
                lvl2_input(js)

        asyncio.create_task(sensor_loop())

    def publish_to_lvl0(self, states: Dict[str, JState]):
        if len(states) <= 0:
            return
        logger.debug(f"publishing to lvl0: {states}")
        self.lvl0_pub.publish(states)

    def publish_to_lvl2(self, states: Dict[str, JState]):
        if len(states) <= 0:
            return
        logger.debug(f"publishing to lvl2: {states}")
        self.lvl2_pub.publish(states)

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
