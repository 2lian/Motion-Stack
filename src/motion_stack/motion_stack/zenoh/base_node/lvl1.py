import asyncio
import logging
from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, List, Union

from motion_stack.core.lvl1_joint import JointCore
from motion_stack.core.utils.joint_state import (
    JState,
    JStateBuffer,
    MultiJState,
    multi_to_js_dict,
)
from motion_stack.core.utils.static_executor import PythonSpinner
from motion_stack.core.utils.time import Time
from motion_stack.zenoh.utils.communication import JointStatePub, JointStateSub


class Lvl1Node(ABC):
    def __init__(self):
        logging.debug("lvl1 node instanciated")
        self._link_publishers()
        self._link_subscribers()
        self._link_timers()
        self._link_sensor_check()
        self._on_startup()

        self.lvl0_pub = JointStatePub(key="TODO/lvl0/joint_commands")
        self.lvl0_sub = JointStateSub(key="TODO/lvl0/joint_states/**")
        self.lvl2_pub = JointStatePub(key="TODO/lvl2/joint_read")
        self.lvl2_sub = JointStateSub(key="TODO/lvl2/joint_set/**")
        self.sensor_buf = JStateBuffer(JState("", time=Time(sec=0.5)))
        self.command_buf = JStateBuffer(JState("", time=Time(sec=0.5)))

        self.d_required = set()
        self.d_displayed = set()
        logging.debug("lvl1 node initialized")

    # @abstractmethod
    def subscribe_to_lvl0(self, lvl0_input: Callable[[MultiJState], Any]):
        async def sensor_loop():
            nonlocal lvl0_input
            while 1:
                js = await self.lvl0_sub.listen_all()
                lvl0_input(js)

        asyncio.create_task(sensor_loop())

    # @abstractmethod
    def subscribe_to_lvl2(self, lvl2_input: Callable[[MultiJState], Any]):
        async def sensor_loop():
            nonlocal lvl2_input
            while 1:
                js = await self.lvl2_sub.listen_all()
                lvl2_input(js)

        asyncio.create_task(sensor_loop())

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
                logging.debug(f"lvl0->2 urgent: {set(is_urgent.keys())}")
                asyncio.get_event_loop().call_later(0.001, flush_to_lvl2)
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
                logging.debug(f"lvl2->0 urgent: {set(is_urgent.keys())}")
                asyncio.get_event_loop().call_later(0.001, flush_to_lvl0)
                command_flush_scheduled = True

        self.subscribe_to_lvl2(lvl2_to_lvl0)

    # @abstractmethod
    def publish_to_lvl0(self, states: Dict[str, JState]):
        if len(states) <= 0:
            return
        logging.debug(f"publishing to lvl0: {states}")
        self.lvl0_pub.publish(states)

    # @abstractmethod
    def publish_to_lvl2(self, states: Dict[str, JState]):
        if len(states) <= 0:
            return
        logging.debug(f"publishing to lvl2: {states}")
        self.lvl2_pub.publish(states)

    def _link_publishers(self): ...

    # @abstractmethod
    def frequently_send_to_lvl2(self, send_function: Callable[[], None]):
        async def periodic():
            while 1:
                send_function()
                await asyncio.sleep(5)

        asyncio.create_task(periodic())

    def _link_timers(self):
        def send():
            b = self.sensor_buf
            to_send = b.pull_new()
            self.publish_to_lvl2(to_send)

        self.frequently_send_to_lvl2(send)

    def _link_sensor_check(self):

        async def happy_new_state():
            while 1:
                b = self.sensor_buf
                available = set(b.accumulated.keys())
                new = self.d_displayed - available
                print(f"YEY new: {new}")
                await asyncio.sleep(1 / 3)

        asyncio.create_task(happy_new_state())

        async def angry_new_state():
            b = self.sensor_buf
            available = set(b.accumulated.keys())
            missing = self.d_required - available
            print(f"OH NYO missing: {missing}")

        asyncio.get_event_loop().call_later(3, angry_new_state)

    # @abstractmethod
    def startup_action(self): ...

    def _on_startup(self):
        asyncio.get_event_loop().call_soon(self.startup_action)

    def close(self):
        self.lvl0_sub.close()
        self.lvl0_pub.close()
        self.lvl2_pub.close()
        self.lvl2_sub.close()


def main():
    loop = asyncio.get_event_loop()
    node = Lvl1Node()
    loop.run_forever()


if __name__ == "__main__":
    main()
