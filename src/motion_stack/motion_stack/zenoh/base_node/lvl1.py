import asyncio
import logging
import time
from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, List, Optional, Union

from motion_stack.core.lvl1_joint import JointCore
from motion_stack.core.lvl1_pipeline import Lvl1Param, Lvl1Pipeline, lvl1_default
from motion_stack.core.utils import static_executor
from motion_stack.core.utils.joint_state import (
    BatchedAsyncioBuffer,
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
    core_type = Lvl1Pipeline

    def __init__(self, params: Lvl1Param = lvl1_default):
        logger.debug("lvl1 node instanciated")

        self.core: Lvl1Pipeline = self.core_type(params)

        logger.debug("lvl1 node initialized")

        self._link()

    def _link(self):
        self._link_publishers()
        self._link_subscribers()
        self._link_timers()
        self._on_startup()

    @abstractmethod
    def subscribe_to_lvl0(self, lvl0_input: Callable[[MultiJState], Any]): ...

    @abstractmethod
    def subscribe_to_lvl2(self, lvl2_input: Callable[[MultiJState], Any]): ...

    def _link_subscribers(self):

        self.subscribe_to_lvl0(self.core.coming_from_lvl0)
        self.subscribe_to_lvl2(self.core.coming_from_lvl2)

    @abstractmethod
    def publish_to_lvl0(self, states: Dict[str, JState]): ...

    @abstractmethod
    def publish_to_lvl2(self, states: Dict[str, JState]): ...

    def _link_publishers(self):
        self.core.send_to_lvl0_callbacks.append(self.publish_to_lvl0)
        self.core.send_to_lvl2_callbacks.append(self.publish_to_lvl2)

    @abstractmethod
    def frequently_send_to_lvl2(
        self, send_function: Callable[[], None], period: float
    ): ...

    def _link_timers(self):
        asyncio.get_event_loop().call_later(3, self.core.display_missing_joints)
        if self.core.params.joint_buffer.time is not None:
            self.frequently_send_to_lvl2(
                self.core.sensor_pipeline._batched_buf.background_flush,
                self.core.params.slow_pub_time,
            )

    def startup_action(self): ...

    def _on_startup(self):
        asyncio.get_event_loop().call_soon(self.startup_action)


def main():
    loop = asyncio.get_event_loop()
    node = Lvl1Node()
    loop.run_forever()


if __name__ == "__main__":
    main()
