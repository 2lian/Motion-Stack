import asyncio
import json
from asyncio.queues import QueueFull
from dataclasses import asdict
import logging
from threading import Lock
from typing import Any, Callable, Dict, List, Optional, Union

import zenoh
from colorama import Fore

from motion_stack.zenoh.encoding.general import Encoded, Method
from motion_stack.zenoh.encoding.serialization import parse, serialize
from motion_stack.zenoh.utils.auto_session import auto_session

from ...core.lvl1_joint import JointCore
from ...core.utils.joint_state import JState, JStateBuffer, MultiJState, impose_state, js_changed, multi_to_js_dict
from ...core.utils.time import Time


class JointStatePub:
    def __init__(
        self,
        key: str = "ms",
        session: Optional[zenoh.Session] = None,
    ) -> None:
        """Serializes and publishes JState data onto a key-expression

        Joint names will be appended to the key as such: ``key + "/" +
        JState.name``.

        Publishers will be created lazily as needed

        Args:
            key: key to publish onto (example: ms/joint_state)
            session: zenoh session to use (if None automatically create and use one shared with all other objects)
        """
        self._session: zenoh.Session
        self.key = key
        if session is None:
            self._session = auto_session()
        else:
            self._session = session
        #: lazy creation of the publishers in a dict
        self._zenoh_pubs: Dict[str, zenoh.Publisher] = dict()

    def publish(self, states: MultiJState):
        states = multi_to_js_dict(states)
        for name, js in states.items():
            pub = self._zenoh_pubs.get(name)
            if pub is None:
                pub = self._session.declare_publisher(
                    f"{self.key}/{name}",
                    reliability=zenoh.Reliability.RELIABLE,
                )
                logging.debug(f"new lazy pub {pub.key_expr}")
                self._zenoh_pubs[name] = pub
            pub.put(**(serialize(js)._asdict()))

    def close(self):
        for p in self._zenoh_pubs.values():
            p.undeclare()


class JointStateSub:
    def __init__(
        self,
        key: str = "ms",
        session: Optional[zenoh.Session] = None,
        queue: Optional[asyncio.Queue] = None,
    ) -> None:
        self._session: zenoh.Session
        self.key = key
        if session is None:
            self._session = auto_session()
        else:
            self._session = session
        if queue is None:
            self.queue: asyncio.Queue[JState] = asyncio.Queue()
        else:
            self.queue = queue
        self._even_loop = asyncio.get_event_loop()
        self._sub = self._session.declare_subscriber(key, self._thrd_callback)

    def _add_to_queue(self, js: JState):
        try:
            self.queue.put_nowait(js)
        except QueueFull:
            print(
                f"{Fore.YELLOW}WARNING [{self.key}]: "
                f"joint queue full. dropping{Fore.RESET}"
            )

    def _thrd_callback(self, sample: zenoh.Sample):
        enc = Encoded(payload=sample.payload.to_string(), encoding=sample.encoding)  # type: ignore
        js: JState = parse(enc)

        assert not self._even_loop.is_closed()
        self._even_loop.call_soon_threadsafe(self._add_to_queue, js)

    async def listen(self) -> JState:
        """Remove and return an item from the queue.

        If queue is empty, wait until an item is available.
        """
        return await self.queue.get()

    async def listen_all(self) -> List[JState]:
        """Remove and return an item from the queue.

        If queue is empty, wait until an item is available.
        """
        # return [await self.queue.get()]
        acc: List[JState] = []
        while 1:
            js= await self.queue.get()
            acc.append(js)
            if self.queue.empty():
                return acc

    def close(self):
        self._sub.undeclare()
