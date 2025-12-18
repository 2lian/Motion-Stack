import asyncio
import json
from asyncio.queues import QueueFull
from dataclasses import asdict
import logging
from threading import Lock
from typing import Any, Callable, Dict, List, Optional, Union

import zenoh
import asyncio_for_robotics.zenoh as afor
from colorama import Fore

from motion_stack.zenoh.encoding.general import Encoded, Method
from motion_stack.zenoh.encoding.serialization import parse, serialize

from ...core.lvl1_joint import JointCore
from ...core.utils.joint_state import JState, JStateBuffer, MultiJState, impose_state, js_changed, multi_to_js_dict
from ...core.utils.time import Time

logger = logging.getLogger(__name__)

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
        self.key = key
        self._session = afor.auto_session(session)
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
                logger.debug(f"Declaring lazy pub %s", pub.key_expr)
                self._zenoh_pubs[name] = pub
            pub.put(**(serialize(js)._asdict()))

    def close(self):
        for p in self._zenoh_pubs.values():
            logger.debug("Undeclaring pub %s", p.key_expr)
            p.undeclare()
