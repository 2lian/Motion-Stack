import json
from dataclasses import asdict
from typing import Any, Callable, Dict, List, Optional

import zenoh

from motion_stack.zenoh.encoding.general import encode
from motion_stack.zenoh.utils.auto_session import auto_session

from ...core.lvl1_joint import JointCore
from ...core.utils.joint_state import JState, impose_state, js_changed
from ...core.utils.time import Time


class JSBuffer:
    def __init__(self, delta: JState) -> None:
        self.delta: JState = delta
        if self.delta.time is None:
            self.delta.time = Time(0)
        self.last_sent: Dict[str, JState] = dict()
        self.accumulated: Dict[str, JState] = dict()
        self.new: Dict[str, JState] = dict()
        self.urgent: Dict[str, JState] = dict()

    def _accumulate(self, states):
        for name, js in states.items():
            self.accumulated[name] = impose_state(self.accumulated.get(name), js)

    def push(self, states: Dict[str, JState]):
        assert self.delta.time is not None
        self.new.update(states)
        self._accumulate(states)
        delta = self.delta.copy()
        for name, js in states.items():
            last = self.last_sent.get(name)
            if last is None:
                self.urgent[name] = js
                continue
            if (
                name in self.urgent.keys()
                or last.time is None
                or last.time == 0
                or self.delta.time.nano() <= 1
            ):
                has_changed = True
            else:
                delta.time = Time(self.delta.time - last.time % self.delta.time)
                has_changed = js_changed(last, js, self.delta)
            if has_changed:
                self.urgent[name] = js

    def pull(self):
        urgent = self.urgent
        new = self.new
        self.urgent = dict()
        self.new = dict()
        return urgent, new


class JointStatePub:
    def __init__(
        self,
        key: str = "ms",
        session: Optional[zenoh.Session] = None,
        # buffer: Callable[[Dict[str, JState]], Dict[str, JState]] = None,
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

    def publish(self, states: Dict[str, JState]):
        for name, js in states.items():
            pub = self._zenoh_pubs.get(js.name)
            if pub is None:
                pub = self._session.declare_publisher(
                    f"{self.key}/{js.name}",
                    reliability=zenoh.Reliability.RELIABLE,
                )
                self._zenoh_pubs[js.name] = pub
            pub.put(**(encode(js)._asdict()))
