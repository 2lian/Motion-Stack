import logging
from typing import Callable, Dict, Generic, Optional, Type, TypeVar

import zenoh

from motion_stack.core.utils.joint_state import JState
from motion_stack.zenoh.encoding.general import Encodable, Encoded
from motion_stack.zenoh.encoding.serialization import parse, serialize
from asyncio_for_robotics.core.sub import BaseSub
from asyncio_for_robotics.zenoh import Sub as RawSub

from ...core.utils.async_base import BasePub
from .auto_session import auto_session

logger = logging.getLogger(__name__)


class ParsedSub(BaseSub[Encodable]):
    def __init__(
        self,
        key_expr: str,
        session: Optional[zenoh.Session] = None,
        buff_size: int = 10,
    ) -> None:
        """Subscriber that automatically parses the data.

        Asigne a type using ParsedSub[Type].

        Args:
            key_expr: topic to sub to
            session: zenoh session
            buff_size:
        """
        self.key_expr = key_expr
        self.raw_sub = RawSub(key_expr, session)
        super().__init__()
        self.raw_sub.asap_callback.append(self._new_data_callback)

    @property
    def name(self) -> str:
        return f"parsed-{self.key_expr}"

    def _new_data_callback(self, sample: zenoh.Sample):
        data: Encodable = parse(sample)
        self._input_data_asyncio(data)

    def close(self):
        self.raw_sub.close()


class ParsedPub(BasePub[Encodable]):
    def __init__(
        self,
        key_expr: str,
        session: Optional[zenoh.Session] = None,
    ) -> None:
        """Subscriber that automatically parses the data.

        Asigne a type using ParsedSub[Type].

        Args:
            key_expr: topic to sub to
            session: zenoh session
            buff_size:
        """
        super().__init__()
        self.key_expr = key_expr
        self.session = auto_session(session)
        self.zenoh_pub = self.session.declare_publisher(key_expr)
        logger.debug("declared pub %s", self.zenoh_pub.key_expr)

    def pub(self, data: Encodable):
        # logger.debug("Publishing on %s ", self.zenoh_pub.key_expr)
        self.zenoh_pub.put(**(serialize(data)._asdict()))

    def close(self):
        self.zenoh_pub.undeclare()

class LazyPub(Generic[Encodable]):
    def __init__(
        self,
        key_expr: str,
        session: Optional[zenoh.Session] = None,
    ) -> None:
        self._session = auto_session(session)
        self._zenoh_pubs: Dict[str, ParsedPub] = dict()
        self.key_expr = key_expr

    def pub(self, data: Encodable, subkey: str):
        pub = self._zenoh_pubs.get(subkey)
        if pub is None:
            name = self.key_expr + "/" + subkey
            pub = ParsedPub(name)
            self._zenoh_pubs[subkey] = pub
        pub.pub(data)

    def close(self):
        death = set()
        for k in self._zenoh_pubs.keys():
            self._zenoh_pubs[k].close()
            death.add(k)
        for k in death:
            del self._zenoh_pubs[k]
