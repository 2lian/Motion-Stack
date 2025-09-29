import logging
from typing import Optional, TypeVar

import zenoh

from motion_stack.zenoh.encoding.general import Encodable, Encoded
from motion_stack.zenoh.encoding.serialization import parse, serialize

from ...core.utils.async_base import BasePub, BaseSub
from .auto_session import auto_session

logger = logging.getLogger(__name__)


class RawSub(BaseSub[zenoh.Sample]):
    def __init__(
        self, key_expr: str, session: Optional[zenoh.Session] = None, buff_size: int = 10
    ) -> None:
        self.session = self._resolve_session(session)
        self.sub = self._resolve_sub(key_expr)
        super().__init__(buff_size)

    @property
    def name(self) -> str:
        return f"raw-{self.sub.key_expr}"

    def _resolve_session(self, session: Optional[zenoh.Session]) -> zenoh.Session:
        return auto_session(session)

    def _resolve_sub(self, key_expr: str):
        return self.session.declare_subscriber(
            key_expr=key_expr, handler=self._unsafe_input_callback
        )

    def _unsafe_input_callback(self, sample: zenoh.Sample):
        try:
            healty = self.input_data(sample)
            if not healty:
                self.sub.undeclare()
        except Exception as e:
            logger.error(e)


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
        self.raw_sub = RawSub(key_expr, session, buff_size)
        super().__init__(buff_size)
        self.raw_sub.asap_callback.append(self._unsafe_input_callback)

    @property
    def name(self) -> str:
        return f"parsed-{self.raw_sub.sub.key_expr}"

    def _unsafe_input_callback(self, sample: zenoh.Sample):
        logger.debug(sample.payload.to_string())
        data: Encodable = parse(sample)
        self._new_message_cbk(data)

    def _new_message_cbk(self, msg: Encodable):
        logger.info(msg)
        return super()._new_message_cbk(msg)

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
        self.session = auto_session(session)
        self.zenoh_pub = self.session.declare_publisher(key_expr)
        logger.debug("declared pub %s", self.zenoh_pub.key_expr)


    def pub(self, data: Encodable):
        # logger.debug("Publishing on %s ", self.zenoh_pub.key_expr)
        self.zenoh_pub.put(**(serialize(data)._asdict()))
