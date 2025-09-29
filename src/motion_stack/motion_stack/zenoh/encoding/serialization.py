from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    NamedTuple,
    Protocol,
    Self,
    Type,
    TypeVar,
    Union,
)

import zenoh

from motion_stack.core.utils.joint_state import JState
from motion_stack.zenoh.encoding import joint_state
from motion_stack.zenoh.encoding.general import Encodable, Encoded, Method

ruleset: Dict[Type, Method] = {
    JState: joint_state.DEFAULT,
    str: Method(
        serializer=lambda x: Encoded(payload=x, encoding="ms_raw_string"),
        parser=lambda x: x.payload,
        encoding="ms_raw_string",
    ),
}

_decode_ruleset = {v.encoding: v for k, v in ruleset.items()}


def serialize(data: Encodable) -> Encoded:
    meth = ruleset[type(data)]
    return meth.serializer(data)


def parse(data: Union[Encoded, zenoh.Sample]) -> Encodable:
    if isinstance(data, zenoh.Sample):
        data = Encoded(payload=data.payload.to_string(), encoding=str(data.encoding))

    meth = _decode_ruleset[data.encoding]
    return meth.parser(data)
