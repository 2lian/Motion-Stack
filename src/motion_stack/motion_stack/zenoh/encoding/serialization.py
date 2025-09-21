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


from motion_stack.core.utils.joint_state import JState
from motion_stack.zenoh.encoding import joint_state
from motion_stack.zenoh.encoding.general import Encodable, Encoded, Method

ruleset: Dict[Type, Method] = {
    JState: joint_state.DEFAULT,
}

_decode_ruleset = {v.encoding: v for k, v in ruleset.items()}


def serialize(data: Encodable) -> Encoded:
    meth = ruleset[type(data)]
    return meth.serializer(data)


def parse(data: Encoded) -> Encodable:
    meth = _decode_ruleset[data.encoding]
    return meth.parser(data)
