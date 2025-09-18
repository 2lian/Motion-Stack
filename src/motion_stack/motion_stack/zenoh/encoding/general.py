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
    Union
)


from motion_stack.core.utils.joint_state import JState
from motion_stack.zenoh.encoding import joint_state

_T = TypeVar("_T")

Encodable = Union[JState]


class Encoded(NamedTuple):
    payload: str
    encoding: str


class Method(NamedTuple, Generic[_T]):
    serializer: Callable[[_T], Encoded]
    parser: Callable[[Encoded], _T]
    encoding: str


ruleset: Dict[Type, Method] = {
    JState: joint_state.DEFAULT,
}

_decode_ruleset = {v.encoding: v for k,v in ruleset.items()}

def encode(data: Encodable) -> Encoded:
    meth = ruleset[type(data)]
    return meth.serializer(data)

def decode(data: Encoded) -> Encodable:
    meth = _decode_ruleset[data.encoding]
    return meth.parser(data)
