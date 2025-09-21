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

from zenoh import ZBytes


from motion_stack.core.utils.joint_state import JState

_T = TypeVar("_T")

Encodable = Union[JState]


class Encoded(NamedTuple):
    payload: str
    encoding: str


class Method(NamedTuple, Generic[_T]):
    serializer: Callable[[_T], Encoded]
    parser: Callable[[Encoded], _T]
    encoding: str
