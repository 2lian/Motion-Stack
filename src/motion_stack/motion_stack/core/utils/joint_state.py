from dataclasses import dataclass, replace
from typing import (
    Any,
    Dict,
    Final,
    Iterable,
    List,
    Literal,
    Optional,
    Sequence,
    Set,
    Tuple,
    Union,
    overload,
)

from .time import Time

Jstamp = Literal["name", "time"]
jstamp: Set[Jstamp] = {"name", "time"}
Jdata = Literal["position", "velocity", "effort"]
jdata: Set[Jdata] = {"position", "velocity", "effort"}
jattr = jdata | jstamp


@dataclass(eq=True, order=True)
class JState:
    name: str
    time: Optional[Time] = None
    position: Optional[float] = None
    velocity: Optional[float] = None
    effort: Optional[float] = None

    @overload
    def getattr(self, name: Literal["name"]) -> str: ...

    @overload
    def getattr(self, name: Literal["time"]) -> Time: ...

    @overload
    def getattr(self, name: Jdata) -> float: ...

    def getattr(self, name: str) -> Any:
        return getattr(self, name, None)

    def copy(self) -> "JState":
        return replace(self)

    @property
    def is_initialized(self) -> bool:
        for attr in jdata:
            val = self.getattr(attr)
            if val is not None:
                return True
        return False


def js_from_dict_list(dil: Dict[Union[Jdata, Jstamp], List]) -> List[JState]:
    lengths = {len(v) for k, v in dil.items() if k in jattr} - {0}
    assert len(lengths) <= 1, f"non-empty lists are of different lengths {lengths}"

    names = dil.get("name")
    if names is None:
        return []
    out: List[JState] = [JState(n) for n in names]

    for i, state in enumerate(out):
        for attr in jdata | (jstamp - {"name"}):
            value: Union[None, List] = dil.get(attr)
            if not value:
                continue
            state.__setattr__(attr, value[i])
    return out


def impose_state(onto: Optional[JState], fromm: Optional[JState]) -> JState:
    if onto is None and fromm is None:
        return JState(name="") 
    if onto is None:
        return fromm.copy()
    if fromm is None:
        return onto.copy()

    out = JState(name="")
    for attr in jattr:
        v1 = onto.getattr(attr)
        v2 = fromm.getattr(attr)
        if v2 is not None:
            out.__setattr__(attr, v2)
        else:
            out.__setattr__(attr, v1)
    return out


def js_changed(j1: JState, j2: JState, delta: JState) -> bool:
    d = js_diff(j1, j2)
    for attr in jattr - {"name"}:
        vd = getattr(d, attr, None)
        vdelta = getattr(delta, attr, None)
        if vdelta is None:
            continue
        if vd is None:
            return True

        if abs(vd) >= abs(vdelta):
            return True
    return False


def js_diff(j1: JState, j2: JState) -> JState:
    assert j1.name == j2.name
    out = JState(j1.name)
    for attr in jattr - {"name"}:
        v1 = getattr(j1, attr, None)
        v2 = getattr(j2, attr, None)
        if v1 is None and v2 is None:
            if attr == "time":
                setattr(out, attr, 0)
            else:
                setattr(out, attr, 0.0)
        elif v1 is None or v2 is None:
            setattr(out, attr, None)
        else:
            assert not (v1 is None or v2 is None)
            setattr(out, attr, v1 - v2)
    return out
