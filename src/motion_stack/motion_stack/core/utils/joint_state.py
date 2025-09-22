import logging
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
    """Data of one joint

    Attributes:
        name:
        time: time of the measurement or command
        position: rad (can go above +-pi)
        velocity: rad/s
        effort: N.m
    """

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


MultiJState = Union[Dict[str, JState], List[JState], JState]


def multi_to_js_dict(states: MultiJState) -> Dict[str, JState]:
    if isinstance(states, list):
        return {js.name: js for js in states}
    elif isinstance(states, dict):
        return states
    else:
        return {states.name: states}


def js_from_dict_list(dil: Dict[Union[Jdata, Jstamp], List]) -> List[JState]:
    """Converts a dictionary of lists into a list of JState"""
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
    """Replaces values of onto with values fromm, unless from is None."""
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
    """checks if the difference is above delta on each field except name"""
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
    """Difference between two JS"""
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


class JStateBuffer:
    def __init__(self, delta: JState) -> None:
        """Accumulate joint measurements, only passing through data that
        changed by more than delta.

        Also skips data that is in the past of a previous data.

        Args:
            delta: Changes since last pull that should mark the joint as urgent.
        """
        #: can be changed at runtime
        self.delta: JState = delta
        if self.delta.time is None:
            self.delta.time = Time(0)
        #: last pulled data
        self.last_sent: Dict[str, JState] = dict()
        #: all data accumulated since start
        self.accumulated: Dict[str, JState] = dict()
        self._new: Dict[str, JState] = dict()
        self._urgent: Dict[str, JState] = dict()

    @staticmethod
    def _accumulate(states: Dict[str, JState], onto: Dict[str, JState]):
        """accumulates data since the beggining and replaces None with data"""
        for name, js in states.items():
            onto[name] = impose_state(onto.get(name), js)

    def push(self, states: MultiJState):
        """Pushes data into the buffer"""
        states = multi_to_js_dict(states)
        not_old = {}
        for k, v in states.items():
            if v.time is None or v.time == 0:
                not_old[k] = v
                continue
            acc = self.accumulated.get(k)
            if acc is None:
                not_old[k] = v
                continue
            if acc.time is None or acc.time == 0:
                not_old[k] = v
                continue
            if acc.time < v.time:
                not_old[k] = v
        if logging.getLogger().isEnabledFor(logging.DEBUG):
            never_seen = set(states.keys()) - set(self.accumulated.keys())
            if never_seen:
                logging.debug(f"new data buffered: %s", never_seen)
        self._new.update(not_old)
        self._accumulate(not_old, self.accumulated)

    @staticmethod
    def _find_urgent(
        last_sent: Dict[str, JState], new: Dict[str, JState], delta: JState
    ) -> Dict[str, JState]:
        urgent = dict()
        for name, js in new.items():
            last = last_sent.get(name)
            if last is None:
                urgent[name] = js
                continue
            if last.time is None or last.time == 0 or delta.time.nano() < 1:
                has_changed = True
            else:
                has_changed = js_changed(last, js, delta)
            if has_changed:
                urgent[name] = js
        return urgent

    def pull_urgent(self, delta=None) -> Dict[str, JState]:
        """Returns and flushes the urgent buffer of data that changed (by more
        than delta).
        """
        if delta is None:
            delta = self.delta
        urgent = self._find_urgent(self.last_sent, self._new, delta)
        if len(urgent) <= 0:
            return dict()
        for k in urgent.keys():
            del self._new[k]
        self.last_sent.update(urgent)
        logging.debug("pulling urgent: %s", urgent.keys())
        return urgent

    def pull_new(self) -> Dict[str, JState]:
        """Returns and flushes all new data"""
        if len(self._new) <= 0:
            return dict()
        new = self._new
        self._new = dict()
        new.update(self._urgent)
        self._urgent = dict()
        self.last_sent.update(new)
        logging.debug(f"pulled new: %s", new.keys())
        return new
