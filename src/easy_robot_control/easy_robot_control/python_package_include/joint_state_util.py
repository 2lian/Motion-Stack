from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from rclpy.clock import Time
from sensor_msgs.msg import JointState

from easy_robot_control.EliaNode import rosTime2Float


@dataclass
class JState:
    name: Optional[str]
    position: Optional[float] = None
    velocity: Optional[float] = None
    effort: Optional[float] = None
    time: Optional[Time] = None


def js_from_ros(jsin: JointState) -> List[JState]:
    areAngle = len(jsin.position) > 0
    areVelocity = len(jsin.velocity) > 0
    areEffort = len(jsin.effort) > 0

    stamp: Optional[Time]
    if jsin.header.stamp is None:
        stamp = None
    else:
        stamp = Time.from_msg(jsin.header.stamp)

    nothingInside = not (areAngle or areVelocity or areEffort)
    if nothingInside:
        return []
    js_out: List[JState] = []
    for index, name in enumerate(jsin.name):
        js = JState(name=name, time=stamp)

        if areAngle:
            js.position = jsin.position[index]
        if areVelocity:
            js.velocity = jsin.velocity[index]
        if areEffort:
            js.effort = jsin.effort[index]
        # self.pwarn(js)

        js_out.append(js)
    return js_out


def intersect_names(js_in: JointState, names: Sequence[str]) -> JointState:
    inters = set(js_in.name) & set(names)
    out = JointState()
    out.header = js_in.header
    out.name = []
    out.position = []
    out.velocity = []
    out.effort = []
    for name in inters:
        ind = list(js_in.name).index(name)
        out.name.append(name)
        if js_in.position:
            out.position.append(js_in.position[ind])
        if js_in.velocity:
            out.velocity.append(js_in.velocity[ind])
        if js_in.effort:
            out.effort.append(js_in.effort[ind])
    return out


def js_copy(js: JState) -> JState:
    out = JState(name="")
    for attr in ["name", "time", "position", "velocity", "effort"]:
        setattr(out, attr, getattr(js, attr, None))
    return out


def impose_state(onto: JState, fromm: JState) -> JState:
    out = JState(name="")
    for attr in ["name", "time", "position", "velocity", "effort"]:
        v1 = getattr(onto, attr, None)
        v2 = getattr(fromm, attr, None)
        if v2 is not None:
            setattr(out, attr, v2)
        else:
            setattr(out, attr, v1)
    return out

    return False


def js_changed(j1: JState, j2: JState, delta: JState) -> bool:
    d = js_diff(j1, j2)
    for attr in ["time", "position", "velocity", "effort"]:
        vd = getattr(d, attr, None)
        vdelta = getattr(delta, attr, None)
        if vdelta is None:
            continue
        if vd is None:
            return True
        if attr == "time":
            vd = vd.nanoseconds
            vdelta = vdelta.nanoseconds

        if abs(vd) >= abs(vdelta):
            return True
    return False


def js_diff(j1: JState, j2: JState) -> JState:
    assert j1.name == j2.name
    out = JState(j1.name)
    for attr in ["time", "position", "velocity", "effort"]:
        v1 = getattr(j1, attr, None)
        v2 = getattr(j2, attr, None)
        if v1 is None and v2 is None:
            if attr == "time":
                setattr(out, attr, Time(seconds=0))
            else:
                setattr(out, attr, 0.0)
        elif v1 is None or v2 is None:
            setattr(out, attr, None)
        else:
            assert not (v1 is None or v2 is None)
            setattr(out, attr, v1 - v2)
    return out


def stateOrderinator3000(allStates: List[JState]) -> List[JointState]:
    # out = [JointState() for i in range(2**3)]
    outDic: Dict[int, JointState] = {}
    for state in allStates:
        idx = 0
        if state.position is not None:
            idx += 2**0
        if state.velocity is not None:
            idx += 2**1
        if state.effort is not None:
            idx += 2**2
        # workingJS = out[idx]

        workingJS: JointState
        if not idx in outDic.keys():
            outDic[idx] = JointState()
            workingJS = outDic[idx]
            workingJS.name = []
            if state.position is not None:
                workingJS.position = []
            if state.velocity is not None:
                workingJS.velocity = []
            if state.effort is not None:
                workingJS.effort = []
        else:
            workingJS = outDic[idx]

        workingJS.name.append(state.name)
        if state.position is not None:
            workingJS.position.append(state.position)
        if state.velocity is not None:
            workingJS.velocity.append(state.velocity)
        if state.effort is not None:
            workingJS.effort.append(state.effort)

    withoutNone: List[JointState] = list(outDic.values())
    return withoutNone
