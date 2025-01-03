from typing import Any, Callable, Dict, Iterable, List, Optional, Union

from rclpy import Node
from rclpy.time import Time as TimeRos
from sensor_msgs.msg import JointState

from ...core.utils.joint_state import Jdata, Jstamp, JState, Time, js_from_dict_list


def ros2js_wrap(callback: Callable[[List[JState]], Any]) -> Callable[[JointState], None]:
    def wrap(msg: JointState) -> None:
        js = ros2js(msg)
        callback(js)

    return wrap


def ros2js(jsin: JointState) -> List[JState]:
    leng = len(jsin.name)
    timestamp = Time(nano=TimeRos.from_msg(jsin.header.stamp).nanoseconds)
    jdict: Dict[Union[Jdata, Jstamp], List] = {
        "time": [timestamp] * leng,
        "name": list(jsin.name),
        "position": list(jsin.position),
        "velocity": list(jsin.velocity),
        "effort": list(jsin.effort),
    }
    return js_from_dict_list(jdict)


def make_joint_state_pub(
    node: Node, topic_name: str, **kwargs
) -> Callable[[List[JState]], Any]:
    pub = node.create_publisher(JointState, topic_name, 10, **kwargs)

    def publisher_func(states: List[JState]):
        msgs = stateOrderinator3000(states)
        stamp = states[0].time.nano() if states[0].time is not None else 0
        stamp = TimeRos(nanoseconds=stamp).to_msg()
        for msg in msgs:
            msg.header.stamp = stamp
            pub.publish(msg)

    return publisher_func

def stateOrderinator3000(allStates: Iterable[JState]) -> List[JointState]:
    """Converts a list  of JState to multiple ros JointStates messages.
    Timestamp ignored."""
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
