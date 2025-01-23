from functools import wraps
from typing import Any, Callable, Dict, Iterable, List, Optional, Union

from rclpy.node import Node
from rclpy.time import Time as TimeRos
from sensor_msgs.msg import JointState

from ...core.utils.joint_state import Jdata, Jstamp, JState, Time, js_from_dict_list
from .executor import error_catcher
from .linking import CallablePublisher


def link_subscription(
    node: Node, topic_name: str, callback: Callable[[List[JState]], Any]
):
    """subscribes to a JointState topic, converts the message then calls the callback.

    Args:
        node: node spinning
        topic_name: name of the JointState topic
        callback: callback using not JointState but List[JState] and input
    """
    node.create_subscription(JointState, topic_name, ros2js_wrap(callback), 10)


def ros2js_wrap(func: Callable[[List[JState]], Any]) -> Callable[[JointState], None]:
    """
    Args:
        callback:
            function with List[JState] as the input

    Returns:
        function with JointState as the input
    """

    @wraps(func)
    @error_catcher
    def wrap(msg: JointState) -> None:
        js = ros2js(msg)
        func(js)

    return wrap


def ros2js(jsin: JointState) -> List[JState]:
    """Converts JointState to a List[JState]"""
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


class JSCallableWrapper:
    def __init__(self, original_callable: Callable[[JointState], None]):
        self._original_callable = original_callable

    def __call__(self, states: List[JState]) -> None:
        if not states:
            return
        msgs = stateOrderinator3000(states)
        stamp = states[0].time.nano() if states[0].time is not None else 0
        stamp = TimeRos(nanoseconds=stamp).to_msg()
        for msg in msgs:
            msg.header.stamp = stamp
            self._original_callable(msg)

    def __getattr__(self, name):
        # Delegate attribute access to the original callable
        return getattr(self._original_callable, name)


def callable_js_publisher(
    node: Node, topic_name: str, **kwargs
) -> Callable[[List[JState]], Any]:
    """Creates a function publishing a JState on ROS2.

    You can then call the function directly with a List[JState] when you wanna send something.

    Args:
        node: node handling the publisher
        topic_name: publisher name
        **kwargs: kwargs for node.create_publisher

    Returns:
        A function, converting List[JState] to (several) JointState, then publishing

    """
    pub = CallablePublisher(node, JointState, topic_name, 10)

    @error_catcher
    def publisher_func(states: List[JState]):
        if not states:
            return
        msgs = stateOrderinator3000(states)
        stamp = states[0].time.nano() if states[0].time is not None else 0
        stamp = TimeRos(nanoseconds=stamp).to_msg()
        for msg in msgs:
            msg.header.stamp = stamp
            pub.pub.publish(msg)

    pub.__call__ = publisher_func

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
