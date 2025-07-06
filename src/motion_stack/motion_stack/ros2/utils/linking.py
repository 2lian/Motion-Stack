from typing import Any, Callable, Union

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.task import Future

from motion_stack.ros2 import communication

from .executor import error_catcher


class CallablePublisher:
    def __init__(
        self,
        node: Node,
        topic_type: type,
        topic_name: str,
        qos_profile: Union[QoSProfile, int] = communication.DEFAULT_QOS,
        *args,
        **kwargs
    ):
        self.__type = topic_type
        self.pub = node.create_publisher(
            topic_type, topic_name, qos_profile, *args, **kwargs
        )

    def __call__(self, msg) -> None:
        assert isinstance(msg, self.__type)
        self.pub.publish(msg)


def link_startup_action(
    node: Node, startup_callback: Callable, argument: Any
) -> Future:
    """Creates a callback to be execute on the node's startup given arguments.

    Args:
        node: spinning node
        lvl1: lvl1 core
    """
    future = Future()

    @error_catcher
    def execute():
        if future.done():
            return
        startup_callback(argument)
        future.set_result(True)

    timer = node.create_timer(1, execute)
    future.add_done_callback(lambda fut: node.destroy_timer(timer))
    return future
