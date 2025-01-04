from typing import Any, Callable

from rclpy.node import Node
from rclpy.task import Future

from .executor import error_catcher


def link_startup_action(node: Node, startup_callback: Callable, argument: Any) -> Future:
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
