"""
ROS2 node handling lvl1
"""

from typing import Any, Callable, List

import numpy as np
from motion_stack_msgs.srv import ReturnJointState, TFService
from rclpy import Future, Node
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time as TimeRos
from sensor_msgs.msg import JointState

from motion_stack.core.utils.static_executor import FlexNode

from ..core.lvl1_joint import JointNode
from .utils.executor import Ros2Spinner
from .utils.joint_state import (
    JState,
    make_joint_state_pub,
    ros2js,
    ros2js_wrap,
    stateOrderinator3000,
)


def make_advertise_service(node: Node, lvl1: JointNode):

    def cbk(
        req: ReturnJointState.Request, res: ReturnJointState.Response
    ) -> ReturnJointState.Response:
        names: List[str] = [h.sensor.name for h in lvl1.jointHandlerDic.values()]
        none2nan = lambda x: x if x is not None else np.nan
        res.js = JointState(
            name=names,
            position=[none2nan(h.sensor.position) for h in lvl1.jointHandlerDic.values()],
            velocity=[none2nan(h.sensor.velocity) for h in lvl1.jointHandlerDic.values()],
            effort=[none2nan(h.sensor.effort) for h in lvl1.jointHandlerDic.values()],
        )

        res.js.header.stamp = node.get_clock().now().to_msg()
        return res

    ser = node.create_service(ReturnJointState, "advertise_joints", cbk)


def make_all_publishers(node: Node, lvl1: JointNode):
    lvl1.send_to_lvl0_callbacks.append(make_joint_state_pub(node, "joint_command"))
    lvl1.send_to_lvl2_callbacks.append(make_joint_state_pub(node, "joint_read"))


def make_all_subscribers(node: Node, lvl1: JointNode):
    node.create_subscription(
        JointState, "joint_state", ros2js_wrap(lvl1.coming_from_lvl0), 10
    )
    node.create_subscription(
        JointState, "joint_set", ros2js_wrap(lvl1.coming_from_lvl2), 10
    )


def initialise_on_first_spin(node: Node, flex: type):
    future = Future()

    def cbk():
        flex(Ros2Spinner(node))
        future.done()

    timer = node.create_timer(1, cbk)
    future.add_done_callback(node.destroy_timer(timer))


def main(*args, **kwargs):
    rclpy.init()
    node = Node("lvl1")
    spinner = Ros2Spinner(node)
    spinner.wait_for_lower_level()
    lvl1 = JointNode(spinner)
    make_all_publishers(node, lvl1)
    make_all_subscribers(node, lvl1)
    make_advertise_service(node, lvl1)

    executor = SingleThreadedExecutor()  # better perf
    executor.add_node(node)
    try:
        node.destroy_node()
    except:
        pass
    try:
        rclpy.shutdown()
    except:
        pass





if __name__ == "__main__":
    main()
