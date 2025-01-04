"""
ROS2 node managing the core of lvl1.

.. Note::

    I implemented this, not in OOP style, but full imperative. This might end-up being a bad idea, very bad idea.
"""

from typing import Callable, List

import numpy as np
import rclpy
from motion_stack_msgs.srv import ReturnJointState
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState

from ..core.lvl1_joint import JointCore
from ..core.utils.joint_state import JState
from ..core.utils.printing import TCOL
from .utils.conversion import time_to_ros
from .utils.executor import Ros2Spinner, error_catcher, my_main
from .utils.joint_state import callable_js_publisher, ros2js_wrap
from .utils.linking import link_startup_action


def create_advertise_service(node: Node, lvl1: JointCore):
    """Creates the advertise_joints service and its callback.

    Callback returns a ReturnJointState.Response wich is a JointState with the name of all joints managed by the node. Other field of JointState are not meant to be used, but are filled with the latest data.

    Args:
        node: spinning node
        lvl1: lvl1 core
    """

    @error_catcher
    def cbk(
        req: ReturnJointState.Request, res: ReturnJointState.Response
    ) -> ReturnJointState.Response:
        print("srv")
        names: List[str] = [h.sensor.name for h in lvl1.jointHandlerDic.values()]
        none2nan = lambda x: x if x is not None else np.nan
        res.js = JointState(
            name=names,
            position=[none2nan(h.sensor.position) for h in lvl1.jointHandlerDic.values()],
            velocity=[none2nan(h.sensor.velocity) for h in lvl1.jointHandlerDic.values()],
            effort=[none2nan(h.sensor.effort) for h in lvl1.jointHandlerDic.values()],
        )

        res.js.header.stamp = time_to_ros(lvl1.now()).to_msg()
        return res

    node.create_service(ReturnJointState, "advertise_joints", cbk)


def link_publishers(node: Node, lvl1: JointCore):
    """Creates the publishers.

    ==============  ==========  ====
    Topic           Type        Note
    ==============  ==========  ====
    joint_commands  JointState  sent to motors (lvl0)
    joint_read      JointState  sent to IK (lvl2)
    ==============  ==========  ====

    Args:
        node: spinning node
        lvl1: lvl1 core
    """
    lvl1.send_to_lvl0_callbacks.append(callable_js_publisher(node, "joint_commands"))
    lvl1.send_to_lvl2_callbacks.append(callable_js_publisher(node, "joint_read"))


def link_subscribers(node: Node, lvl1: JointCore):
    """Creates the subscribers.

    ==============  ==========  ====
    Topic           Type        Note
    ==============  ==========  ====
    joint_states    JointState  coming from sensors (lvl0)
    joint_set       JointState  coming from IK (lvl2)
    ==============  ==========  ====

    Args:
        node: spinning node
        lvl1: lvl1 core
    """
    node.create_subscription(
        JointState, "joint_states", ros2js_wrap(lvl1.coming_from_lvl0), 10
    )
    node.create_subscription(
        JointState, "joint_set", ros2js_wrap(lvl1.coming_from_lvl2), 10
    )


def frequently_send_lvl2(node: Node, lvl1: JointCore):
    """Creates a timer to send the fresh sensor state to the IK (lvl2).

    This timer, instead of sending as soon as possible, introduces a very small latency but reduces compute load.

    .. Note::

        If the sensor data is not "fresh" (meaning it is too similar to the previous state)
        it will not be sent.

    Args:
        node: spinning node
        lvl1: lvl1 core
    """

    @error_catcher
    def execute():
        lvl1.send_sensor_up()

    node.create_timer(1 / lvl1.ms_param["mvmt_update_rate"], execute)


def startup_action(lvl1: JointCore):
    """Actions to be executed on startup.

    This sends an empty JointState with just the joint names to lvl0.

    Make it execute on startup by passing it to :py:func:`.link_startup_action`.

    example::

        rclpy.init()
        node = Node("lvl1")
        spinner = Ros2Spinner(node)
        lvl1 = JointNode(spinner)
        link_startup_action(node, lvl1, startup_action)

    Args:
        lvl1: lvl1 core
    """
    lvl1.send_to_lvl0(
        [JState(time=lvl1.now(), name=n) for n in lvl1.jointHandlerDic.keys()]
    )


def main(*args, **kwargs):
    rclpy.init()
    node = Node("lvl1")
    spinner = Ros2Spinner(node)
    lvl1 = JointCore(spinner)
    link_startup_action(node, startup_action, lvl1)
    link_publishers(node, lvl1)
    link_subscribers(node, lvl1)
    create_advertise_service(node, lvl1)
    frequently_send_lvl2(node, lvl1)
    spinner.wait_for_lower_level()

    my_main(node)

if __name__ == "__main__":
    main()
