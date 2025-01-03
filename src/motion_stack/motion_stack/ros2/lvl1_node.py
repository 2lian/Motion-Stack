"""
ROS2 node handling lvl1
"""

from typing import Any, Callable, List

import numpy as np
import rclpy
from motion_stack_msgs.srv import ReturnJointState
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from sensor_msgs.msg import JointState

from motion_stack.core.utils.printing import TCOL

from ..core.lvl1_joint import JointNode
from .utils.executor import Ros2Spinner
from .utils.joint_state import make_joint_state_pub, ros2js_wrap


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
        executor.spin()
    except KeyboardInterrupt:
        m = f"{TCOL.OKCYAN}KeyboardInterrupt intercepted, {TCOL.OKBLUE}shuting down. :){TCOL.ENDC}"
        print(m)
        return
    except ExternalShutdownException:
        m = f"{TCOL.OKCYAN}External Shutdown Command intercepted, {TCOL.OKBLUE}shuting down. :){TCOL.ENDC}"
        print(m)
        return

    except Exception as exception:
        m = f"Exception intercepted: \033[91m{traceback.format_exc()}\033[0m"
        print(m)
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
