from typing import Any, Callable, List

import numpy as np
from motion_stack_msgs.srv import ReturnJointState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from ...core.lvl1_joint import JointCore
from ...core.utils.joint_state import JState
from ..base_node.lvl1 import Lvl1Node
from ..communication import lvl1 as comms
from ..utils.conversion import time_to_ros
from ..utils.executor import error_catcher
from ..utils.joint_state import CallablePublisher, JSCallableWrapper, ros2js_wrap


class DefaultLvl1(Lvl1Node):
    """Default implementation of the Joint node of lvl1.

    Refer to :py:class:`.ros2.base_node.lvl1` for documentation on linking ros2 and python core of lvl1. This only makes use of this base to create the default implementation and give an example.

    **Publishers:**

    ==============  ==========  ====
    Topic           Type        Note
    ==============  ==========  ====
    joint_commands  JointState  sent to motors (lvl0)
    joint_read      JointState  sent to IK (lvl2)
    ==============  ==========  ====

    **Subscribers:**

    ==============  ==========  ====
    Topic           Type        Note
    ==============  ==========  ====
    joint_states    JointState  coming from sensors (lvl0)
    joint_set       JointState  coming from IK (lvl2)
    ==============  ==========  ====

    **Service server:**

    ================ ================  ====
    Topic            Type              Note
    ================ ================  ====
    advertise_joints ReturnJointState  JointState with the name of all joints
    ================ ================  ====

    **Timers:**
        - Sends to lvl2, freq.= ROS2_PARAMETER[``mvmt_update_rate``].

    **Startup:**
        - Sends empty message to lvl0 with only joint names.
    """

    alive_srv = comms.alive

    def __init__(self):
        super().__init__()
        raw_publisher: Callable[[JointState], None] = CallablePublisher(
            node=self,
            topic_type=comms.output.motor_command.type,
            topic_name=comms.output.motor_command.name,
        )
        self.wrapped_pub_lvl0 = JSCallableWrapper(raw_publisher)
        raw_publisher: Callable[[JointState], None] = CallablePublisher(
            node=self,
            topic_type=comms.output.joint_state.type,
            topic_name=comms.output.joint_state.name,
        )
        self.wrapped_pub_lvl2 = JSCallableWrapper(raw_publisher)
        create_advertise_service(self, self.core)

    def subscribe_to_lvl2(self, lvl2_input: Callable[[List[JState]], Any]):
        """"""
        self.create_subscription(
            comms.input.joint_target.type,
            comms.input.joint_target.name,
            ros2js_wrap(lvl2_input),
            10,
        )

    def subscribe_to_lvl0(self, lvl0_input: Callable[[List[JState]], Any]):
        """"""
        self.create_subscription(
            comms.input.motor_sensor.type,
            comms.input.motor_sensor.name,
            ros2js_wrap(lvl0_input),
            10,
        )

    def publish_to_lvl0(self, states: List[JState]):
        """"""
        self.wrapped_pub_lvl0(states)

    def publish_to_lvl2(self, states: List[JState]):
        """"""
        self.wrapped_pub_lvl2(states)

    def frequently_send_to_lvl2(self, send_function: Callable[[], None]):
        """"""
        tmr = self.create_timer(1 / self.core.ms_param["mvmt_update_rate"], send_function)
        # self.core.error(tmr.timer_period_ns/1e9)

    def startup_action(self, core: JointCore):
        """"""
        core.send_empty_command_to_lvl0()
        # self.get_logger().error("hey")
        self.create_service(self.alive_srv.type, self.alive_srv.name, lambda req, res: res)


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
        names: List[str] = [h._sensor.name for h in lvl1.jointHandlerDic.values()]
        none2nan = lambda x: x if x is not None else np.nan
        res.js = JointState(
            name=names,
            position=[none2nan(h._sensor.position) for h in lvl1.jointHandlerDic.values()],
            velocity=[none2nan(h._sensor.velocity) for h in lvl1.jointHandlerDic.values()],
            effort=[none2nan(h._sensor.effort) for h in lvl1.jointHandlerDic.values()],
        )

        res.js.header.stamp = time_to_ros(lvl1.now()).to_msg()
        return res

    node.create_service(comms.output.advertise.type, comms.output.advertise.name, cbk)


def main(*args, **kwargs):
    DefaultLvl1.spin()
