from typing import Any, Callable, List

import numpy as np
from geometry_msgs.msg import Transform, TransformStamped
from motion_stack_msgs.srv import ReturnJointState
from rclpy.node import Node
from sensor_msgs.msg import JointState

from motion_stack.core.utils.pose import Pose

from ...core.utils.joint_state import JState
from ..base_node.lvl2 import Lvl2Node, IKCore
from ..communication import lvl2 as comms
from ..utils.conversion import pose_to_transform, time_to_ros, transform_to_pose
from ..utils.executor import error_catcher
from ..utils.joint_state import CallablePublisher, JSCallableWrapper, ros2js_wrap


class DefaultLvl2(Lvl2Node):
    """Default implementation of the Joint node of lvl2.

    Refer to :py:class:`.ros2.base_node.lvl2` for documentation on linking ros2 and python core of lvl1. This only makes use of this base to create the default implementation and give an example.
    """

    alive_srv = comms.alive

    def __init__(self):
        super().__init__()
        raw_publisher: Callable[[JointState], None] = CallablePublisher(
            node=self,
            topic_type=comms.output.joint_target.type,
            topic_name=comms.output.joint_target.name,
        )
        self.wrapped_lvl1_PUB = JSCallableWrapper(raw_publisher)
        self.tip_pos_PUB: Callable[[Transform], None] = CallablePublisher(
            node=self,
            topic_type=comms.output.tip_pos.type,
            topic_name=comms.output.tip_pos.name,
        )

    def subscribe_to_lvl1(self, lvl1_input: Callable[[List[JState]], Any]):
        """"""
        self.create_subscription(
            comms.input.joint_state.type,
            comms.input.joint_state.name,
            ros2js_wrap(lvl1_input),
            10,
        )

    def subscribe_to_lvl3(self, lvl3_input: Callable[[Pose], Any]):
        """"""

        def cbk(tf: Transform):
            return lvl3_input(transform_to_pose(tf, time=self.core.now()))

        self.create_subscription(
            comms.input.set_ik.type,
            comms.input.set_ik.name,
            cbk,
            10,
        )

    def publish_to_lvl1(self, states: List[JState]):
        """"""
        self.wrapped_lvl1_PUB(states)

    def publish_to_lvl3(self, pose: Pose):
        """"""
        self.tip_pos_PUB(pose_to_transform(pose))

    def startup_action(self, lvl2: IKCore):
        """"""
        self.create_service(self.alive_srv.type, self.alive_srv.name, lambda *_: None)


def main(*args, **kwargs):
    DefaultLvl2.spin()
