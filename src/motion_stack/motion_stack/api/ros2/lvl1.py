from abc import ABC, abstractmethod
from typing import List

import numpy as np
import rclpy
from motion_stack_msgs.srv import ReturnJointState
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState

from ...core.lvl1_joint import JointNode
from ...core.utils.joint_state import JState
from ...core.utils.printing import TCOL
from ...ros2.utils.executor import Ros2Spinner, error_catcher
from ...ros2.utils.joint_state import joint_state_pub, ros2js_wrap


class Lvl1Node(Node):
    def __init__(self):
        super().__init__("lvl1")
        rclpy.init()
        self._spinner = Ros2Spinner(self)
        self.lvl1 = JointNode(self._spinner)
        self.subscribe_to_lvl0()
        self.subscribe_to_lvl2()
        self.publish_to_lvl0()
        self.publish_to_lvl2()
        self.frequently_send_to_lvl2()
        self._on_startup()
        self._spinner.wait_for_lower_level()

    def subscribe_to_lvl0(self):
        callback = ros2js_wrap(self.lvl1.coming_from_lvl0)
        self.create_subscription(JointState, "joint_states", callback, 10)

    def subscribe_to_lvl2(self):
        callback = ros2js_wrap(self.lvl1.coming_from_lvl2)
        self.create_subscription(JointState, "joint_set", callback, 10)

    def publish_to_lvl0(self):
        publish_func = joint_state_pub(self, "joint_commands")
        self.lvl1.send_to_lvl0_callbacks.append(publish_func)

    def publish_to_lvl2(self):
        publish_func = joint_state_pub(self, "joint_read")
        self.lvl1.send_to_lvl2_callbacks.append(publish_func)

    def frequently_send_to_lvl2(self):
        @error_catcher
        def execute():
            self.lvl1.send_sensor_up()

        self.create_timer(1 / self.lvl1.ms_param["mvmt_update_rate"], execute)

    def startup_action(self):
        self.lvl1.send_to_lvl0(
            [
                JState(time=self.lvl1.now(), name=n)
                for n in self.lvl1.jointHandlerDic.keys()
            ]
        )

    def _on_startup(self):
        future = Future()

        @error_catcher
        def startup_callback():
            if future.done():
                return
            self.startup_action()
            future.set_result(True)

        timer = self.create_timer(1, startup_callback)
        future.add_done_callback(lambda fut: self.destroy_timer(timer))

    def spin(self):
        executor = SingleThreadedExecutor()  # better perf
        executor.add_node(self)
